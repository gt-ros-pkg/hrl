
#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <hrl_clickable_world/ClickImage.h>
#include <hrl_clickable_world/DisplayButtons.h>
#include <hrl_clickable_world/ButtonAction.h>
#include <pixel_2_3d/Pixel23d.h>

using namespace std;

namespace hrl_clickable_world {

    class DisplayManager {
        public:
            ros::Publisher button_pushed_pub, perceive_pub;
            ros::ServiceServer display_buttons_srv, clear_buttons_srv, image_click_srv;
            ros::ServiceClient button_action_srv, pixel23d_srv;
            ros::Subscriber l_image_click_sub, r_image_click_sub;
            ros::NodeHandle nhdl;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_transport::Publisher overlay_pub, button_pub;
            image_geometry::PinholeCameraModel cam_model;
            tf::TransformListener tf_listener;
            boost::shared_ptr<boost::thread> img_click_thread;
            boost::mutex img_click_lock;

            bool buttons_on;
            vector<visualization_msgs::Marker> buttons;
            vector<uint32_t> button_inds;
            vector<cv::Mat> button_rasters;

            DisplayManager();
            ~DisplayManager();
            void onInit();
            void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg);
            bool imageClickSrvCB(ClickImage::Request& req, ClickImage::Response& resp);
            void lImageClickCB(const geometry_msgs::PointStamped& msg);
            void rImageClickCB(const geometry_msgs::PointStamped& msg);
            void doLImageClickCB(const geometry_msgs::PointStamped& msg);
            bool displayButtonsCB(DisplayButtons::Request& req, DisplayButtons::Response& resp);
            void displayButtonsCB(std::vector<visualization_msgs::Marker>& buttons_msg);
            bool clearButtonsCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
        
    };

    DisplayManager::DisplayManager() : nhdl("/clickable_world"), img_trans(nhdl) {
        onInit();
    }

    DisplayManager::~DisplayManager() {
    }

    void DisplayManager::onInit() {
        buttons_on = false;
        camera_sub = img_trans.subscribeCamera<DisplayManager>
                                              ("/image", 1, 
                                               &DisplayManager::doOverlay, this);
        overlay_pub = img_trans.advertise("image_buttons", 1);
        button_pub = img_trans.advertise("image_buttons_mask", 1);
        button_pushed_pub = nhdl.advertise<std_msgs::Int32>("button_pushed", 1);
        perceive_pub = nhdl.advertise<std_msgs::Bool>("perceive_buttons", 1);
        image_click_srv = nhdl.advertiseService("click_image", 
                                                &DisplayManager::imageClickSrvCB, this);
        l_image_click_sub = nhdl.subscribe("l_mouse_click", 1, 
                                           &DisplayManager::lImageClickCB, this);
        r_image_click_sub = nhdl.subscribe("r_mouse_click", 1, 
                                           &DisplayManager::rImageClickCB, this);

        display_buttons_srv = nhdl.advertiseService("display_buttons", 
                                                &DisplayManager::displayButtonsCB, this);
        button_action_srv = nhdl.serviceClient<ButtonAction>("button_action");
        pixel23d_srv = nhdl.serviceClient<pixel_2_3d::Pixel23d>("/pixel_2_3d");
        ROS_INFO("[display_manager] DisplayManager loaded.");
    }

    void DisplayManager::doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg) {

        // convert camera image into opencv
        cam_model.fromCameraInfo(info_msg);
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
        if(!buttons_on) {
            // buttons turned off so we don't do anything to the image
            overlay_pub.publish(cv_img->toImageMsg());
            return;
        }

        cv::Size resolution = cam_model.fullResolution();
        cv::Mat button_raster_back = cv::Mat::zeros(resolution.width, resolution.height, CV_8UC1);
        for(uint32_t i=0;i<buttons.size();i++) {
            visualization_msgs::Marker button = buttons[i];
            cv::Scalar color = CV_RGB(button.color.r, button.color.g, button.color.b);

            // project polygon onto image
            cv::Point* cv_poly = new cv::Point[button.points.size()];
            for(uint32_t j=0;j<button.points.size();j++) {
                geometry_msgs::PointStamped cur_pt, proj_pt;
                cur_pt.header = button.header; cur_pt.point = button.points[j];
                cur_pt.header.stamp = ros::Time(0);
                tf_listener.transformPoint(cam_model.tfFrame(), cur_pt, proj_pt);
                cv::Point3d proj_pt_cv(proj_pt.point.x, proj_pt.point.y, proj_pt.point.z);
                cv_poly[j] = cam_model.project3dToPixel(proj_pt_cv);
                if(cv_poly[j].x < 0) cv_poly[j].x = 0;
                if(cv_poly[j].y < 0) cv_poly[j].y = 0;
                if(cv_poly[j].x >= resolution.width-1) cv_poly[j].x = resolution.width-2;
                if(cv_poly[j].y >= resolution.height-1) cv_poly[j].y = resolution.height-2;
            }

            // fill a raster image with this button
            const cv::Point** cv_poly_list = new const cv::Point*[1]; 
            cv_poly_list[0] = cv_poly;
            int* npts = new int[1]; npts[0] = button.points.size()-1;
            //cv::Mat button_raster_cur = cv::Mat::zeros(resolution.width, resolution.height, CV_8UC3);
            cv::fillPoly(button_raster_back, cv_poly_list, npts, 1, cv::Scalar(i+1));
            //cv::max(button_raster_back, button_raster_cur, button_raster_back);
            // etch polygon on image
            cv::polylines(cv_img->image, cv_poly_list, npts, 1, 1, color, 2);
            delete[] npts; delete[] cv_poly; delete[] cv_poly_list;
            button_rasters[i] = button_raster_back;
        }
        overlay_pub.publish(cv_img->toImageMsg());
        //button_pub.publish(cv_img->toImageMsg());
    }

    bool DisplayManager::imageClickSrvCB(ClickImage::Request& req, ClickImage::Response& resp) {
        lImageClickCB(req.image_click);
        return true;
    }
    
    void DisplayManager::lImageClickCB(const geometry_msgs::PointStamped& image_click) {
        if(img_click_lock.try_lock())
            img_click_thread = boost::shared_ptr<boost::thread>(
                                      new boost::thread(
                                          boost::bind(&DisplayManager::doLImageClickCB,
                                                      this, image_click)));
    }

    void DisplayManager::rImageClickCB(const geometry_msgs::PointStamped& msg) {
        if(img_click_lock.try_lock()) {
            std_msgs::Bool bool_msg;
            perceive_pub.publish(bool_msg);
            img_click_lock.unlock();
        }
    }
    
    void DisplayManager::doLImageClickCB(const geometry_msgs::PointStamped& image_click) {
        if(buttons_on) {
            buttons_on = false;
            //figure out which button was clicked on
            uint8_t button_id = 0;
            for(uint32_t i=0;i<button_rasters.size();i++) {
                button_id = button_rasters[i].at<uint8_t>(image_click.point.y, 
                                                          image_click.point.x);
                if(button_id != 0)
                    break;
            }
            ROS_INFO("[display_manager] Image click recieved: (Pix_X: %f, Pix_Y: %f, ID: %d)", 
                                                 image_click.point.x, image_click.point.y, 
                                                 (int) button_id);
            // don't process a click not on a button
            if(button_id != 0) {
                ROS_INFO("Num buttons: %d", (int) buttons.size());
                // Make button action given known information about button press.
                ButtonAction::Request ba_req;
                ba_req.pixel_x = image_click.point.x; ba_req.pixel_y = image_click.point.y;
                ba_req.camera_frame = cam_model.tfFrame();
                ba_req.button_id = buttons[button_id-1].id;
                ba_req.button_type = buttons[button_id-1].id; 
                pixel_2_3d::Pixel23d::Request p3d_req; pixel_2_3d::Pixel23d::Response p3d_resp;
                p3d_req.pixel.point.x = image_click.point.x; p3d_req.pixel.point.y = image_click.point.y; 
                pixel23d_srv.call(p3d_req, p3d_resp);
                ba_req.pixel3d.header.frame_id = p3d_resp.pixel3d.header.frame_id;
                ba_req.pixel3d.header.stamp = p3d_resp.pixel3d.header.stamp;
                ba_req.pixel3d.point.x = p3d_resp.pixel3d.point.x;
                ba_req.pixel3d.point.y = p3d_resp.pixel3d.point.y;
                ba_req.pixel3d.point.z = p3d_resp.pixel3d.point.z;
                ButtonAction::Response ba_resp;
                button_action_srv.call(ba_req, ba_resp);
            }
        }
        img_click_lock.unlock();
    }

    bool DisplayManager::displayButtonsCB(DisplayButtons::Request& req, 
                                          DisplayButtons::Response& resp) {
        displayButtonsCB(req.buttons);
        return true;
    }

    void DisplayManager::displayButtonsCB(std::vector<visualization_msgs::Marker>& buttons_msg) {
        buttons = buttons_msg;
        buttons_on = true;
        button_rasters.resize(buttons.size());
        cv::Size resolution = cam_model.fullResolution();
        for(uint32_t i=0;i<buttons.size();i++)
            button_rasters[i] = cv::Mat::zeros(resolution.width, resolution.height, CV_8UC1);
    }

    bool DisplayManager::clearButtonsCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
        buttons_on = false;
        return true;
    }

};

using namespace hrl_clickable_world;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "display_manager");
    DisplayManager dm;
    ros::spin();
    return 0;
}
