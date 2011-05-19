
#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <std_srvs/Empty.h>

#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <hrl_clickable_world/ClickImage.h>
#include <hrl_clickable_world/DisplayButtons.h>
#include <hrl_clickable_world/ButtonAction.h>


using namespace std;

namespace hrl_clickable_world {

    class DisplayManager {
        public:
            ros::Publisher button_pushed_pub;
            ros::ServiceServer display_buttons_srv, clear_buttons_srv, image_click_srv;
            ros::ServiceClient button_action_srv;
            ros::NodeHandle nhdl;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_transport::Publisher overlay_pub;
            image_geometry::PinholeCameraModel cam_model;
            tf::TransformListener tf_listener;

            bool buttons_on;
            vector<visualization_msgs::Marker> buttons;
            vector<cv::Point*> button_polys;
            vector<uint32_t> button_vert_counts;
            vector<uint32_t> button_inds;
            cv::Mat button_rastor;

            DisplayManager();
            ~DisplayManager();
            void onInit();
            void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg);
            bool imageClickCB(ClickImage::Request& req, ClickImage::Response& resp);
            bool displayButtonsCB(DisplayButtons::Request& req, DisplayButtons::Response& resp);
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
                                              ("/kinect_head/rgb/image_color", 1, 
                                               &DisplayManager::doOverlay, this);
        overlay_pub = img_trans.advertise("image_buttons", 1);
        button_pushed_pub = nhdl.advertise<std_msgs::Int32>("button_pushed", 1);
        image_click_srv = nhdl.advertiseService("click_image", 
                                                &DisplayManager::imageClickCB, this);
        display_buttons_srv = nhdl.advertiseService("display_buttons", 
                                                &DisplayManager::displayButtonsCB, this);
        button_action_srv = nhdl.serviceClient<ButtonAction>("button_action");
    }

    void DisplayManager::doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg) {
        std::vector<cv::Scalar> button_colors;
        button_colors.push_back(CV_RGB(0, 255, 255)); 
        button_colors.push_back(CV_RGB(255, 0, 0)); 
        button_colors.push_back(CV_RGB(0, 255, 0)); 
        button_colors.push_back(CV_RGB(0, 0, 255)); 

        cam_model.fromCameraInfo(info_msg);
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
        if(buttons_on) {
            for(uint32_t i=0;i<button_polys.size();i++) {
                cv::Point** pts_list = new cv::Point*[1];
                pts_list[0] = button_polys[i];
                int* npts = new int[1]; npts[0] = button_vert_counts[i];
                cv::polylines(cv_img->image, (const cv::Point**) pts_list, npts, 
                              1, 1, button_colors[button_inds[i]], 2);
                delete[] pts_list;
            }
        }
        overlay_pub.publish(cv_img->toImageMsg());
    }
    
    bool DisplayManager::imageClickCB(ClickImage::Request& req, ClickImage::Response& resp) {
        if(buttons_on) {
            //figure out which button was clicked on
            float button_id = button_rastor.at<float>(req.image_click.point.x, 
                                                      req.image_click.point.y);
            ButtonAction::Request ba_req;
            ba_req.click_loc = req.image_click;
            ba_req.button_id = button_id;
            ba_req.button_type = ""; // TODO fill this in
            ButtonAction::Response ba_resp;
            button_action_srv.call(ba_req, ba_resp);
            ROS_INFO("ID: %f", button_id);
            buttons_on = false;
        }
        return true;
    }

    bool DisplayManager::displayButtonsCB(DisplayButtons::Request& req, 
                                          DisplayButtons::Response& resp) {
        /*
        std::vector<std::string> button_types;
        button_types.push_back("face"); button_types.push_back("object"); 
        button_types.push_back("table"); button_types.push_back("floor"); 
        */

        cv::Size resolution = cam_model.fullResolution();
        button_rastor = cv::Mat::zeros(resolution.width, resolution.height, CV_32F);
        for(uint32_t i=0;i<button_polys.size();i++) 
            delete button_polys[i];
        button_polys.clear();
        button_vert_counts.clear();
        for(uint32_t i=0;i<req.buttons.size();i++) {
            visualization_msgs::Marker button = req.buttons[i];
            /*
            uint32_t type_ind = std::find(button_types.begin(), button_types.end(), 
                                          button.ns) - button_types.begin();
            if(type_ind == button_types.size()) {
                ROS_ERROR("Button type %s unknown", button.ns);
                return false;
            }
            */
            uint32_t type_ind = 0;

            cv::Point* cv_poly = new cv::Point[button.points.size()];
            for(uint32_t j=0;j<button.points.size();j++) {
                geometry_msgs::PointStamped cur_pt, proj_pt;
                cur_pt.header = button.header; cur_pt.point = button.points[j];
                cur_pt.header.stamp = ros::Time(0);
                tf_listener.transformPoint(cam_model.tfFrame(), cur_pt, proj_pt);
                cv::Point3d proj_pt_cv(proj_pt.point.x, proj_pt.point.y, proj_pt.point.z);
                cv_poly[j] = cam_model.project3dToPixel(proj_pt_cv);
            }
            button_polys.push_back(cv_poly);
            button_vert_counts.push_back(button.points.size());
            button_inds.push_back(type_ind);
            cv::fillConvexPoly(button_rastor, cv_poly, button.points.size(), 
                               cv::Scalar(button.id));
        }
        buttons_on = true;
        return true;
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
