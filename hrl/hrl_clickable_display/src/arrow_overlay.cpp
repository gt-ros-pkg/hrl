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

#include <hrl_clickable_display/ArrowOverlayCmd.h>

using namespace std;

namespace hrl_clickable_display {

    class ArrowOverlay {
        public:
            ros::NodeHandle nh;
            ros::Subscriber command_sub;
            ros::Subscriber pixel3d_sub;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_transport::Publisher overlay_pub;
            image_geometry::PinholeCameraModel cam_model;
            tf::TransformListener tf_listener;
            double ARROW_POINTS[7][3];
            bool auto_clear;

            std::vector<geometry_msgs::PoseStamped> arrows;

            ArrowOverlay();
            ~ArrowOverlay();
            void onInit();
            void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg);
            void processCommand(const ArrowOverlayCmd& msg);
            void pixelCallback(const geometry_msgs::PoseStamped& msg);
    };

    ArrowOverlay::ArrowOverlay() : nh("~"), img_trans(nh) {
    }

    ArrowOverlay::~ArrowOverlay() {
    }

    void ArrowOverlay::onInit() {
        double A, B, C;
        bool debug;
        nh.param<double>("shaft_size", A, 0.1);
        nh.param<double>("head_width", B, 0.03);
        nh.param<double>("head_length", C, 0.03);
        nh.param<bool>("debug_mode", debug, false);
        nh.param<bool>("auto_clear", auto_clear, false);
        ARROW_POINTS[0][0] = 0;    ARROW_POINTS[0][1] = 0;    ARROW_POINTS[0][2] = 0;
        ARROW_POINTS[1][0] = 0;    ARROW_POINTS[1][1] = 0;    ARROW_POINTS[1][2] = A;
        ARROW_POINTS[2][0] = -B/2; ARROW_POINTS[2][1] = -B/2; ARROW_POINTS[2][2] = A;
        ARROW_POINTS[3][0] =  B/2; ARROW_POINTS[3][1] = -B/2; ARROW_POINTS[3][2] = A;
        ARROW_POINTS[4][0] =  B/2; ARROW_POINTS[4][1] =  B/2; ARROW_POINTS[4][2] = A;
        ARROW_POINTS[5][0] = -B/2; ARROW_POINTS[5][1] =  B/2; ARROW_POINTS[5][2] = A;
        ARROW_POINTS[6][0] = 0;    ARROW_POINTS[6][1] = 0;    ARROW_POINTS[6][2] = A+C;

        camera_sub = img_trans.subscribeCamera<ArrowOverlay>
                                              ("/image_in", 1, 
                                               &ArrowOverlay::doOverlay, this);
        overlay_pub = img_trans.advertise("/image_out", 1);

        command_sub = nh.subscribe("arrow_command", 1, 
                                           &ArrowOverlay::processCommand, this);

        if(debug) {
            pixel3d_sub = nh.subscribe("/pixel3d", 1, 
                                               &ArrowOverlay::pixelCallback, this);
        }
        ROS_INFO("[display_manager] ArrowOverlay loaded.");
    }

    void ArrowOverlay::doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg) {

        // convert camera image into opencv
        cam_model.fromCameraInfo(info_msg);
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

        for(uint32_t i=0;i<arrows.size();i++) {
            // get arrow's rotation matrix
            tf::Pose arrow_pose;
            tf::poseMsgToTF(arrows[i].pose, arrow_pose);
            std::vector<cv::Point> cv_pts;
            for(int j=0;j<7;j++) {
                //geometry_msgs::PointStamped arr_pt;
                btVector4 arr_pt;
                arr_pt.setX(ARROW_POINTS[j][0]);
                arr_pt.setY(ARROW_POINTS[j][1]);
                arr_pt.setZ(ARROW_POINTS[j][2]);
                arr_pt.setW(1);
                btVector3 arr_pt_rot = arrow_pose * arr_pt;
                cv::Point3d proj_pt_cv(arr_pt_rot.getX(), arr_pt_rot.getY(), arr_pt_rot.getZ());
                cv::Point pt2d = cam_model.project3dToPixel(proj_pt_cv);
                cv_pts.push_back(pt2d);
            }
            const cv::Point** cv_poly_list = new const cv::Point*[6];
            cv::Point* shaft = new cv::Point[2];
            shaft[0] = cv_pts[0]; shaft[1] = cv_pts[1]; 
            cv::Point* base = new cv::Point[5];
            base[0] = cv_pts[2]; base[1] = cv_pts[3]; base[2] = cv_pts[4]; 
            base[3] = cv_pts[5];  base[4] = cv_pts[2]; 
            cv::Point* pyrm1 = new cv::Point[2];
            pyrm1[0] = cv_pts[2]; pyrm1[1] = cv_pts[6]; 
            cv::Point* pyrm2 = new cv::Point[2];
            pyrm2[0] = cv_pts[3]; pyrm2[1] = cv_pts[6]; 
            cv::Point* pyrm3 = new cv::Point[2];
            pyrm3[0] = cv_pts[4]; pyrm3[1] = cv_pts[6]; 
            cv::Point* pyrm4 = new cv::Point[2];
            pyrm4[0] = cv_pts[5]; pyrm4[1] = cv_pts[6]; 
            cv_poly_list[0] = shaft; cv_poly_list[1] = base; cv_poly_list[2] = pyrm1;
            cv_poly_list[3] = pyrm2; cv_poly_list[4] = pyrm3; cv_poly_list[5] = pyrm4;
            cv::Scalar color = CV_RGB(255, 0, 0);
            int npts[6] = {2, 5, 2, 2, 2, 2};
            cv::polylines(cv_img->image, cv_poly_list, npts, 6, 0, color, 1);
        }

        overlay_pub.publish(cv_img->toImageMsg());
    }

    void ArrowOverlay::processCommand(const ArrowOverlayCmd& msg) {
        if(msg.clear_arrows)
            arrows.clear();
        else {
            for(uint32_t i=0;i<msg.arrows.size();i++) {
                geometry_msgs::PoseStamped new_arrow;
                tf_listener.transformPose(cam_model.tfFrame(), ros::Time(0), msg.arrows[i], msg.arrows[i].header.frame_id, new_arrow);
                arrows.push_back(new_arrow);
            }
        }
    }

    void ArrowOverlay::pixelCallback(const geometry_msgs::PoseStamped& msg) {
        if(auto_clear)
            arrows.clear();
        geometry_msgs::PoseStamped new_arrow;
        tf_listener.transformPose(cam_model.tfFrame(), ros::Time(0), msg, msg.header.frame_id, new_arrow);
        arrows.push_back(new_arrow);
    }

};

using namespace hrl_clickable_display;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arrow_overlay");
    ArrowOverlay ao;
    ao.onInit();
    ros::spin();
    return 0;
}
