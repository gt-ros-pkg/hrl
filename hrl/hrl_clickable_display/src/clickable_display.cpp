#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace hrl_clickable_world {

    class ClickableDisplay {
        public:
            ros::NodeHandle nh, nh_priv;
            image_transport::ImageTransport img_trans;
            image_transport::Subscriber camera_sub;
            ros::Publisher l_click_pub, r_click_pub;
            std::string img_frame;
            cv_bridge::CvImagePtr img_ptr;
            bool have_img;

            ClickableDisplay();
            ~ClickableDisplay();
            void onInit();
            void imgCallback(const sensor_msgs::ImageConstPtr& img_msg);
            void showImg();
            static void mouseClickCallback(int event, int x, int y, int flags, void* data);

    };

    ClickableDisplay::ClickableDisplay() : nh_priv("~"), 
                                           img_trans(nh),
                                           have_img(false) {
    }

    void ClickableDisplay::onInit() {
        cv::namedWindow("Clickable World", 1);
        cv::setMouseCallback("Clickable World", &ClickableDisplay::mouseClickCallback, this);

        camera_sub = img_trans.subscribe<ClickableDisplay>
                                              ("image", 1, 
                                               &ClickableDisplay::imgCallback, this);
        l_click_pub = nh_priv.advertise<geometry_msgs::PointStamped>("l_mouse_click", 1);
        r_click_pub = nh_priv.advertise<geometry_msgs::PointStamped>("r_mouse_click", 1);
        
        ROS_INFO("[clickable_display] ClickableDisplay loaded.");
        ros::Duration(1).sleep();
    }

    void ClickableDisplay::imgCallback(const sensor_msgs::ImageConstPtr& img_msg) {
        img_frame = img_msg->header.frame_id;
        try {
            img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            have_img = true;
        }
        catch(cv_bridge::Exception& e) {
            ROS_ERROR("[clickable_display] cv_bridge exception: %s", e.what());
            return;
        }
    }

    void ClickableDisplay::showImg() {
        ros::Rate r(30);
        while(ros::ok()) {
            ros::spinOnce();
            if(have_img) {
                cv::imshow("Clickable World", img_ptr->image);
                cv::waitKey(3);
            }
            r.sleep();
        }
    }

    void ClickableDisplay::mouseClickCallback(int event, int x, int y, int flags, void* param) {
        ClickableDisplay* this_ = reinterpret_cast<ClickableDisplay*>(param);
        if(event == CV_EVENT_LBUTTONDOWN) {
            geometry_msgs::PointStamped click_pt;
            click_pt.header.frame_id = this_->img_frame;
            click_pt.header.stamp = ros::Time::now();
            click_pt.point.x = x; click_pt.point.y = y;
            this_->l_click_pub.publish(click_pt);
        }
        if(event == CV_EVENT_RBUTTONDOWN) {
            geometry_msgs::PointStamped click_pt;
            click_pt.header.frame_id = this_->img_frame;
            click_pt.header.stamp = ros::Time::now();
            click_pt.point.x = x; click_pt.point.y = y;
            this_->r_click_pub.publish(click_pt);
        }
    }

    ClickableDisplay::~ClickableDisplay() {
        cv::destroyWindow("Clickable World");
    }

};


using namespace hrl_clickable_world;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clickable_display");
    ClickableDisplay cd;
    cd.onInit();
    cd.showImg();
    ros::spin();
    return 0;
}
