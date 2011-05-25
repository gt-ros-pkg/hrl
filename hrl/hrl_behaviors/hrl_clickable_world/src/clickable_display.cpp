#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include <geometry_msgs/PointStamped.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifdef HAVE_GTK
#include <gtk/gtk.h>
static void destroyNode(GtkWidget *widget, gpointer data) { exit(0); }
#endif

namespace hrl_clickable_world {

    class ClickableDisplay {
        public:
            ros::NodeHandle nh;
            image_transport::ImageTransport img_trans;
            image_transport::Subscriber camera_sub;
            ros::Publisher pt_pub;
            std::string img_frame;

            ClickableDisplay();
            ~ClickableDisplay();
            void onInit();
            void imgCallback(const sensor_msgs::ImageConstPtr& img_msg);
            static void mouseClickCallback(int event, int x, int y, int flags, void* t);

    };

    ClickableDisplay::ClickableDisplay() : img_trans(nh) {
    }

    typedef void (*ptr_to_func)(int,int,int,int,void*);
    void callbackWrapper(ClickableDisplay* cd, int a, int b, int c, int d, void* e) { 
        cd->mouseClickCallback(a,b,c,d,e);
    }

    void ClickableDisplay::onInit() {
        cv::namedWindow("Clickable World", 1);
        cv::setMouseCallback("Clickable World", &ClickableDisplay::mouseClickCallback, this);

#ifdef HAVE_GTK
        GtkWidge *widget = GTK_WIDGET(cvGetWindowHandle("Clickable World"));
        g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
#endif

        camera_sub = img_trans.subscribe<ClickableDisplay>
                                              ("image", 1, 
                                               &ClickableDisplay::imgCallback, this);
        pt_pub = nh.advertise<geometry_msgs::PointStamped>("mouse_click", 1);
        ROS_INFO("[clickable_display] ClickableDisplay loaded.");
        ros::Duration(1).sleep();
    }

    void ClickableDisplay::imgCallback(const sensor_msgs::ImageConstPtr& img_msg) {
        img_frame = img_msg->header.frame_id;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Clickable World", cv_ptr->image);
            cv::waitKey(3);
        }
        catch(cv_bridge::Exception& e) {
            ROS_ERROR("[clickable_display] cv_bridge exception: %s", e.what());
            return;
        }
    }

    void ClickableDisplay::mouseClickCallback(int event, int x, int y, int flags, void* param) {
        ClickableDisplay* this_ = reinterpret_cast<ClickableDisplay*>(param);
        if(event == CV_EVENT_LBUTTONDOWN) {
            geometry_msgs::PointStamped click_pt;
            click_pt.header.frame_id = this_->img_frame;
            click_pt.header.stamp = ros::Time::now();
            click_pt.point.x = x; click_pt.point.y = y;
            this_->pt_pub.publish(click_pt);
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
    ros::spin();
    return 0;
}
