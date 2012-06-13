#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <hrl_head_tracking/hsl_rgb_conversions.h>

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

PCRGB::Ptr aligned_pc;
ros::Subscriber align_sub;

image_transport::CameraSubscriber camera_sub;
image_transport::Publisher overlay_pub;
image_geometry::PinholeCameraModel cam_model;

void subAlignCallback(const PCRGB::Ptr& aligned_pc_)
{
    aligned_pc = aligned_pc_;
}

void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {

    // convert camera image into opencv
    cam_model.fromCameraInfo(info_msg);
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

    uint8_t r, g, b;
    if(!aligned_pc)
        return;
    for(uint32_t i=0;i<aligned_pc->size();i++) {
        cv::Point3d proj_pt_cv(aligned_pc->points[i].x, aligned_pc->points[i].y, 
                               aligned_pc->points[i].z);
        cv::Point pt2d = cam_model.project3dToPixel(proj_pt_cv);
        extractRGB(aligned_pc->points[i].rgb, r, g, b);
        cv_img->image.at<cv::Vec3b>(pt2d.y, pt2d.x)[0] = r;
        cv_img->image.at<cv::Vec3b>(pt2d.y, pt2d.x)[1] = g;
        cv_img->image.at<cv::Vec3b>(pt2d.y, pt2d.x)[2] = b;
    }
    
    overlay_pub.publish(cv_img->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);

    align_sub = nh.subscribe("/head_registration/aligned_pc", 1, &subAlignCallback);
    camera_sub = img_trans.subscribeCamera("/kinect_head/rgb/image_color", 1, 
                                           &doOverlay);
    overlay_pub = img_trans.advertise("/head_registration/confirmation", 1);

    ros::Rate r(5);
    while(ros::ok()) {
        cv::Mat image(480, 640, CV_8UC3);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
