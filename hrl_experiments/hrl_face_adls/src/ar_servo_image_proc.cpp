#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

image_transport::CameraSubscriber camera_sub;
image_transport::Publisher output_pub;
boost::shared_ptr<tf::TransformListener> tf_list;
cv_bridge::CvImagePtr cv_img, new_cv_img;

void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {

    // convert camera image into opencv
    cam_model.fromCameraInfo(info_msg);
    cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    new_cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

    cv::equalizeHist(cv_img->image, new_cv_img->image);
    output_pub.publish(new_cv_img->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_servo_image_proc");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);
    tf_list = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    camera_sub = img_trans.subscribeCamera("/camera", 1, 
                                           &doOverlay);
    output_pub = img_trans.advertise("/output", 1);

    return 0;
}
