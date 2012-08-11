#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;

image_transport::CameraSubscriber camera_sub;
image_transport::Publisher output_pub;
cv_bridge::CvImagePtr cv_img;

void doOverlay(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg) {

    // convert camera image into opencv
    cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat input(cv_img->image.rows, cv_img->image.cols, CV_8UC1);
    cv_bridge::CvImage new_cv_img;
    new_cv_img.image = cv::Mat(cv_img->image.rows, cv_img->image.cols, CV_8UC1);
    double black_cap, white_cap;
    ros::param::param<double>("~black_cap", black_cap, 50);
    ros::param::param<double>("~white_cap", white_cap, 150);
    for(int j=0;j<cv_img->image.cols;j++)
        for(int i=0;i<cv_img->image.rows;i++) {
            double inten = (cv_img->image.at<cv::Vec3b>(i, j)[0] +
                            cv_img->image.at<cv::Vec3b>(i, j)[1] +
                            cv_img->image.at<cv::Vec3b>(i, j)[2])/3.;
            inten = 255.0 / (white_cap - black_cap) * (inten - black_cap);
            inten = max(0.0, min(inten, 255.0));
            input.at<uint8_t>(i, j) = (uint8_t) inten;
        }

    new_cv_img.image = input;
    //cv::equalizeHist(input, new_cv_img.image);
    new_cv_img.header = cv_img->header;
    new_cv_img.encoding = sensor_msgs::image_encodings::MONO8;
    output_pub.publish(new_cv_img.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_servo_image_proc");
    ros::NodeHandle nh;
    image_transport::ImageTransport img_trans(nh);

    camera_sub = img_trans.subscribeCamera("/camera", 1, 
                                           &doOverlay);
    output_pub = img_trans.advertise("/output", 1);

    ros::spin();

    return 0;
}
