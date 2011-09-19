#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <sstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>

class BagExtractor
{
 public:
  BagExtractor(ros::NodeHandle &n) :
      n_(n), it_(n), img_count_(0),
      base_path_("/home/thermans/sandbox/bag_imgs/")
  {
    image_sub_ = it_.subscribe("image_topic", 1000,
                               &BagExtractor::imageCallback, this);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    cv::Mat input_image;
    input_image = bridge_.imgMsgToCv(msg_ptr);
    cv::Mat output_image;
    cv::cvtColor(input_image, output_image, CV_RGB2BGR);

    // cv::imshow("input_image", input_image);
    // cv::imshow("output_image", output_image);
    // cv::waitKey();

    std::stringstream out_name;
    out_name << base_path_;
    out_name << img_count_++;
    out_name << ".tiff";
    cv::imwrite(out_name.str(), output_image);
  }

  void spin()
  {
    while(n_.ok())
    {
      ros::spinOnce();
    }
  }

 protected:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  int img_count_;
  std::string base_path_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "overhead_tracking");
  ros::NodeHandle n;
  BagExtractor extractor(n);
  extractor.spin();
}

