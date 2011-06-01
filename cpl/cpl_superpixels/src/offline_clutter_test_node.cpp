#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "opencv2/highgui/highgui.hpp"
#include <string>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <cpl_superpixels/SmoothClutter.h>
using cpl_superpixels::SmoothClutter;

struct PointXYZIL;

class OfflineClutterTestNode
{
 public:
  OfflineClutterTestNode(ros::NodeHandle n): n_(n)
  {
    ros::NodeHandle n_private("~");
    std::string def_image_name = "fake.png";
    std::string def_mask_name = "fake_mask.png";
    std::string def_pcd_name = "fake.pcd";
    std::string def_smooth_name = "fake-smooth.png";
    std::string def_clutter_name = "fake-clutter.png";

    n_private.param("input_image_name", input_image_name_, def_image_name);
    n_private.param("input_pcd_name", input_pcd_name_, def_pcd_name);
    n_private.param("input_mask_name", input_mask_name_, def_mask_name);
    n_private.param("smooth_output_name", smooth_output_name_, def_smooth_name);
    n_private.param("clutter_output_name", clutter_output_name_, def_clutter_name);

    superpixel_client_ = n_.serviceClient<SmoothClutter>("smooth_clutter");

  }

  void spin()
  {
    // Read in image
    cv::Mat input_img = cv::imread(input_image_name_);
    cv::Mat mask_img = cv::imread(input_mask_name_, 0);
    // Read in PCD
    sensor_msgs::PointCloud2 input_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile(input_pcd_name_, input_cloud) == -1)
    {
      ROS_ERROR_STREAM("Failed to read in pcd file: " << input_pcd_name_);
      return;
    }

    // TODO: Figure out how to convert to Point<x y z i L>
    pcl::fromROSMsg(input_cloud, cloud);
    ROS_INFO_STREAM("Fields: " << pcl::getFieldsList(input_cloud));
    // TODO: Perform clutter classification
    cv::Mat label_img = mask_img;

    // Smooth with superpixels
    SmoothClutter sc;
    IplImage raw_ipl = input_img;
    IplImage label_ipl = mask_img;

    ROS_INFO("Converting input images.");
    sensor_msgs::Image::ConstPtr msg_in = bridge_.cvToImgMsg(&raw_ipl);
    sensor_msgs::Image::ConstPtr labels_in = bridge_.cvToImgMsg(&label_ipl);
    sc.request.raw_img = *msg_in;
    sc.request.clutter_labels = *labels_in;

    ROS_INFO("Calling smoother client.");
    if( !superpixel_client_.call(sc) )
    {
      ROS_ERROR_STREAM("Failure of call to superpixel smoother service!");
    }

    sensor_msgs::ImageConstPtr msg_out =
        boost::shared_ptr<sensor_msgs::Image const>(
        new sensor_msgs::Image(sc.response.smooth_clutter_labels));
    cv::Mat smooth_labels = bridge_.imgMsgToCv(msg_out);
    ROS_INFO("Converted result.");

    // Do something with the output
    // Save data
    cv::imwrite(smooth_output_name_, smooth_labels);
    cv::imwrite(clutter_output_name_, label_img);

    // Display results with high_gui
    ROS_INFO("Displaying results.");
    cv::imshow("GT Labels", mask_img);
    cv::imshow("Smooth Labels", smooth_labels);
    cv::imshow("Raw image", input_img);
    cv::waitKey();
  };

 protected:
  ros::NodeHandle n_;
  ros::ServiceClient superpixel_client_;
  std::string input_pcd_name_;
  std::string input_image_name_;
  std::string input_mask_name_;
  std::string smooth_output_name_;
  std::string clutter_output_name_;
  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "");
  ros::NodeHandle n;
  OfflineClutterTestNode clutter_test(n);
  clutter_test.spin();
}
