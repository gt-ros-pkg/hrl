#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

std::string win_name = "El-e Carrriage";
//std::string win_name = "El-e in-hand Camera";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  sensor_msgs::CvBridge bridge;
  try
  {
    cvShowImage(win_name.c_str(), bridge.imgMsgToCv(msg, "rgb8"));
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "theora_listener");
  ros::NodeHandle nh;
  cvNamedWindow(win_name.c_str());
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("hrl_compressed_img/image", 1, imageCallback);
  ros::spin();
  cvDestroyWindow(win_name.c_str());
}

