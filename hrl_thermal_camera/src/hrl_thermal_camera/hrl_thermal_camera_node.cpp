
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//#include "hrl_thermal_camera_node.h"
#include <hrl_thermal_camera/hrl_thermal_camera.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>
#include <sstream>
#include <std_srvs/Empty.h>
//#include "opencv2/imgproc.hpp"
//#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace std;

using namespace cv;

namespace hrl_thermal_cam {

class HRLThermalCameraNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  sensor_msgs::CameraInfo cam_info_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, camera_name_, distortion_model_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  std::vector<double> distortion_coefficients_;
  std::vector<double> camera_matrix_;
  std::vector<double> rectification_matrix_;
  std::vector<double> projection_matrix_;
//  float distortion_coefficients_ [];
//  float camera_matrix_ [9];
//  float rectification_matrix_ [9];
//  float projection_matrix_ [12];


  HRLThermalCamera cam_;

  ros::ServiceServer service_start_, service_stop_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }

  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  HRLThermalCameraNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 15);
    node_.param("distortion_model", distortion_model_, std::string("plumb_bob"));
    node_.getParam("distortion_coefficients/data", distortion_coefficients_);
    node_.getParam("camera_matrix/data", camera_matrix_);
    node_.getParam("rectification_matrix/data", rectification_matrix_);
    node_.getParam("projection_matrix/data", projection_matrix_);
    cam_info_.width = image_width_;
    cam_info_.height = image_height_;

    for (int idx = 0; idx < distortion_coefficients_.size(); idx++)
    {
      cam_info_.D.push_back(distortion_coefficients_[idx]);
    }
    for (int idx = 0; idx < camera_matrix_.size(); idx++)
    {
      cam_info_.K[idx] = camera_matrix_[idx];
    }
    for (int idx = 0; idx < rectification_matrix_.size(); idx++)
    {
      cam_info_.R[idx] = rectification_matrix_[idx];
    }
    for (int idx = 0; idx < projection_matrix_.size(); idx++)
    {
      cam_info_.P[idx] = projection_matrix_[idx];
    }

    cam_info_.distortion_model = distortion_model_;
//    cam_info_.D = distortion_coefficients_;
//    cam_info_.K = camera_matrix_;
//    cam_info_.R = rectification_matrix_;
//    cam_info_.P = projection_matrix_;
//    cam_info_.framerate = framerate_

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("thermal_camera_frame"));
    node_.param("camera_name", camera_name_, std::string("thermal_camera"));


    // create Services
    service_start_ = node_.advertiseService("start_capture", &HRLThermalCameraNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &HRLThermalCameraNode::service_stop_cap, this);

    // start the camera
    cam_.start(image_width_, image_height_, framerate_);
  }

  virtual ~HRLThermalCameraNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(img_, cam_info_);

    // grab the camera info
    //  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    //  ci->header.frame_id = img_.header.frame_id;
    //  ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, cam_info_);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }

};
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hrl_thermal_camera");
  hrl_thermal_cam::HRLThermalCameraNode a;
  a.spin();
  return EXIT_SUCCESS;
}


