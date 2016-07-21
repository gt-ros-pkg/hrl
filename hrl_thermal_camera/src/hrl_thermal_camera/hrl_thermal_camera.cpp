#include "hrl_thermal_camera_node.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

int main(int argc,char **argv)
{
  ros::init(argc,argv,"ros_pleora");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  sensor_msgs::Image msg;
  sensor_msgs::CameraInfo cam_info;
  HRL_Thermal_Camera camera;
  camera.Connect();
  camera.StartAcquisition();

  ros::Rate loop_rate(100);

  while(nh.ok())
  {
    const ros::Time time = ros::Time::now();
    try
    {
      msg.header.stamp = time;
      if(camera.GrabImage(msg,cam_info))
      {
	cam_info.header = msg.header;
	pub.publish(msg);
      }
    }
    catch(const std::exception &e)
    {
	ROS_ERROR("%s: %s",nh.getNamespace().c_str(),e.what());
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  camera.StopAcquisition();
  camera.Disconnect();
  return 0;
}
