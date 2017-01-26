#ifndef PCLOUD_PAINTER_H_
#define PCLOUD_PAINTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

// message filters
#include <message_filters/subscriber.h>
#include <message_filters/connection.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class PCloud_Painter
{
private:
  ros::NodeHandle                                                   nh;
  tf::TransformListener 											tf_listener;
  image_geometry::PinholeCameraModel 								cam_model_;
  ros::Subscriber  													thermal_camera_info_sub;
  cv::Point2d 														uv;
  ros::Publisher                                                    painted_pcloud_pub,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	image_pub;
  sensor_msgs::Image 												thermal_image,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	rgb_image;
  int 																ind;
  uint16_t this_temp;
  float																thermal_val;

  tf::StampedTransform 												rgb_to_thermal_tf;
  std_msgs::Header 												    pointcloud_msg_header;
  bool																paint_cloud_active,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	    currently_painting;
  cv_bridge::CvImagePtr cv_ptr;
  bool has_camera;
  pcl::PointCloud<pcl::PointXYZI>                                   tcloud;

  sensor_msgs::PointCloud2                                          converted_cloud;
  pcl::PointXYZI													conversion_point;
  sensor_msgs::PointCloud2									        pcloud_rgb_in,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	pcloud_thermal_in,
																	pcloud_out;

  pcl::PointCloud<pcl::PointXYZ>									rgbcloud,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	rgb_transformed_cloud;
  sensor_msgs::ImageConstPtr 										tcam;
  sensor_msgs::CameraInfoConstPtr	  	  	  	  	  	  	  	  	rgbcam_info,
  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	    tcam_info;
  message_filters::Connection										callback_control;
  message_filters::Subscriber<sensor_msgs::Image>                   thermal_image_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>             rgb_pcloud_sub;

  typedef message_filters::sync_policies::ApproximateTime
          <sensor_msgs::Image,
          sensor_msgs::PointCloud2>                                 MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy>                       sync;

public:

  PCloud_Painter(    const ros::NodeHandle                   & nh, const tf::TransformListener & tf_listener);

  ~PCloud_Painter()
  {}

  void
  thermal_camera_info_cb (const sensor_msgs::CameraInfoConstPtr& info_msg);

  void
  painting_callback (	const sensor_msgs::ImageConstPtr        & thermal_image_msg,
		  	  	  	  	const sensor_msgs::PointCloud2ConstPtr  & rgb_pcloud_msg);

  /* void */
  /* publish_cloud(const pcl::PointCloud<pcl::PointXYZI>& pcloud, */
  /*                        	   	const std_msgs::Header& pcloud_msg_header, */
  /*   							ros::Publisher& pub); */

  bool
  spin();

};

#endif /* PCLOUD_PAINTER_H_ */





