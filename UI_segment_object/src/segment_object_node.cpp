// #
// # Copyright (c) 2009, Georgia Tech Research Corporation
// # All rights reserved.
// #
// # Redistribution and use in source and binary forms, with or without
// # modification, are permitted provided that the following conditions are met:
// #     * Redistributions of source code must retain the above copyright
// #       notice, this list of conditions and the following disclaimer.
// #     * Redistributions in binary form must reproduce the above copyright
// #       notice, this list of conditions and the following disclaimer in the
// #       documentation and/or other materials provided with the distribution.
// #     * Neither the name of the Georgia Tech Research Corporation nor the
// #       names of its contributors may be used to endorse or promote products
// #       derived from this software without specific prior written permission.
// #
// # THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
// # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// # DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
// # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// # OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// # OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// # ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// #

// #  \author Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)


#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <pcl_ros/point_cloud.h>
#include "pcl/ModelCoefficients.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include "geometry_msgs/Point.h"
#include "UI_segment_object/GetObject.h"
#include "UI_segment_object/GetPt.h"
#include "UI_segment_object/None_Bool.h"
#include "UI_segment_object/Save.h"
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>

boost::mutex m;
/*****************************************************************************/
/*  Global variables and function declaration necessary for using mouse call back  */

std::vector<std::vector<int> > poly_vec;
std::vector<int> in_indices;
int counter(0);
int counter_ind(0);
//bool mouse_active(false);
CvPoint pts[1000];
int done (0);

/***********************************************************************************/


class UI {

public:
  //  bool got_image;
  UI(ros::NodeHandle &nh);
  ~UI();
  bool reset_cb(UI_segment_object::None_Bool::Request &reg, UI_segment_object::None_Bool::Response &res);
  static void mouse_callback(int event, int x, int y, int flags, void* param);
  void color_segment_callback();
  void pt_callback();
  void callback(const sensor_msgs::ImageConstPtr& msg_ptr);
  void cloud_callback();
  int pnpoly(int npol, std::vector<std::vector<int > > poly_vec, int x, int y);
  bool save_image(UI_segment_object::Save::Request &reg, UI_segment_object::Save::Response &res);
  ros::ServiceServer reset_service;
  ros::ServiceServer save_img;
  
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Subscriber image_sub_;
  IplImage* cv_image;
  int height;
  int width;
};


  
UI::UI(ros::NodeHandle &nh) :
  nh_(nh), it_(nh_)
{
  //    got_image = false;
  //    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &UI::callback, this);
  cv_image = NULL;
  counter = 0;
  counter_ind = 0;
  //    cvNamedWindow("Select");                                                               
  //    cvSetMouseCallback("Select", mouse_callback);
  reset_service = nh_.advertiseService("UI_reset", &UI::reset_cb, this);
  save_img = nh_.advertiseService("save_image", &UI::save_image, this);
}

UI::~UI()
{
  cvDestroyWindow("Select");                                                             
}

bool UI::reset_cb(UI_segment_object::None_Bool::Request &reg, UI_segment_object::None_Bool::Response &res)
{
  printf("resetting vectors..");
  poly_vec.clear();
  in_indices.clear();
  counter = 0;
  counter_ind = 0;
  done = 0;
  res.state = true;
  return (true);
}


void UI::mouse_callback(int event, int x, int y, int flags, void* param)
{
  // if (event == CV_EVENT_LBUTTONDOWN)
  //   {
  //     mouse_active = true;
  //   }
  // if (event == CV_EVENT_LBUTTONUP)
  //   {
  //     mouse_active = false;
  //   }
  // if (event == CV_EVENT_MOUSEMOVE && mouse_active == true)
  //   {
  //     poly_vec.push_back(std::vector<int>());
  //     poly_vec[counter].push_back(x);
  //     poly_vec[counter].push_back(y);
  //     counter++;
  //   }

  if (event == CV_EVENT_LBUTTONDOWN)
    {
      poly_vec.push_back(std::vector<int>());
      poly_vec[counter].push_back(x);
      poly_vec[counter].push_back(y);
      counter++;
    }

  if (event == CV_EVENT_RBUTTONDOWN)
    {
      std::cerr << "you did click the right mouse button" << std::endl;
      done = 1;
    }
}

void UI::color_segment_callback()
{
  cvNamedWindow("Select");
  cvSetMouseCallback("Select", mouse_callback);
  sensor_msgs::ImageConstPtr image_in = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_color", nh_);
  cv_image = bridge_.imgMsgToCv(image_in, "bgr8"); 
  while (done == 0)
    {
      for (int i = 0; i < counter; i++)
	{
	  cvCircle(cv_image, cvPoint(poly_vec[i][0], poly_vec[i][1]), 2, CV_RGB(255, 0, 0), 2, 8);
	}
      cvShowImage("Select", cv_image);
      cvWaitKey(33);
    }


  //decrease the search in the image by finding the bounding box
  height = cv_image->height;
  width = cv_image->width;
  
  for (int j = 0; j < poly_vec.size(); j++)
    {
      pts[j] = cvPoint(poly_vec[j][0], poly_vec[j][1]);
    }
  IplImage* mask = cvCreateImage(cvSize(width, height), 8, 1);
  cvFillConvexPoly(mask, pts, poly_vec.size(), cvScalar(255));
  cvSaveImage("./mask.png", mask);

  //do some color segmentation selection for indices after selecting a point of certain color or maybe multiple, would
  //allow auto segmentation of a certain region automatically

}

void UI::pt_callback()
{
  cvNamedWindow("Select");                                                               
  cvSetMouseCallback("Select", mouse_callback);

  //    sensor_msgs::ImageConstPtr image_in;
  sensor_msgs::ImageConstPtr image_in = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_color", nh_);
  cv_image = bridge_.imgMsgToCv(image_in, "bgr8");
  //    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
  while (done == 0)
    {
      for (int i = 0; i < counter; i++)
	{
	  cvCircle(cv_image, cvPoint(poly_vec[i][0], poly_vec[i][1]), 2, CV_RGB(255, 0, 0), 2, 8);
	}
      cvShowImage("Select", cv_image);
      cvWaitKey(33);
    }

  //decrease the search in the image by finding the bounding box
  height = cv_image->height;
  width = cv_image->width;
  int index;
  index = poly_vec[0][1]*width+poly_vec[0][0];
  in_indices.push_back(index);
  cvDestroyWindow("Select");
}

void UI::callback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
}

void UI::cloud_callback(){
  cvNamedWindow("Select");                                                               
  cvSetMouseCallback("Select", mouse_callback);

  //    sensor_msgs::ImageConstPtr image_in;
  sensor_msgs::ImageConstPtr image_in = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_color", nh_);
  cv_image = bridge_.imgMsgToCv(image_in, "bgr8");
  //    cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
  while (done == 0)
    {
      for (int i = 0; i < counter; i++)
	{
	  cvCircle(cv_image, cvPoint(poly_vec[i][0], poly_vec[i][1]), 2, CV_RGB(255, 0, 0), 2, 8);
	}
      cvShowImage("Select", cv_image);
      cvWaitKey(33);
    }

  //decrease the search in the image by finding the bounding box
  height = cv_image->height;
  width = cv_image->width;
  int max_x(0);
  int min_x(width);
  int max_y(0);
  int min_y(height);
  for (int i=0; i<counter; i++)
    {
      if (poly_vec[i][0]>max_x)
        max_x = poly_vec[i][0];
      if (poly_vec[i][0]<min_x)
    	min_x = poly_vec[i][0];
      if (poly_vec[i][1]>max_y)
    	max_y = poly_vec[i][1];
      if (poly_vec[i][1]<min_y)
    	min_y = poly_vec[i][1];
    }

  //check for image position of 3D points within
  //user defined polygon
  for (int i=min_y; i<max_y; i++)
    {
      for (int j=min_x; j<max_x; j++)
	{
	  int in;
	  in = pnpoly(counter, poly_vec, j, i);
	  if (in == 1)
	    {
	      in_indices.push_back((i)*width+j);
	      counter_ind ++;
	    }
	}
    }
  //    got_image = true;
  cvDestroyWindow("Select");
}


int UI::pnpoly(int npol, std::vector<std::vector<int > > poly_vec, int x, int y)
{
  int i, j, c = 0;
  for (i = 0, j = npol-1; i < npol; j = i++) 
    {
      if ((((poly_vec[i][1] <= y) && (y < poly_vec[j][1])) ||
	   ((poly_vec[j][1] <= y) && (y < poly_vec[i][1]))) &&
	  (x < (poly_vec[j][0] - poly_vec[i][0]) * (y - poly_vec[i][1]) / (poly_vec[j][1] - poly_vec[i][1]) + poly_vec[i][0]))
	c = !c;
    }
  return c;   //returns 1 for interior and 0 for exterior points
}

bool UI::save_image(UI_segment_object::Save::Request &reg, UI_segment_object::Save::Response &res)
{
  //    sensor_msgs::ImageConstPtr image_in = 
  m.lock();
  cv_image = bridge_.imgMsgToCv(ros::topic::waitForMessage<sensor_msgs::Image>("/camera/rgb/image_color", nh_), "bgr8");
  cvSaveImage(reg.file.c_str(), cv_image);
  m.unlock();
  res.state = true;
  return(true);
}




class PointCloudPub {

public:
  PointCloudPub(ros::NodeHandle &nh);
  ~PointCloudPub();
  //  void publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);
  void publish_region(const sensor_msgs::PointCloud2 &msg);
  void publish_object(const sensor_msgs::PointCloud2 &msg);
  sensor_msgs::PointCloud2 cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool get_cloud(UI_segment_object::GetObject::Request &reg, UI_segment_object::GetObject::Response &res);
  bool get_pt(UI_segment_object::GetPt::Request &reg, UI_segment_object::GetPt::Response &res);
  bool reset_plane_coeff(UI_segment_object::None_Bool::Request &reg, UI_segment_object::None_Bool::Response &res);
  bool save_cloud(UI_segment_object::Save::Request &reg, UI_segment_object::Save::Response &res);
  geometry_msgs::Point pt_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  UI::UI ui;


protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_region_;
  ros::Publisher pub_object_;
  pcl::PointCloud<pcl::PointXYZ> cloud_plane_; //(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz_rgb;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz_rgb2;
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;
  sensor_msgs::PointCloud2::ConstPtr ptcloud2_xyz_rgb;
  pcl::ModelCoefficients coefficients;
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  
  pcl::PointIndices inliers;
  bool new_plane_coeff;
  bool transform_pts;
  std::string transform_pts_frame_id;
};

PointCloudPub::PointCloudPub(ros::NodeHandle &nh):
    ui(nh)
{
  nh_ = nh;
  pub_region_ = nh_.advertise<sensor_msgs::PointCloud2>("segment_plane", 1);
  pub_object_ = nh_.advertise<sensor_msgs::PointCloud2>("segment_object", 1);
  // Create the segmentation object
  new_plane_coeff = true;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setProbability(0.98);

  while (nh_.getParam("/transform_cloud/use", transform_pts) == false)
    {
      std::cerr << "waiting for boolean param to transform pts or not ... \n";
      sleep(0.05);
    }
  if (transform_pts == true)
    {
      while (nh_.getParam("/transform_cloud/frame_id", transform_pts_frame_id) == false)
	{
	  std::cerr << "waiting for frame_id to transform pts ... \n";
	  sleep(0.05);
	}
    }
}

PointCloudPub::~PointCloudPub()
{
}


void PointCloudPub::publish_region(const sensor_msgs::PointCloud2 &msg)
{
  pub_region_.publish(msg);
}

void PointCloudPub::publish_object(const sensor_msgs::PointCloud2 &msg)
{
  pub_object_.publish(msg);
}


bool PointCloudPub::get_cloud(UI_segment_object::GetObject::Request &reg, UI_segment_object::GetObject::Response &res)
{
  m.lock();
  //  sensor_msgs::PointCloud2::ConstPtr msg;
  cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points", nh_, ros::Duration(30.0));

  if (done == 0)
    {
      ui.cloud_callback();
    }
  //process the point cloud for object and return object
  //  sensor_msgs::PointCloud2 msg2 = cloud_callback(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points2", nh_, ros::Duration(6.0)));

  sensor_msgs::PointCloud2 msg2 = cloud_callback(cloud_msg);
  //  res.object = msg2;
  m.unlock();
  return (true);
}


bool PointCloudPub::save_cloud(UI_segment_object::Save::Request &reg, UI_segment_object::Save::Response &res)
{
  m.lock();
  pcl::io::savePCDFileASCII (reg.file, cloud_xyz_rgb);
  res.state = true;
  m.unlock();
  return(true);
}


bool PointCloudPub::reset_plane_coeff(UI_segment_object::None_Bool::Request &reg, UI_segment_object::None_Bool::Response &res)
{
  new_plane_coeff = true;
  res.state = true;
  return (true);
}


//////////////////////////////////////////////finish here for 3d  point returned
bool PointCloudPub::get_pt(UI_segment_object::GetPt::Request &reg, UI_segment_object::GetPt::Response &res)
{
  //  sensor_msgs::PointCloud2::Ptr msg (new sensor_msgs::PointCloud2());

  cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points", nh_, ros::Duration(30.0));

  if (done == 0)
    {
      ui.pt_callback();
    }
  geometry_msgs::Point pt = pt_callback(cloud_msg);
  res.pt = pt;

  return (true);
}

geometry_msgs::Point PointCloudPub::pt_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  m.lock();
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::fromROSMsg(*msg, cloud_xyz);

  geometry_msgs::Point pt;

  pt.x = cloud_xyz.points[in_indices[0]].x;
  pt.y = cloud_xyz.points[in_indices[0]].y;
  pt.z = cloud_xyz.points[in_indices[0]].z;
  std::cerr << "here is ind :" << in_indices[0] << std::endl;
  std::cerr << "here is pt :" << pt.x << "\t" << pt.y << "\t" << pt.z << std::endl;
  m.unlock();
  return pt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

sensor_msgs::PointCloud2 PointCloudPub::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  tf::TransformListener tf_listener;

  pcl::PointCloud<pcl::PointXYZRGB> pt_cloud_before_trans;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz2;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_plane;  //(new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2 object;
  sensor_msgs::PointCloud2 region;
  std::cerr << "Point cloud data: " << msg->height*msg->width << " points" << std::endl;
  
  //     pcl::fromROSMsg(msg, cloud_xyz);pp
  //     pcl::getFields (cloud_xyz, fields);
  //   }
  
  pcl::fromROSMsg(*msg, cloud_xyz);

  if (transform_pts == true)
    {
      pcl::fromROSMsg(*msg, pt_cloud_before_trans);
      tf::StampedTransform transform;
      tf_listener.waitForTransform(transform_pts_frame_id, "/openni_rgb_optical_frame2", ros::Time(0), ros::Duration(5.0));
      tf_listener.lookupTransform(transform_pts_frame_id, "/openni_rgb_optical_frame2", ros::Time(0), transform);
      pcl_ros::transformPointCloud(pt_cloud_before_trans, cloud_xyz_rgb, transform);
    }
  else
    {
      pcl::fromROSMsg(*msg, cloud_xyz_rgb);
    }

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //  pcl::PointIndices::Ptr ui_ind (new pcl::PointIndices());
  pcl::PointIndices ui_ind;
  //////////////setup ui_ind here...../////////////
  for (int i = 0; i < counter_ind ; i++)
    {
      ui_ind.indices.push_back(in_indices[i]);
    }
  
  extract.setInputCloud(cloud_xyz_rgb.makeShared());

  extract.setIndices(boost::make_shared<pcl::PointIndices>(ui_ind));
  extract.setNegative(false);
  extract.filter(cloud_xyz_rgb2);
  pcl::toROSMsg(cloud_xyz_rgb2, region);
  publish_region(region);

  if (new_plane_coeff==true)
    {
      new_plane_coeff = false;
      //      seg.setInputCloud (cloud_xyz2.makeShared());
      //      seg.segment (inliers, coefficients);
    }

  seg.setInputCloud (cloud_xyz_rgb2.makeShared());
  seg.segment (inliers, coefficients);

  extract.setInputCloud(cloud_xyz_rgb2.makeShared());
  extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
  extract.setNegative(true);
  extract.filter(cloud_plane);

  pcl::toROSMsg(cloud_plane, object);
  publish_object(object);

  if (inliers.indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    }

  std::cerr << "Model coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " "
	    << coefficients.values[2] << " " << coefficients.values[3] << std::endl;

  return object;
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "segment_object");
  ros::NodeHandle nh;
  PointCloudPub pcb(nh);
  ros::ServiceServer service = nh.advertiseService("get_object_on_plane", &PointCloudPub::get_cloud, &pcb);
  ros::ServiceServer service2 = nh.advertiseService("get_3D_pt", &PointCloudPub::get_pt, &pcb);
  ros::ServiceServer service3 = nh.advertiseService("save_pt_cloud", &PointCloudPub::save_cloud, &pcb);
  //  ros::ServiceServer service4 = nh.advertiseService("reset_plane_coeff", &PointCloudPub::reset_plane_coeff, &pcb);
  ros::spin();

  return (0);
}
