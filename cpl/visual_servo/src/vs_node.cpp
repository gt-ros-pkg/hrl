/*********************************************************************
 *
 *  Copyright (c) 2011, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

// Boost
#include <boost/shared_ptr.hpp>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
#include <vector>
#include <deque>
#include <queue>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <utility>
#include <stdexcept>
#include <float.h>
#include <math.h>
#include <time.h> // for srand(time(NULL))
#include <cstdlib> // for MAX_RAND

// Else
#include <visual_servo/VisualServoTwist.h>
#define DEBUG_MODE 1
#define PRINT_TWISTS 1

#define JACOBIAN_TYPE_INV 1
#define JACOBIAN_TYPE_PSEUDO 2
#define JACOBIAN_TYPE_AVG 3

typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
using boost::shared_ptr;
using geometry_msgs::PoseStamped;
using geometry_msgs::PointStamped;
using visual_servo::VisualServoTwist;


class VisualServoNode
{
  public:
    VisualServoNode(ros::NodeHandle &n) :
      n_(n), n_private_("~"),
      image_sub_(n, "color_image_topic", 1),
      depth_sub_(n, "depth_image_topic", 1),
      cloud_sub_(n, "point_cloud_topic", 1),
      sync_(MySyncPolicy(15), image_sub_, depth_sub_, cloud_sub_),
      it_(n), tf_(), have_depth_data_(false), camera_initialized_(false),
      cur_twist_(cv::Mat::zeros(6,1, CV_32F))
  {
    tf_ = shared_ptr<tf::TransformListener>(new tf::TransformListener());
    n_private_.param("display_wait_ms", display_wait_ms_, 3);
    std::string default_workspace_frame = "/torso_lift_link";
    n_private_.param("workspace_frame", workspace_frame_, default_workspace_frame);
    n_private_.param("num_downsamples", num_downsamples_, 2);
    std::string cam_info_topic_def = "/kinect_head/rgb/camera_info";
    n_private_.param("cam_info_topic", cam_info_topic_, cam_info_topic_def);

    n_private_.param("crop_min_x", crop_min_x_, 0);
    n_private_.param("crop_max_x", crop_max_x_, 640);
    n_private_.param("crop_min_y", crop_min_y_, 0);
    n_private_.param("crop_max_y", crop_max_y_, 480);
    n_private_.param("min_workspace_x", min_workspace_x_, -1.0);
    n_private_.param("min_workspace_y", min_workspace_y_, -1.2);
    n_private_.param("min_workspace_z", min_workspace_z_, -0.8);
    n_private_.param("max_workspace_x", max_workspace_x_, 1.75);
    n_private_.param("max_workspace_y", max_workspace_y_, 1.2);
    n_private_.param("max_workspace_z", max_workspace_z_, 0.6);

    // color segmentation parameters
    n_private_.param("tape_hue_value", tape_hue_value_, 137);
    n_private_.param("tape_hue_threshold", tape_hue_threshold_, 10);
    n_private_.param("default_sat_bot_value", default_sat_bot_value_, 40);
    n_private_.param("default_sat_top_value", default_sat_top_value_, 40);
    n_private_.param("default_val_value", default_val_value_, 200);
    n_private_.param("min_contour_size", min_contour_size_, 10.0);

    // others
    n_private_.param("gain_vel", gain_vel_, 50e-3);
    n_private_.param("gain_rot", gain_rot_, 1e-1);
    n_private_.param("jacobian_type", jacobian_type_, JACOBIAN_TYPE_AVG);
    // Setup ros node connections
    sync_.registerCallback(&VisualServoNode::sensorCallback, this);
  }



    cv::Mat convertImageFrameToCameraFrame(cv::Mat in) 
    {
      cv::Mat out  = cv::Mat::zeros(6,1,CV_32F);
      if (in.rows != 6 || in.cols != 1)
      { 
        return out;
      }
      // XYZ in camera coords and robot control coord are different
      out.at<float>(0,0) =  in.at<float>(2,0);
      out.at<float>(1,0) = -in.at<float>(0,0);
      out.at<float>(2,0) = -in.at<float>(1,0);
      out.at<float>(3,0) =  in.at<float>(5,0);
      out.at<float>(4,0) = -in.at<float>(3,0);
      out.at<float>(5,0) = -in.at<float>(4,0);
      return out;
    }

    bool getTwist(VisualServoTwist::Request &req, VisualServoTwist::Response &res)
    {
      // Do not respond to service call when the node got no sensor callback
      if (!camera_initialized_)
      {
        return false;
      }
      /** Main Logic **/
      // segment color -> find contour -> moments -> 
      // reorder to recognize each point -> find error, interaction matrix, & twist
      cv::Mat tape_mask = colorSegment(cur_color_frame_.clone(), tape_hue_value_, 
          tape_hue_threshold_);
      std::vector<cv::Moments> ms = findMoments(tape_mask, cur_color_frame_); 
      std::vector<cv::Point> pts = getMomentCoordinates(ms);
      cv::Mat twist = computeTwist(desired_locations_, cur_color_frame_, cur_depth_frame_, pts);

#ifdef DEBUG_MODE
      // put dots on the centroids of wrist
      for (unsigned int i = 0; i < pts.size(); i++) 
      {
        cv::circle(cur_orig_color_frame_, pts.at(i), 2, cv::Scalar(100*i, 0, 110*(2-i)), 2);
      }
      // put dots on the desired location
      for (unsigned int i = 0; i < desired_locations_.size(); i++)
      {
        cv::circle(cur_orig_color_frame_, desired_locations_.at(i), 2, cv::Scalar(100*i, 0, 110*(2-i)), 2);
      }

      cv::imshow("Output", cur_orig_color_frame_.clone());
      cv::waitKey(display_wait_ms_);
#endif

      //cv::Mat temp = convertImageFrameToCameraFrame(twist); 
      cv::Mat temp = twist.clone(); 
      
      // have to transform twist in camera frame to torso_lift_link
      tf::Vector3 twist_rot(temp.at<float>(3), temp.at<float>(4), temp.at<float>(5));
      tf::Vector3 twist_vel(temp.at<float>(0), temp.at<float>(1), temp.at<float>(2));
      tf::StampedTransform transform; 
      
      ros::Time now = ros::Time(0);
      try {
        tf::TransformListener listener;
        listener.waitForTransform(workspace_frame_, "/openni_rgb_optical_frame",  now, ros::Duration(1.0));
        listener.lookupTransform(workspace_frame_, "/openni_rgb_optical_frame",  now, transform);
      }
      catch (tf::TransformException e)
      {
        // return 0 value in case of error
        res.vx = 0; res.vy = 0.; res.vz = 0; res.wx= 0; res.wy = 0; res.wz = 0;
        ROS_WARN_STREAM(e.what());
        return true;
      }

      btVector3 out_rot = transform.getBasis() * twist_rot;
      btVector3 out_vel = transform.getBasis() * twist_vel + transform.getOrigin().cross(out_rot);

      // twist cannot be larger than clipping threshold
      // this is a safety measure to prevent robot from breaking
      res.vx =  out_vel.x()*gain_vel_;
      res.vy =  out_vel.y()*gain_vel_;
      res.vz =  out_vel.z()*gain_vel_;
      res.wx =  out_rot.x()*gain_rot_;
      res.wy =  out_rot.y()*gain_rot_;
      res.wz =  out_rot.z()*gain_rot_;

#ifdef PRINT_TWISTS
      printMatrix(temp.t());
      printf("%+.5f\t%+.5f\t%+.5f\t%+.5f\t%+.5f\t%+.5f\n", res.vx, res.vy, res.vz, res.wx, res.wy, res.wz);
#endif
      return true;
    }

    /*
     * Experiment: visual servo
     * Given a fixed engineered setting, we are taping robot hand with
     * three painter's tape and use those three features to do the image-based
     * visual servoing
     **/
    void sensorCallback(const sensor_msgs::ImageConstPtr& img_msg, 
        const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      if (!camera_initialized_)
      {
        cam_info_ = *ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic_, n_, ros::Duration(2.0));
        camera_initialized_ = true;
      }

      /** Preparing the image **/             
      cv::Mat color_frame(bridge_.imgMsgToCv(img_msg));
      cv::Mat depth_frame(bridge_.imgMsgToCv(depth_msg));

      // Swap kinect color channel order
      cv::cvtColor(color_frame, color_frame, CV_RGB2BGR);

      XYZPointCloud cloud; 
      pcl::fromROSMsg(*cloud_msg, cloud);
      tf_->waitForTransform(workspace_frame_, cloud.header.frame_id,
          cloud.header.stamp, ros::Duration(0.5));
      pcl_ros::transformPointCloud(workspace_frame_, cloud, cloud, *tf_);
      prev_camera_header_ = cur_camera_header_;
      cur_camera_header_ = img_msg->header;

      //ROS_INFO("%d", cur_camera_header_.frame_id);

      cv::Mat workspace_mask(color_frame.rows, color_frame.cols, CV_8UC1,
          cv::Scalar(255));
      // Black out pixels in color and depth images outside of workspace
      // As well as outside of the crop window
      for (int r = 0; r < color_frame.rows; ++r)
      {
        uchar* workspace_row = workspace_mask.ptr<uchar>(r);
        for (int c = 0; c < color_frame.cols; ++c)
        {
          // NOTE: Cloud is accessed by at(column, row)
          pcl::PointXYZ cur_pt = cloud.at(c, r);
          if (cur_pt.x < min_workspace_x_ || cur_pt.x > max_workspace_x_ ||
              cur_pt.y < min_workspace_y_ || cur_pt.y > max_workspace_y_ ||
              cur_pt.z < min_workspace_z_ || cur_pt.z > max_workspace_z_ ||
              r < crop_min_y_ || c < crop_min_x_ || r > crop_max_y_ ||
              c > crop_max_x_ )
          {
            workspace_row[c] = 0;
          }
        }
      }
      // focus only on the tabletop setting. do not care about anything far or too close
      color_frame.copyTo(cur_color_frame_, workspace_mask);
      cur_orig_color_frame_ = color_frame.clone();
      depth_frame.copyTo(cur_depth_frame_, workspace_mask);
      cur_point_cloud_ = cloud;

      // if desired point is not initialized
      if (desired_locations_.size() != 3 || countNonZero(desired_jacobian_)==0) {
        desired_locations_ = setDesiredPosition();
        desired_jacobian_ = getInteractionMatrix(cur_depth_frame_, desired_locations_);
        // returned jacobian is null, which means we need to crap out
        if (desired_locations_.size() != 3 || countNonZero(desired_jacobian_) == 0) {
          ROS_WARN("Could not get Image Jacobian for Desired Location. Please re-arrange your setting and retry.");
          ros::Duration(0.5).sleep();
        } 
        else {
          // No error: advertise twist service
          twistServer = n_.advertiseService("visual_servo_twist", &VisualServoNode::getTwist, this);
          ROS_DEBUG("Ready to use the getTwist service");
        }
      }

#ifdef DEBUG_MODE
      // put dots on the desired location
      for (unsigned int i = 0; i < desired_locations_.size(); i++) {
        cv::circle(cur_orig_color_frame_, desired_locations_.at(i), 2, cv::Scalar(100*i, 0, 110*(2-i)), 2);
      }
      cv::imshow("Raw+Goal", cur_orig_color_frame_.clone());
      cv::waitKey(display_wait_ms_);
#endif
    }   

    /**
     * Still in construction:
     * to fix the orientation of the wrist, we now take a look at how the hand is positioned
     * and use it to compute the desired positions.
     **/
    std::vector<cv::Point> setDesiredPosition()
    {
      std::vector<cv::Point> desired; desired.clear();
      // Looking up the hand
      cv::Mat tape_mask = colorSegment(cur_color_frame_.clone(), tape_hue_value_, 
          tape_hue_threshold_);
      std::vector<cv::Moments> ms = findMoments(tape_mask, cur_color_frame_); 
      std::vector<cv::Point> pts = getMomentCoordinates(ms);

      // in case we can't find the hand
      if (pts.size() != 3){
        ROS_WARN("Need to find the hand for reference. Retrying");
        return desired;
      }

      // look up the position of these hands in pc
      pcl::PointXYZ pt0 = cur_point_cloud_.at(pts.at(0).x, pts.at(0).y);
      pcl::PointXYZ pt1 = cur_point_cloud_.at(pts.at(1).x, pts.at(1).y);
      pcl::PointXYZ pt2 = cur_point_cloud_.at(pts.at(2).x, pts.at(2).y);

      // Setting the Desired Location of the wrist
      // Desired location: center of the screen
      pcl::PointXYZ origin = cur_point_cloud_.at(cur_color_frame_.cols/2, cur_color_frame_.rows/2);
      origin.z += 0.1;
      pcl::PointXYZ two = origin;
      pcl::PointXYZ three = origin;

      two.y -= 0.05; //sqrt(pow(pt0.x - pt1.x,2) + pow(pt0.y - pt1.y,2)); 
      three.x -= 0.03;//sqrt(pow(pt0.x - pt2.x, 2) + pow(pt0.y - pt2.y, 2));

      // now we need to convert the position of these desired points that are in pc into the image location
      cv::Point p0 = projectPointIntoImage(origin, cur_point_cloud_.header.frame_id, cur_camera_header_.frame_id);
      cv::Point p1 = projectPointIntoImage(two, cur_point_cloud_.header.frame_id, cur_camera_header_.frame_id);								
      cv::Point p2 = projectPointIntoImage(three, cur_point_cloud_.header.frame_id, cur_camera_header_.frame_id);
      if (p0.x < 0 || p0.y < 0 || 
          p1.x < 0 || p1.y < 0 || 
          p2.x < 0 || p2.y < 0) {
        // impossible to get negative pixel value. exit
        ROS_WARN("Invalid Points Computed. Retrying");
        ROS_WARN("Points at [%3d,%3d][%3d,%3d][%3d,%3d]", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y); 
        return desired;
      }
      desired.push_back(p0); desired.push_back(p1); desired.push_back(p2);
      return desired;
      ROS_INFO("Setting the Desired Point at [%3d,%3d][%3d,%3d][%3d,%3d]", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y); 
    }

    cv::Mat computeTwist(std::vector<cv::Point> desired, cv::Mat color_frame, 
        cv::Mat depth_frame, std::vector<cv::Point> pts) 
    {
      cv::Mat ret = cv::Mat::zeros(6,1, CV_32F);
      if (pts.size() == 3){
        cv::Mat error_mat = cv::Mat::zeros(6,1, CV_32F);

        for (int i = 0; i < 3; i++) {
          // Error = Desired location in image - Current position in image
          error_mat.at<float>(i*2,0) 	= pts.at(i).x - desired.at(i).x ; 
          error_mat.at<float>(i*2+1,0)  = pts.at(i).y - desired.at(i).y ;
        }

        // testing: buffing error seem to make velocity stable near the match 
        // error_mat *= 10;

        cv::Mat im = getInteractionMatrix(depth_frame, pts);

        // if we can't compute interaction matrix, just make all twists 0
        if (countNonZero(im) == 0) {
          return ret;
        }

        // inverting the matrix (3 approaches)
        cv::Mat iim;
        switch (jacobian_type_) {
          case JACOBIAN_TYPE_INV:
            iim = (im).inv();
            break;
          case JACOBIAN_TYPE_PSEUDO:
            iim = (im.t() * im).inv()*im.t();
            break;
          default: // JACOBIAN_TYPE_AVG
            // We use specific way shown on visual servo by Chaumette 2006
            cv::Mat temp = desired_jacobian_ + im;
            iim = 0.5*(temp.t() * temp).inv() * temp.t();
        }
        // Gain Matrix K
        cv::Mat gain = cv::Mat::eye(6,6, CV_32F);
        gain.at<float>(0,0) = 5e-2;
        gain.at<float>(1,1) = 5e-2;
        /*
        gain.at<float>(2,2) = gain_vel_;
        gain.at<float>(3,3) = gain_rot_;
        gain.at<float>(4,4) = gain_rot_;
        gain.at<float>(5,5) = gain_rot_;
         */
        // K x IIM x ERROR = TWIST
        ret = gain*(iim*error_mat);


#ifdef DEBUG_MODE
#ifdef PRINT_TWISTS
        ROS_INFO("1. error matrix, 2. im frame twist, 3. torso frame twist");
        printMatrix((error_mat).t());
#endif
#endif

      }
      return ret;
    }

    void printMatrix(cv::Mat_<double> in)
    {
      for (int i = 0; i < in.rows; i++) {
        for (int j = 0; j < in.cols; j++) {
          printf("%+.5f\t", in(i,j)); 
        }
        printf("\n");
      }
    }

    cv::Mat getInteractionMatrix(cv::Mat depth_frame, std::vector<cv::Point> &pts) 
    {
      // interaction matrix, image jacobian
      cv::Mat L = cv::Mat::zeros(6,6,CV_32F);
      if (pts.size() == 3) {
        for (int i = 0; i < 3; i++) {
          cv::Point m = pts.at(i);
          int x = m.x;
          int y = m.y;
          float z = depth_frame.at<float>(y, x);
          // float z = cur_point_cloud_.at(y,x).z;
          int l = i * 2;
          if (z <= 0 || isnan(z)) return cv::Mat::zeros(6,6, CV_32F);
          L.at<float>(l,0) = 1/z;   L.at<float>(l+1,0) = 0;
          L.at<float>(l,1) = 0;      L.at<float>(l+1,1) = 1/z;
          L.at<float>(l,2) = -x/z;    L.at<float>(l+1,2) = -y/z;
          L.at<float>(l,3) = x*y;    L.at<float>(l+1,3) = -(1 + pow(y,2));
          L.at<float>(l,4) = (1+pow(x,2));  L.at<float>(l+1,4) = x*y;
          L.at<float>(l,5) = -y;      L.at<float>(l+1,5) = x;
        }
      }
#ifdef DEBUG_MODE
      //      ROS_DEBUG("Interaction");
      //      printMatrix(L);
      //      ROS_DEBUG("Inverse");
      //      printMatrix(L.inv());
      //      ROS_DEBUG("Pseudo");
      //      printMatrix(pseudo);
#endif
      return L;
    }

    std::vector<cv::Point> getMomentCoordinates(std::vector<cv::Moments> ms)
    {
      std::vector<cv::Point> ret;
      ret.clear();
      if (ms.size() == 3) { 
        double centroids[3][2];
        for (int i = 0; i < 3; i++) {
          cv::Moments m0 = ms.at(i);
          double x0, y0;
          x0 = m0.m10/m0.m00;
          y0 = m0.m01/m0.m00;
          centroids[i][0] = x0; 
          centroids[i][1] = y0; 
        }

        // find the top left corner using distance scheme
        double dist[3][3], lowest(1e6);
        int one(-1);
        for (int i = 0; i < 3; i++) {
          for (int j = i; j < 3; j++) {
            double temp = pow(centroids[j][0]- centroids[i][0],2) 
              + pow(centroids[j][1]-centroids[i][1],2);
            dist[i][j] = temp;
            dist[j][i] = temp;
          }
          double score = dist[i][0] + dist[i][1] + dist[i][2];
          if (score < lowest) {
            lowest = score;
            one = i;
          }
        }

        // index of others depending on the index of the origin
        int a = one == 0 ? 1 : 0;
        int b = one == 2 ? 1 : 2; 
        // vectors of origin to a point
        double vX0, vY0, vX1, vY1, result;
        vX0 = centroids[a][0] - centroids[one][0];
        vY0 = centroids[a][1] - centroids[one][1];
        vX1 = centroids[b][0] - centroids[one][0];
        vY1 = centroids[b][1] - centroids[one][1];
        cv::Point pto(centroids[one][0], centroids[one][1]);
        cv::Point pta(centroids[a][0], centroids[a][1]);
        cv::Point ptb(centroids[b][0], centroids[b][1]);

        // cross-product: simplified assuming that z = 0 for both
        result = vX1*vY0 - vX0*vY1;
        ret.push_back(pto);
        if (result >= 0) {
          ret.push_back(ptb);
          ret.push_back(pta);
        }
        else {
          ret.push_back(pta);
          ret.push_back(ptb);
        }

      }
      return ret;
    } 

    /**
     * findMoments: first, apply morphology to filter out noises and find contours around
     * possible features. Then, it returns the three largest moments
     * Arg
     * in: single channel image input
     * Return
     * three largest moment in the image
     */
    std::vector<cv::Moments> findMoments(cv::Mat in, cv::Mat &color_frame) 
    {
      cv::Mat open, temp;
      cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
      cv::morphologyEx(in.clone(), open, cv::MORPH_OPEN, element);
      std::vector<std::vector<cv::Point> > contours; contours.clear();
      temp = open.clone();
      cv::findContours(temp, contours, cv::RETR_CCOMP,CV_CHAIN_APPROX_NONE);
      std::vector<cv::Moments> moments; moments.clear();

      for (unsigned int i = 0; i < contours.size(); i++) {
        cv::Moments m = cv::moments(contours[i]);
        if (m.m00 > min_contour_size_) {
          // first add the forth element
          moments.push_back(m);
          // find the smallest element of 4 and remove that
          if (moments.size() > 3) {
            double small(moments.at(0).m00);
            unsigned int smallInd(0);
            for (unsigned int j = 1; j < moments.size(); j++){
              if (moments.at(j).m00 < small) {
                small = moments.at(j).m00;
                smallInd = j;
              }
            }
            moments.erase(moments.begin() + smallInd);
          }
        }
      }
#ifdef DEBUG_MODE
      cv::drawContours(cur_orig_color_frame_, contours, -1,  cv::Scalar(50,225,255), 2);
      //cv::imshow("in", in); 
      //cv::imshow("open", open.clone());   
#endif
      return moments;
    }

    cv::Mat colorSegment(cv::Mat color_frame, int hue, int threshold)
    {
      /*
       * Often value = 0 or 255 are very useless. 
       * The distance at those end points get very close and it is not useful
       * Same with saturation 0. Low saturation makes everything more gray scaled
       * So the default setting are below 
       */
      return colorSegment(color_frame, hue - threshold, hue + threshold,  default_sat_bot_value_, default_sat_top_value_, 50, default_val_value_);
    }

    /** 
     * colorSegment
     * Very Basic Color Segmentation done in HSV space
     * Takes in Hue value and threshold as input to compute the distance in color space
     * cv::Mat color_frame: color input from image
     * Return: mask from the color segmentation 
     */
    cv::Mat colorSegment(cv::Mat color_frame, int _hue_n, int _hue_p, int _sat_n, int _sat_p, int _value_n,  int _value_p)
    {
      cv::Mat temp (color_frame.clone());
      cv::cvtColor(temp, temp, CV_RGB2HSV);
      std::vector<cv::Mat> hsv;
      cv::split(temp, hsv);

      // masking out values that do not fall between the condition 
      cv::Mat wm(color_frame.rows, color_frame.cols, CV_8UC1, cv::Scalar(0));
      for (int r = 0; r < temp.rows; r++)
      {
        uchar* workspace_row = wm.ptr<uchar>(r);
        for (int c = 0; c < temp.cols; c++)
        {
          int hue = (int)hsv[0].at<uchar>(r, c), sat = (int)hsv[1].at<uchar>(r, c), value = (int)hsv[2].at<uchar>(r, c);
          if (_hue_n < hue && hue < _hue_p)
            if (_sat_n < sat && sat < _sat_p)
              if (_value_n < value && value < _value_p)
                workspace_row[c] = 255;

        } 
      }

      // removing unwanted parts by applying mask to the original image
      return wm;
    }

    /** 
     * Helper Method from Object_Singulation
     */ 
    cv::Point projectPointIntoImage(pcl::PointXYZ cur_point_pcl,
        std::string point_frame, std::string target_frame)
    {
      PointStamped cur_point;
      cur_point.header.frame_id = point_frame;
      cur_point.point.x = cur_point_pcl.x;
      cur_point.point.y = cur_point_pcl.y;
      cur_point.point.z = cur_point_pcl.z;
      return projectPointIntoImage(cur_point, target_frame);
    }

    cv::Point projectPointIntoImage(PointStamped cur_point,
        std::string target_frame)
    {
      cv::Point img_loc;
      try
      {
        // Transform point into the camera frame
        PointStamped image_frame_loc_m;
        tf_->transformPoint(target_frame, cur_point, image_frame_loc_m);
        // Project point onto the image
        img_loc.x = static_cast<int>((cam_info_.K[0]*image_frame_loc_m.point.x +
              cam_info_.K[2]*image_frame_loc_m.point.z) /
            image_frame_loc_m.point.z);
        img_loc.y = static_cast<int>((cam_info_.K[4]*image_frame_loc_m.point.y +
              cam_info_.K[5]*image_frame_loc_m.point.z) /
            image_frame_loc_m.point.z);

      }
      catch (tf::TransformException e)
      {
        ROS_ERROR_STREAM(e.what());
      }
      return img_loc;
    }

    /**
     * Executive control function for launching the node.
     */
    void spin()
    {
      while(n_.ok())
      {
        ros::spinOnce();
      }
    }

  protected:
    ros::NodeHandle n_;
    ros::NodeHandle n_private_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    image_transport::ImageTransport it_;
    sensor_msgs::CameraInfo cam_info_;
    sensor_msgs::CvBridge bridge_;
    shared_ptr<tf::TransformListener> tf_;
    cv::Mat cur_color_frame_;
    cv::Mat cur_orig_color_frame_;
    cv::Mat cur_depth_frame_;
    cv::Mat cur_workspace_mask_;
    std_msgs::Header cur_camera_header_;
    std_msgs::Header prev_camera_header_;
    XYZPointCloud cur_point_cloud_;

    bool have_depth_data_;
    int display_wait_ms_;
    int num_downsamples_;
    std::string workspace_frame_;
    bool camera_initialized_;
    std::string cam_info_topic_;
    int tracker_count_;
    // filtering 
    double min_workspace_x_;
    double max_workspace_x_;
    double min_workspace_y_;
    double max_workspace_y_;
    double min_workspace_z_;
    double max_workspace_z_;
    int crop_min_x_;
    int crop_max_x_;
    int crop_min_y_;
    int crop_max_y_;
    // segmenting
    int tape_hue_value_;
    int tape_hue_threshold_;
    int default_sat_bot_value_;
    int default_sat_top_value_;
    int default_val_value_;
    double min_contour_size_;

    int jacobian_type_;
    double gain_vel_;
    double gain_rot_;
    ros::ServiceServer twistServer;
    std::vector<cv::Point> desired_locations_;
    cv::Mat cur_twist_; 
    cv::Mat desired_jacobian_;
};

int main(int argc, char ** argv)
{
  srand(time(NULL));
  ros::init(argc, argv, "visual_servo_node");

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  ros::NodeHandle n;
  VisualServoNode vs_node(n);
  vs_node.spin();
  return 0;
}
