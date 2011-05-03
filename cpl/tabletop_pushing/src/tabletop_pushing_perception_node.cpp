/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Visual features
#include <cpl_visual_features/sliding_window.h>
#include <cpl_visual_features/features/hsv_color_histogram.h>
#include <cpl_visual_features/features/attribute_learning_base_feature.h>

// tabletop_pushing
#include <tabletop_pushing/PushPose.h>

// STL
#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>

//#define DEBUG_CANNY_OUTPUT 1
//#define DEBUG_BOUNDS 1
//#define DEBUG_PUSH_POINT 1
//#define DEBUG_PUSH_ORIENTATION 1

using tabletop_pushing::PushPose;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;

class PushPoint
{
 public:
  PushPoint(cv::Point p2, pcl::PointXYZ p3) : point2d(p2), point3d(p3) {}
  cv::Point point2d;
  pcl::PointXYZ point3d;

  bool operator<(const PushPoint& other) const
  {
    return ((other.point3d.x*other.point3d.x +
             other.point3d.y*other.point3d.y) <
            (point3d.x*point3d.x + point3d.y*point3d.y));
  }
};

typedef std::priority_queue<PushPoint> PushPointList;

class TabletopPushingPerceptionNode
{
 public:
  TabletopPushingPerceptionNode(ros::NodeHandle &n) :
      n_(n),
      image_sub_(n,"table_image_in", 1),
      pc_sub_(n,"table_point_cloud_in", 1),
      sync_(image_sub_, pc_sub_, 10),
      tf_(),
      int_canny_thresh_1_(2100.0),
      int_canny_thresh_2_(2300.0),
      depth_canny_thresh_1_(2300.0),
      depth_canny_thresh_2_(2500.0),
      patch_diameter_(5),
      have_depth_data_(false),
      num_calls_(0)
  {
    // Setup parameters
    const std::string default_height_map_frame = "/base_link";
    ros::NodeHandle n_private("~");
    n_private.param("height_map_frame", height_map_frame_,
                    default_height_map_frame);
    n_private.param("intensity_canny_thresh_1", int_canny_thresh_1_, 2100.0);
    n_private.param("intensity_canny_thresh_2", int_canny_thresh_2_, 2300.0);
    n_private.param("depth_canny_thresh_1", depth_canny_thresh_1_, 2300.0);
    n_private.param("depth_canny_thresh_2", depth_canny_thresh_2_, 2500.0);
    n_private.param("height_map_patch_diameter", patch_diameter_, 5);
    n_private.param("x_min_bound",  x_min_bound_, 0.1);
    n_private.param("x_max_bound",  x_max_bound_, 1.0);
    n_private.param("y_min_bound",  y_min_bound_, -0.3);
    n_private.param("y_max_bound",  y_max_bound_, 0.3);
    n_private.param("z_min_bound",  z_min_bound_, 0.6);
    n_private.param("z_max_bound",  z_max_bound_, 2.0);
    n_private.param("sobel_kernel_size", sobel_size_, 3);
    // Filter size much be smaller than the patch_diameter
    sobel_size_ = std::min(sobel_size_, patch_diameter_);

    // Combine the subscribed topics with a message filter
    sync_.registerCallback(&TabletopPushingPerceptionNode::callback, this);
    push_point_server_ = n_.advertiseService(
        "get_push_pose", &TabletopPushingPerceptionNode::getPushPose, this);
    converted_pcl_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>(
        "transformed_pcl_cloud", 1000);

  }

  void callback(const sensor_msgs::ImageConstPtr& img_msg,
                const sensor_msgs::PointCloud2ConstPtr& pc_msg)
  {
    // Convert images to OpenCV format
    cv::Mat input_frame(bridge_.imgMsgToCv(img_msg));

    // Convert Point Cloud to PCL structure
    XYZPointCloud input_cloud;
    pcl::fromROSMsg (*pc_msg, input_cloud);

    // ROS_DEBUG_STREAM("Converting point cloud to depth_frame.");
    // Convert
    cv::Mat depth_frame(input_frame.size(), CV_64FC1);

    for (unsigned int x = 0; x < input_cloud.width; ++x)
    {
      for (unsigned int y = 0; y < input_cloud.height; ++y)
      {
        depth_frame.at<float>(y,x) = input_cloud.points[y*input_cloud.width +
                                                        x].z;
      }
    }

    // Convert cloud to be height_map
    XYZPointCloud height_map;
    try
    {
      tf::StampedTransform transform;
      tf_.waitForTransform (input_cloud.header.frame_id, height_map_frame_,
                            input_cloud.header.stamp, ros::Duration(3.0));
      tf_.lookupTransform (input_cloud.header.frame_id, height_map_frame_,
                           input_cloud.header.stamp, transform);
      pcl_ros::transformPointCloud(input_cloud, height_map, transform);
    }
    catch (tf::TransformException tf_ex)
    {
      ROS_ERROR_STREAM("Exception transforming point cloud to height map: "
                       << tf_ex.what());
      return;
    }

    // Save to the class members
    cur_frame_ = input_frame;
    cur_depth_frame_ = depth_frame;
    cur_height_map_ = height_map;
    have_depth_data_ = true;
  }

  bool getPushPose(PushPose::Request& req, PushPose::Response& res)
  {
    num_calls_++;
    ROS_INFO_STREAM("Calling findPushCandidate.");
    if ( have_depth_data_ )
    {
      res = findPushPose(cur_frame_, cur_depth_frame_, cur_height_map_);
    }
    else
    {
      ROS_ERROR_STREAM("Calling getPushPose prior to receiving sensor data.");
      return false;
    }
    return true;
  }

  PushPose::Response findPushPose(cv::Mat& input_frame,
                                  cv::Mat& depth_frame,
                                  XYZPointCloud& height_map)
  {
    // TODO: Change this to use a multi-demnsional gaussian filter across
    // all channels including depth to find the edges instead of Canny
    cv::Mat frame_bw(input_frame.size(), CV_8UC1);
    cv::Mat int_edges(input_frame.size(), CV_8UC1);
    cv::Mat depth_8u(depth_frame.size(), CV_8UC1);
    cv::Mat depth_edges(depth_frame.size(), CV_8UC1);

    // Find edges for candidate pushing locations
    if (input_frame.channels() == 3)
    {
      ROS_INFO_STREAM("Converting to BW.");
      cv::cvtColor(input_frame, frame_bw, CV_BGR2GRAY);
    }
    else
    {
      frame_bw = input_frame;
    }

    // Convert depth image from CV_32FC1 to CV_8UC1
    depth_frame.convertTo(depth_8u, depth_8u.type());

    cv::Canny(frame_bw, int_edges, int_canny_thresh_1_,
              int_canny_thresh_2_, 5);
    cv::Canny(depth_8u, depth_edges, depth_canny_thresh_1_,
              depth_canny_thresh_2_, 5);

#ifdef DEBUG_CANNY_OUTPUT
    cv::imshow("Intensity Edges", int_edges);
    cv::imshow("Intensity img", input_frame);
    cv::imshow("Depth Edges", depth_edges);
    cv::imshow("Depth img", depth_8u);
    cv::waitKey();
#endif // DEBUG_CANNY_OUTPUT

    // Find candidate poses from the edge images
    PushPointList strong_edges;
    int patch_radius = patch_diameter_/2;
    int count510 = 0;

#ifdef DEBUG_BOUNDS
    float z_min = 100;
    float z_max = 0;
    float x_min = 100;
    float x_max = 0;
    float y_min = 100;
    float y_max = 0;
#endif // DEBUG_BOUNDS

    for(int r = patch_radius; r < int_edges.rows - patch_radius; ++r)
    {
      for(int c = patch_radius; c < int_edges.cols - patch_radius; ++c)
      {
        int edge_score = int_edges.at<uchar>(r,c) + int_edges.at<uchar>(r,c);

        // TODO: Filter more appropriately based on table location
        PushPoint edge_point(cv::Point(c,r),
                             height_map.points[r * height_map.width + c]);

        // Only use the heighest ranked scores
        if (edge_score != 510 ||  isnan(edge_point.point3d.x) ) continue;
        count510++;

#ifdef DEBUG_BOUNDS
        if (edge_point.point3d.x < x_min &&
            edge_point.point3d.x > 0 ) x_min = edge_point.point3d.x;
        if (edge_point.point3d.x > x_max) x_max = edge_point.point3d.x;
        if (edge_point.point3d.y < y_min) y_min = edge_point.point3d.y;
        if (edge_point.point3d.y > y_max) y_max = edge_point.point3d.y;
        if (edge_point.point3d.z < z_min) z_min = edge_point.point3d.z;
        if (edge_point.point3d.z > z_max) z_max = edge_point.point3d.z;
#endif // DEBUG_BOUNDS
        if (edge_point.point3d.x > x_min_bound_ &&
            edge_point.point3d.x < x_max_bound_ &&
            edge_point.point3d.y > y_min_bound_ &&
            edge_point.point3d.y < y_max_bound_ &&
            edge_point.point3d.z > z_min_bound_ &&
            edge_point.point3d.z < z_max_bound_)
        {
          // Use a priority queue orderd by distance in x-direction
          strong_edges.push(edge_point);
        }
      }
    }


    ROS_INFO_STREAM("Found " << strong_edges.size() << " push_points from: "
                    << count510 << " edge candidates.");
#ifdef DEBUG_BOUNDS
    ROS_INFO_STREAM("X bounds are: (" << x_min << ", " << x_max << ")" );
    ROS_INFO_STREAM("Y bounds are: (" << y_min << ", " << y_max << ")" );
    ROS_INFO_STREAM("Z bounds are: (" << z_min << ", " << z_max << ")" );
#endif // DEBUG_BOUNDS

    PushPose::Response res;
    geometry_msgs::PoseStamped p;
    if (strong_edges.size() < 1)
    {
      res.push_pose = p;
      res.invalid_push_pose = true;
      ROS_INFO_STREAM("Found no points, returning a fake push_pose.");
      return res;
    }

    // Convert image loc to 3D loc using the point_cloud
    PushPoint push_point = strong_edges.top();
    strong_edges.pop();

    p.header.frame_id = height_map_frame_;
    p.pose.position.x = push_point.point3d.x;
    p.pose.position.y = push_point.point3d.y;
    p.pose.position.z = push_point.point3d.z;
    ROS_INFO_STREAM("Point at: (" << p.pose.position.x << ", "
                    << p.pose.position.y << ", " << p.pose.position.z << ")");

#ifdef DEBUG_PUSH_POINT
    cv::Mat output_frame;
    if (input_frame.channels() != 3)
    {
      cv::cvtColor(input_frame, output_frame, CV_GRAY2RGB);
    }
    else
    {
      output_frame = input_frame;
    }
    cv::circle(output_frame, push_point.point2d, 10, cv::Scalar(0,255,0), 2);
    // cv::imshow("Intensity img", output_frame);
    cv::Mat output_save_frame(output_frame.size(), CV_16UC3);
    output_frame.convertTo(output_save_frame, CV_16UC3);
    std::stringstream output_path;
    output_path << "/u/thermans/data/push_point" << num_calls_ << ".png";
    cv::imwrite(output_path.str(), output_save_frame);
#endif // DEBUG_PUSH_POINT

    // Determine pushing direction
    ROS_INFO_STREAM("Getting orientation.");
    // p.pose.orientation = getPushOrientation(height_map, push_point);
    res.push_pose = p;
    res.invalid_push_pose = false;
    return res;
  }

  /**
   * Method to determine the pushing angle of the end effector for tabletop
   * pushing
   *
   * @param height_map A point cloud of the tabletop
   * @param push_point The (x,y) location in the image, at which to push
   *
   * @return The orientation of the end effector with respect to the
   *         base_link for pushing.
   */
  geometry_msgs::Quaternion getPushOrientation(XYZPointCloud& height_map,
                                               PushPoint push_point)
  {

    // Generate image patch around the height_map point
    int patch_radius = patch_diameter_/2;
    cv::Mat height_img(patch_diameter_, patch_diameter_, CV_64FC1);
    // cv::Mat x_img(patch_diameter_, patch_diameter_, CV_64FC1);
    // cv::Mat y_img(patch_diameter_, patch_diameter_, CV_64FC1);
    cv::Mat height_dx(patch_diameter_, patch_diameter_, CV_64FC1);
    cv::Mat height_dy(patch_diameter_, patch_diameter_, CV_64FC1);

    ROS_INFO_STREAM("Making height map.");
    for (int r = push_point.point2d.y - patch_radius, r_set = 0;
         r <= push_point.point2d.y + patch_radius; ++r, ++r_set)
    {
      for (int c = push_point.point2d.x - patch_radius, c_set = 0;
           c <= push_point.point2d.x + patch_radius; ++c, ++c_set)
      {
        height_img.at<float>(r_set, c_set) = height_map.points[
            r*height_map.width + c].z;
        // x_img.at<float>(r_set, c_set) = height_map.points[
        //     r*height_map.width + c].x;
        // y_img.at<float>(r_set, c_set) = height_map.points[
        //     r*height_map.width + c].y;
      }
    }

    // Convolve with Sobel filters to determine strongest gradient orientation
    // TODO: Test if this is better with Scharr filters?
    ROS_INFO_STREAM("Getting height map edges.");
    cv::Sobel(height_img, height_dx, height_dx.depth(), 1, 0, sobel_size_);
    cv::Sobel(height_img, height_dy, height_dy.depth(), 0, 1, sobel_size_);

#ifdef DEBUG_PUSH_ORIENTATION
    cv::imshow("height_img", height_img);
    cv::imshow("height_dx", height_dx);
    cv::imshow("height_dy", height_dy);
    // cv::imshow("img_x", x_img);
    // cv::imshow("img_y", y_img);
    cv::waitKey(3);
#endif // DEBUG_PUSH_ORIENTATION

    ROS_INFO_STREAM("Calculating height map orientations.");
    int center = patch_radius + 1;
    double yaw = atan2(height_dy.at<double>(center, center),
                       height_dx.at<double>(center, center));
    ROS_INFO_STREAM("(R, P, Y): (" << 0.0 << ", " << 0.0 << ", " << yaw << ")");

    geometry_msgs::Quaternion q;
    try
    {
      ROS_INFO_STREAM("Transforming to quaternion.");
      q = tf::createQuaternionMsgFromYaw(yaw);
    }
    catch (tf::TransformException te)
    {
      ROS_ERROR_STREAM("Error converting yaw to Quaternion: " << te.what());
    }
    return q;
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
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image,
                                    sensor_msgs::PointCloud2> sync_;
  tf::TransformListener tf_;
  ros::ServiceServer push_point_server_;
  ros::Publisher converted_pcl_cloud_pub_;
  double int_canny_thresh_1_;
  double int_canny_thresh_2_;
  double depth_canny_thresh_1_;
  double depth_canny_thresh_2_;
  double x_min_bound_;
  double x_max_bound_;
  double y_min_bound_;
  double y_max_bound_;
  double z_min_bound_;
  double z_max_bound_;
  int patch_diameter_;
  sensor_msgs::CvBridge bridge_;
  std::string height_map_frame_;
  int sobel_size_;
  cv::Mat cur_frame_;
  cv::Mat cur_depth_frame_;
  XYZPointCloud cur_height_map_;
  bool have_depth_data_;
  int num_calls_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tabletop_perception_node");
  ros::NodeHandle n;
  TabletopPushingPerceptionNode perception_node(n);
  perception_node.spin();
}
