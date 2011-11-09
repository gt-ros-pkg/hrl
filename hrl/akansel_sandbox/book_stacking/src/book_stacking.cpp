//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl_ros/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

//PR2
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>



typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;


class book_stacking
{

private:
  PointHeadClient* point_head_client_;
  ros::Subscriber pc_sub_;
  double min_table_z_, max_table_z_,leaf_size;
  bool use_downsample;
  double table_ransac_thresh_;
public:
ros::NodeHandle nh;


book_stacking():
nh("~")
{
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  while(!point_head_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
pc_sub_=nh.subscribe("/camera/depth/points",1,&book_stacking::KinectCallback,this);

 LoadParameters();

lookAt("base_link", 1.5, 0.0, 0.2);
//TestArm();
//shakeHead(2);
}
  void LoadParameters()
  {
    nh.param("min_table_z", min_table_z_, -0.5);
    nh.param("max_table_z", max_table_z_, 1.5);
    nh.param("use_downsample", use_downsample, true);
    nh.param("leaf_size", leaf_size, 0.01);
  }

void lookAt(std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
  }

  void shakeHead(int n)
  {
    int count = 0;
    while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
      lookAt("base_link", 5.0, 1.0, 1.2);

      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
      lookAt("base_link", 5.0, -1.0, 1.2);
    }
  }


void TestArm()
{
ROS_INFO("In TestArm()");
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.188;
  desired_pose.pose.position.z = 0;

  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);



  if (nh.ok())
  {
    bool finished_within_time = false;
ROS_INFO("BOOKSTACK Giving Goal");
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }

}

void KinectCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
ROS_INFO("PT CLOUD");
}

geometry_msgs::PoseStamped getTablePlane(XYZPointCloud& cloud)
  {
    XYZPointCloud cloud_downsampled;
    if (use_downsample)
    {
      pcl::VoxelGrid<pcl::PointXYZ> downsample;
      downsample.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud));
      downsample.setLeafSize(leaf_size,leaf_size,leaf_size);
      downsample.filter(cloud_downsampled);
    }

    // Filter Cloud to not look for table planes on the ground
    XYZPointCloud cloud_z_filtered;
    pcl::PassThrough<pcl::PointXYZ> z_pass;
    if (use_downsample)
    {
      z_pass.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud_downsampled));
    }
    else
    {
      z_pass.setInputCloud(
          boost::make_shared<XYZPointCloud>(cloud));
    }
    z_pass.setFilterFieldName ("z");
    z_pass.setFilterLimits(min_table_z_, max_table_z_);
    z_pass.filter(cloud_z_filtered);

    // Segment the tabletop from the points using RANSAC plane fitting
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices plane_inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType (pcl::SACMODEL_PLANE);
    plane_seg.setMethodType (pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold (table_ransac_thresh_);
    plane_seg.setInputCloud (
        boost::make_shared<XYZPointCloud>(cloud_z_filtered));
    plane_seg.segment(plane_inliers, coefficients);

    // Check size of plane_inliers
    if (plane_inliers.indices.size() < 1)
    {
      ROS_WARN_STREAM("No points found by RANSAC plane fitting");
      geometry_msgs::PoseStamped p;
      p.pose.position.x = 0.0;
      p.pose.position.y = 0.0;
      p.pose.position.z = 0.0;
      p.header = cloud.header;
      return p;
    }

    // Extract the plane members into their own point cloud
    XYZPointCloud plane_cloud;
    pcl::copyPointCloud(cloud_z_filtered, plane_inliers, plane_cloud);

    // Return plane centroid
    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid(plane_cloud, xyz_centroid);
    geometry_msgs::PoseStamped p;
    p.pose.position.x = xyz_centroid[0];
    p.pose.position.y = xyz_centroid[1];
    p.pose.position.z = xyz_centroid[2];
    p.header = cloud.header;
    ROS_INFO_STREAM("Table centroid is: ("
                    << p.pose.position.x << ", "
                    << p.pose.position.y << ", "
                    << p.pose.position.z << ")");
    //drawTablePlaneOnImage(plane_cloud, p);
    return p;
  }


};

int main(int argc, char** argv)
{
ros::init(argc, argv, "BookStackingNode");
//ROS_INFO("BOOK STACKING INIT");
book_stacking bs;
//bs.TestArm();
ros::spin();

}
