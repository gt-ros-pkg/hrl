#define DEBUG_DRAW_TABLE_MARKERS
#define DEBUG_DRAW_TABLETOP_OBJECTS

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
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

//TF
#include <tf/transform_listener.h>

//PR2
#include <pr2_controllers_msgs/PointHeadAction.h>

//Manipulation
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

//STL
#include <string.h>

//VISUALIZATION
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <plane_extractor.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class book_stacking
{

private:
  PointHeadClient* point_head_client_;
  ros::Subscriber point_cloud_sub_;
  double min_table_z_, max_table_z_,leaf_size;
  bool use_downsample;
  double table_ransac_thresh_;
  tf::TransformListener tf_listener;
  std::string workspace_frame;

  bool filter_spatial;
  double filter_spatial_xmin, filter_spatial_xmax;
  double filter_spatial_ymin, filter_spatial_ymax;
  double filter_spatial_zmin, filter_spatial_zmax;

  bool filter_outliers;  
  int filter_outliers_meank;
  double filter_outliers_stddev_thresh;
  bool downsample_cloud;
  double downsample_grid_size_;

  bool concave_hull_mode_;
  bool use_normal_seg_;
  int max_planes_;
  bool use_omp_;
  double plane_distance_thresh_;
  double normal_search_radius_;
  int min_plane_inliers_;
  int max_sac_iterations_;
  double sac_probability_;

  std::string base_frame_tf;

public:
  ros::NodeHandle n_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher plane_marker_pub_;
  ros::Publisher obj_marker_pub_;

book_stacking():
  n_("~")
{
  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  while(!point_head_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
point_cloud_sub_=n_.subscribe("/camera/depth/points",1,&book_stacking::KinectCallback,this);
filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",1);
 
//Visualization Publishers
 plane_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("akans_plane_marker_array",1);
 obj_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obj_markers",1);

 LoadParameters();

 lookAt("base_link", 1.5, 0.0, 0.2);
//TestArm();
//shakeHead(2);
}
  void LoadParameters()
  {
/*
    n_.param("min_table_z", min_table_z_, -0.5);
    n_.param("max_table_z", max_table_z_, 1.5);
    n_.param("use_downsample", use_downsample, true);
    n_.param("leaf_size", leaf_size, 0.01); 
    std::string default_workspace_frame="/base_link";
    n_.param("workspace_frame", workspace_frame, default_workspace_frame);*/

    //Spatial Filtering Params
    n_.param("filter_spatial",filter_spatial,true);
    n_.param("filter_spatial_zmax",filter_spatial_zmax,3.0);
    n_.param("filter_spatial_zmin",filter_spatial_zmin,0.05);
    n_.param("filter_spatial_ymax",filter_spatial_ymax,10.0);
    n_.param("filter_spatial_ymin",filter_spatial_ymin,-10.0);
    n_.param("filter_spatial_xmax",filter_spatial_xmax,10.0);
    n_.param("filter_spatial_xmin",filter_spatial_xmin,-10.0);

    //Outlier Filtering Params
    n_.param("filter_outliers",filter_outliers,true);
    n_.param("filter_outliers_meank",filter_outliers_meank,50);
    n_.param("filter_outliers_stddev_thresh", filter_outliers_stddev_thresh,1.0);

    //Downsampling Params
    n_.param("downsample_cloud",downsample_cloud,true);
    n_.param("downsample_grid_size",downsample_grid_size_,0.01);

    //Plane Extraction Parameters
    n_.param("max_planes",max_planes_,4);
    n_.param("use_omp",use_omp_,false);
    n_.param("plane_distance_thresh",plane_distance_thresh_,0.03);
    n_.param("normal_search_radius",normal_search_radius_,0.1);
    n_.param("min_plane_inliers",min_plane_inliers_,1000);
    n_.param("max_sac_iterations",max_sac_iterations_,1000);
    n_.param("sac_probability",sac_probability_,0.99);
    n_.param("concave_hull_mode",concave_hull_mode_,false);
    n_.param("use_normal_seg",use_normal_seg_,false);
    n_.param("base_frame_tf", base_frame_tf,std::string("/base_link"));

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



  if (n_.ok())
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

void KinectCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
ROS_INFO("PT CLOUD");
 XYZPointCloud raw_cloud;
 XYZPointCloud cloud;
 pcl::fromROSMsg(*cloud_msg,raw_cloud);


 //pcl_ros::transformPointCloud(workspace_frame,cloud,cloud,tf_listener); //didn't work.

    //Transform it to base frame
    tf::StampedTransform transf;
    try{
      tf_listener.waitForTransform(base_frame_tf, raw_cloud.header.frame_id,
				   cloud_msg->header.stamp, ros::Duration(2.0));
      tf_listener.lookupTransform(base_frame_tf, raw_cloud.header.frame_id,
				  cloud_msg->header.stamp, transf);
    }catch(tf::TransformException ex){
      ROS_ERROR("Scene segmentation unable to put kinect data in ptu reference frame due to TF error:%s", ex.what());
      return;
    }
    
    tf::Vector3 v3 = transf.getOrigin();
    tf::Quaternion quat = transf.getRotation();
    Eigen::Quaternionf rot(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::Vector3f offset(v3.x(), v3.y(), v3.z());
    pcl::transformPointCloud(raw_cloud,cloud,offset,rot);
    cloud.header = raw_cloud.header;
    cloud.header.frame_id = base_frame_tf;
   
    

 book_stacking_msgs::PlaneInfo table_plane_info=getTablePlane(cloud);

 bool detect_objects=true;
    if(detect_objects)
      {
	ROS_INFO("Extracting objects...");
	book_stacking_msgs::ObjectInfos objects = getObjectsOverPlane(table_plane_info,cloud,-2.0,2.0);
	  
#ifdef DEBUG_DRAW_TABLETOP_OBJECTS
	if(objects.objects.size() < 1)
	  {
	    ROS_WARN("No objects over this plane.");
	  } 
	else 
	  {
	ROS_INFO("# OF OBJS: %d",objects.objects.size());
	    drawObjectPrisms(objects,obj_marker_pub_,table_plane_info,0.0f,1.0f,0.0f);
	    //object_pub_.publish(objects);
	  }	  
      }
#endif

    
}

book_stacking_msgs::PlaneInfo getTablePlane(XYZPointCloud& cloud)
  {
    if(filter_spatial)
      {
	pcl::PassThrough<Point> pass;
	pass.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
	pass.setFilterFieldName("z");
	pass.setFilterLimits(filter_spatial_zmin,filter_spatial_zmax);
	pass.filter(cloud);
	pass.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
	pass.setFilterFieldName("x");
	pass.setFilterLimits(filter_spatial_xmin,filter_spatial_xmax);
	pass.filter(cloud);
	pass.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
	pass.setFilterFieldName("y");
	pass.setFilterLimits(filter_spatial_ymin,filter_spatial_ymax);
	pass.filter(cloud);
      }

    //Filter Outliers
    if(filter_outliers){
      pcl::StatisticalOutlierRemoval<Point> sor_filter;
      sor_filter.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
      sor_filter.setMeanK(filter_outliers_meank);//50
      sor_filter.setStddevMulThresh(filter_outliers_stddev_thresh);
      sor_filter.filter(cloud);
    }
    
    //Downsample Cloud
    if(downsample_cloud)
      {
      //downsample
      pcl::VoxelGrid<Point> grid;
      grid.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(cloud));
      grid.setLeafSize(downsample_grid_size_,downsample_grid_size_,downsample_grid_size_);//0.05
      grid.filter(cloud);
    }

    //Publish the filtered cloud
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(cloud,filtered_msg);
    ROS_INFO("Publishing filtered cloud...");
    filtered_cloud_pub_.publish(filtered_msg);

    book_stacking_msgs::PlaneInfos plane_infos= getPlanesByNormals(cloud,4,true,concave_hull_mode_,use_omp_,plane_distance_thresh_,max_sac_iterations_,sac_probability_,min_plane_inliers_,normal_search_radius_,0.1);
    plane_infos.header.stamp=cloud.header.stamp; 
#ifdef DEBUG_DRAW_TABLE_MARKERS
    drawPlaneMarkers(plane_infos,plane_marker_pub_,1.0,0.0,0.0);
#endif  
    return plane_infos.planes[0];

  }


};

int main(int argc, char** argv)
{
ros::init(argc, argv, "BookStackingNode");
book_stacking bs;
//bs.TestArm();
ros::spin();

}
