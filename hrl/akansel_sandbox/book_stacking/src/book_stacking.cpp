#define DEBUG_DRAW_TABLE_MARKERS
#define DEBUG_DRAW_TABLETOP_OBJECTS

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <arm_navigation_msgs/CollisionOperation.h>

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
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> ArmActionClient;

class book_stacking
{

private:
  PointHeadClient* point_head_client_;
  TorsoClient *torso_client_;
  ros::Subscriber point_cloud_sub_;
  tf::TransformListener tf_listener;
  ArmActionClient *move_right_arm_client_;
  ArmActionClient *move_left_arm_client_;

  std::string workspace_frame;
  std::string base_frame_tf;

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
  bool robot_initialized;


public:
  ros::NodeHandle n_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher plane_marker_pub_;
  ros::Publisher obj_marker_pub_;
  ros::Publisher vis_pub_;

book_stacking():
  n_("~")
{
 robot_initialized=false; 
 LoadParameters();
 InitializeRobot();
 robot_initialized=true; 
//TestArm();
//shakeHead(2);
}

 void InitializeRobot()
{
 filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",1);
 plane_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("akans_plane_marker_array",1);
 obj_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obj_markers",1);
 vis_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker",1);

  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
    while(!torso_client_->waitForServer(ros::Duration(5.0)))
    {
      //ROS_INFO("Waiting for the torso action server to come up");
    }

  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  while(!point_head_client_->waitForServer(ros::Duration(5.0)))
    {
      //ROS_INFO("Waiting for the point_head_action server to come up");
    }

  move_right_arm_client_ = new ArmActionClient("move_right_arm",true);  
  while(!move_right_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      //ROS_INFO("Waiting for the point_head_action server to come up");
    }

  move_left_arm_client_ = new ArmActionClient("move_left_arm",true);
  while(!move_left_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      //ROS_INFO("Waiting for the point_head_action server to come up");
    }
  	
  
 moveTorsoToPosition(0.2);
 lookAt("base_link", 1.0, 0.0, 0.0);
 point_cloud_sub_=n_.subscribe("/camera/rgb/object_modeling_points_filtered",1,&book_stacking::KinectCallback,this);

/*
  visualization_msgs::Marker mr;
  mr.header.frame_id=start_pose.header.frame_id;
  mr.header.stamp=ros::Time::now();
  mr.type=visualization_msgs::Marker::ARROW;
  mr.action=visualization_msgs::Marker::ADD;
  mr.pose=start_pose.pose;
  mr.scale.x=0.1;
  mr.scale.y=0.1;
  mr.scale.z=0.1;
  mr.color.a=1.0;
  mr.color.r=0.0;
  mr.color.g=1.0;
  mr.color.b=1.0;
  vis_pub_.publish(mr); */

}
 
void moveTorsoToPosition(double d) //0.2 is max up, 0.0 is min.
 {
    pr2_controllers_msgs::SingleJointPositionGoal q;
    q.position = d;  //all the way up is 0.2
    q.min_duration = ros::Duration(2.0);
    q.max_velocity = 1.0;
    torso_client_->sendGoal(q);
    torso_client_->waitForResult();
  }


  void LoadParameters()
  {
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



bool pushObject(book_stacking_msgs::ObjectInfo objInfo, geometry_msgs::Vector3Stamped dir,double dist)
{
//determine start position, given the object
/*
  arm_navigation_msgs::SimplePoseConstraint start_pose;
  start_pose.header.frame_id = "torso_lift_link";
  start_pose.link_name = "r_wrist_roll_link";
  start_pose.pose.position.x = 0.55;
  start_pose.pose.position.y = -0.188;
  start_pose.pose.position.z = 0;
  start_pose.pose.orientation.x = 0.0;
  start_pose.pose.orientation.y = 0.0;
  start_pose.pose.orientation.z = 0.0;
  start_pose.pose.orientation.w = 1.0;
*/

  arm_navigation_msgs::SimplePoseConstraint prepush_pose;
  prepush_pose.header.frame_id = objInfo.header.frame_id;

  std::cout<<"Object Frame ID: "<< objInfo.header.frame_id<<std::endl;
  //start_pose.link_name = "r_gripper_tool_frame";
  prepush_pose.link_name = "r_wrist_roll_link";

  double pad_dist=0.08;
  double gripper_offset=0.00;
	

  double startX=objInfo.centroid.x-dir.vector.x*(pad_dist+gripper_offset);
  double startY=objInfo.centroid.y-dir.vector.y*(pad_dist+gripper_offset);
  double startZ=objInfo.centroid.z-dir.vector.z*(pad_dist+gripper_offset)+0.05;
  prepush_pose.pose.position.x = startX;
  prepush_pose.pose.position.y = startY;
  prepush_pose.pose.position.z = startZ;
  //prepush_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,M_PI/2);
  prepush_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);

  visualization_msgs::Marker mr;
  mr.header.frame_id=prepush_pose.header.frame_id;
  mr.header.stamp=ros::Time::now();
  mr.type=visualization_msgs::Marker::ARROW;
  mr.action=visualization_msgs::Marker::ADD;
  mr.pose=prepush_pose.pose;
  mr.scale.x=0.1;
  mr.scale.y=0.1;
  mr.scale.z=0.1;
  mr.color.a=1.0;
  mr.color.r=1.0;
  mr.color.g=0.2;
  mr.color.b=1.0;
  vis_pub_.publish(mr); 

//determine end position, given the dist and dir of the push
/*
  arm_navigation_msgs::SimplePoseConstraint end_pose;
  end_pose.header.frame_id = "torso_lift_link";
  end_pose.link_name = "r_wrist_roll_link";
  end_pose.pose.position.x = 0.75;
  end_pose.pose.position.y = -0.188;
  end_pose.pose.position.z = 0.0;
  end_pose.pose.orientation.x = 0.0;
  end_pose.pose.orientation.y = 0.0;
  end_pose.pose.orientation.z = 0.0;
  end_pose.pose.orientation.w = 1.0;
*/
//move the base if necessary.

//Move the arm to the start position.
PlaceEndEffector(true,move_right_arm_client_,prepush_pose,false);

//Move the arm to the end position.
//PlaceEndEffector(true,move_right_arm_client_,end_pose,true);
return true;
}

bool PlaceEndEffector(bool use_right_arm, ArmActionClient *arm_ac_client_, arm_navigation_msgs::SimplePoseConstraint &desired_pose,bool disable_gripper)
{
  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;
  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;

  arm_navigation_msgs::MoveArmGoal goalA;
	if(use_right_arm)
	{
	  goalA.motion_plan_request.group_name = "right_arm";
	}
	else
	{
	  goalA.motion_plan_request.group_name = "left_arm";
	}

  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(2.0);  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);


  bool finished_within_time = false; 
if(disable_gripper)
{
  arm_navigation_msgs::CollisionOperation coll_disable_msg;
	if(use_right_arm)
	{
	 coll_disable_msg.object1 = "r_end_effector";
	}
	else
	{
	 coll_disable_msg.object1 = "l_end_effector";
	}
 
  coll_disable_msg.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  coll_disable_msg.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  goalA.operations.collision_operations[0] = coll_disable_msg;
}

//    std::cout<<"Object Frame ID: "<< goalA.<<std::endl;
    arm_ac_client_->sendGoal(goalA);
    finished_within_time = arm_ac_client_->waitForResult(ros::Duration(20.0));
    if (!finished_within_time)
    {
      arm_ac_client_->cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = arm_ac_client_->getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
	return success;
    }
  return false;

}

void TestArm()
{
ROS_INFO("In TestArm()");

  arm_navigation_msgs::MoveArmGoal goalA;
  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  //desired_pose.link_name = "r_gripper_tool_frame";

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


    //ROS_INFO("BOOKSTACK Giving Goal");
     move_right_arm_client_->sendGoal(goalA);
    finished_within_time =   move_right_arm_client_->waitForResult(ros::Duration(200.0));
    if (!finished_within_time)
    {
      move_right_arm_client_->cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state =   move_right_arm_client_->getState();
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
if(!robot_initialized)
return;
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
   
    
book_stacking_msgs::PlaneInfo table_plane_info;
bool gotPlane=getTablePlane(cloud,table_plane_info);

if(gotPlane)
{
/*
 pcl::PointCloud<Point> table_cloud;
 pcl::toROSMsg(table_cloud,table_plane_info.hull);
 geometry_msgs::Point32 table_center=calcCentroid(table_cloud);
//lookAt(table_plane_info.header.frame_id,table_center.x,table_center.y,table_center.z);

 std::cout<<"Frame: "<<table_plane_info.header.frame_id<<" Center x: "<<table_center.x<<" Center y: "<<table_center.y<<" Center z: "<<table_center.z;

*/

 bool detect_objects=true;
    if(detect_objects)
      {

	ROS_INFO("Extracting objects...");
	book_stacking_msgs::ObjectInfos allObjectInfos = getObjectsOverPlane(table_plane_info,cloud,-1.01,-0.015);
	  

	if(allObjectInfos.objects.size() < 1)
	  {
	    ROS_WARN("No objects over this plane.");
	  } 
	else 
	  {
	ROS_INFO("# OF OBJS: %d",(int)(allObjectInfos.objects.size()));

#ifdef DEBUG_DRAW_TABLETOP_OBJECTS
	    drawObjectPrisms(allObjectInfos,obj_marker_pub_,table_plane_info,0.0f,1.0f,0.0f);
	    //object_pub_.publish(objects);
#endif

	book_stacking_msgs::ObjectInfo pushedObjectInfo=allObjectInfos.objects[0];
	
	geometry_msgs::Vector3Stamped pushVector;
	pushVector.header.frame_id="/base_link";
	pushVector.header.stamp=ros::Time::now();
	pushVector.vector.x=0.0;
	pushVector.vector.y=1.0;
	pushVector.vector.z=0.0;

	pushObject(pushedObjectInfo,pushVector, 0.15);
	robot_initialized=false;


	for (unsigned int i=0; i<allObjectInfos.objects.size();i++)
	{
/*
	geometry_msgs::Point32 bbox_min=allObjectInfos.objects[i].bbox_min;
	geometry_msgs::Point32 bbox_max=allObjectInfos.objects[i].bbox_max;
	 pcl::ModelCoefficients circle_coeff;
 	 circle_coeff.values.resize (3);    // We need 3 values
         circle_coeff.values[0] = 1.0;
         circle_coeff.values[1] = 1.0;
         circle_coeff.values[2] = 0.3;
         vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff, z);*/

	}

	  }	  
      }
}

    
}

bool getTablePlane(XYZPointCloud& cloud, book_stacking_msgs::PlaneInfo &pl_info)
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
    if(plane_infos.planes.size()>0)
    {
    drawPlaneMarkers(plane_infos,plane_marker_pub_,1.0,0.0,0.0);
    }
#endif
	if(plane_infos.planes.size()>0)
	{
	pl_info=plane_infos.planes[0];
	return true;
	}
	else
	{
	return false;	
	}

  }


};

int main(int argc, char** argv)
{
ros::init(argc, argv, "BookStackingNode");
book_stacking bs;
ros::spin();
}
