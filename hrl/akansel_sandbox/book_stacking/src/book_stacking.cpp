#define DEBUG_DRAW_TABLE_MARKERS
#define DEBUG_DRAW_TABLETOP_OBJECTS

#include <plane_extractor.h>

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <std_msgs/String.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

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

//STL
#include <string.h>

//VISUALIZATION
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
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
  ros::ServiceClient set_planning_scene_diff_client;
  //ros::ServiceClient ik_client;
  //ros::ServiceClient query_client;

public:
  ros::NodeHandle n_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher plane_marker_pub_;
  ros::Publisher obj_marker_pub_;
  ros::Publisher vis_pub_;
  ros::Subscriber command_subscriber_;
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


  //ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  //query_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");

  //ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
  //ik_client = n_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");
  set_planning_scene_diff_client = n_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);


 filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",1);
 plane_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("akans_plane_marker_array",1);
 obj_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obj_markers",1);
 vis_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker",1);
command_subscriber_=n_.subscribe<std_msgs::String>("/command_generator_PR2_topic",1,&book_stacking::commandCallback, this);

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
  	
/*
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
  }
  */
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


void commandCallback  (const std_msgs::String::ConstPtr& msg)
{
	std::cout<<"Command: "<<msg->data<<std::endl;
	char key0 = msg->data.c_str()[0];

	switch (key0)
	{
	case 'b': //message is to follower module.

		if(msg->data.length()>2)
		{
			char key2 = msg->data.c_str()[2];
			switch (key2)
			{
			case 'd': //detect objects
				
			robot_initialized=true;
			break;
			default:
			  break;
			}
		}
	default:
	  break;
	}
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

  arm_navigation_msgs::SimplePoseConstraint prepush_constraints;
  //prepush_pose.header.frame_id = objInfo.header.frame_id;
  prepush_constraints.header.frame_id = "torso_lift_link";
  prepush_constraints.link_name = "r_gripper_tool_frame";
  //prepush_constraints.link_name = "r_wrist_roll_link";

  double pad_dist=0.08;
  double gripper_offset=0.00;	
  geometry_msgs::PointStamped input_point;
  input_point.header.frame_id=objInfo.header.frame_id;
  input_point.point.x=objInfo.centroid.x;
  input_point.point.y=objInfo.centroid.y;
  input_point.point.z=objInfo.centroid.z;
  geometry_msgs::PointStamped output_point;
  tf_listener.transformPoint("torso_lift_link", input_point, output_point);
  double startX=output_point.point.x-dir.vector.x*(pad_dist+gripper_offset);
  double startY=output_point.point.y-dir.vector.y*(pad_dist+gripper_offset);
  double startZ=output_point.point.z-dir.vector.z*(pad_dist+gripper_offset);

  prepush_constraints.pose.position.x = startX;
  prepush_constraints.pose.position.y = startY;
  prepush_constraints.pose.position.z = startZ;
  prepush_constraints.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,M_PI/2,0.0);
  //prepush_constraints.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);


    geometry_msgs::PoseStamped tpose;
    tpose.header=prepush_constraints.header;
    tpose.pose.orientation=prepush_constraints.pose.orientation;
    tpose.pose.position=prepush_constraints.pose.position;
/*  tpose.pose.position.x=prepush_constraints.pose.position.x;
    tpose.pose.position.y=prepush_constraints.pose.position.y;
    tpose.pose.position.z=prepush_constraints.pose.position.z;
    tpose.pose.orientation.x=prepush_constraints.pose.orientation.x;
    tpose.pose.orientation.y=prepush_constraints.pose.orientation.y;
    tpose.pose.orientation.z=prepush_constraints.pose.orientation.z;
    tpose.pose.orientation.w=prepush_constraints.pose.orientation.w;
    */


 sensor_msgs::JointState solution;
 std::string link_name=prepush_constraints.header.frame_id;

bool ik_sln_found=false;
for (double zOff=0.05; zOff<0.11;zOff=zOff+0.01)
{
tpose.pose.position.z=startZ+zOff;
for (double pitchOff=M_PI/2*1.2; pitchOff>M_PI/2*0.4;pitchOff=pitchOff-0.08)
{
tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,pitchOff,0.0);

  if (computeIK(tpose, link_name, solution))
  {
        
	prepush_constraints.pose.orientation=tpose.pose.orientation;
        prepush_constraints.pose.position=tpose.pose.position;
        ik_sln_found=true;
	break;
  }
  else
  {
  //std::cout<<"zOff: "<<zOff<<", pitch: "<<pitchOff<<" didn't work"<<std::endl;
  }
}
if(ik_sln_found)
break;
}

  visualization_msgs::Marker mr;
  mr.header.frame_id=prepush_constraints.header.frame_id;
  mr.header.stamp=ros::Time::now();
  mr.type=visualization_msgs::Marker::ARROW;
  mr.action=visualization_msgs::Marker::ADD;
  mr.pose=prepush_constraints.pose;
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
PlaceEndEffector(true,move_right_arm_client_,prepush_constraints,false);

//Move the arm to the end position.
//PlaceEndEffector(true,move_right_arm_client_,end_pose,true);
return true;
}

/*
void transformPoint(const tf::TransformListener& listener,std::string from, double fromX, double fromY, double fromZ, std::string to, geometry_msgs::PointStamped &outputPoint)
{
  geometry_msgs::PointStamped fromPt;
  fromPt.header.frame_id = from;
  fromPt.header.stamp = ros::Time();
  fromPt.point.x = fromX;
  fromPt.point.y = fromY;
  fromPt.point.z = fromZ;
  try
  {
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform a point from : %s", ex.what());
  }


}
*/


  bool computeIK(const geometry_msgs::PoseStamped &tpose,  
                 const std::string &link_name, 
                 sensor_msgs::JointState &solution)
  {
    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    
    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      return -1;
    }

  ros::NodeHandle rh;
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");

    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    if(query_client.call(request,response))
      {
	for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
	  {
	    ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
	  }
      }
    else
      {
	ROS_ERROR("Could not call query service");
      }

  kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

  gpik_req.timeout = ros::Duration(5.0);
  //gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";
  gpik_req.ik_request.ik_link_name = "r_gripper_tool_frame";

  /*  gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.75;
  gpik_req.ik_request.pose_stamped.pose.position.y = -0.188;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
  */

  gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  //gpik_req.ik_request.pose_stamped.header.frame_id = tpose.header.frame_id;
  gpik_req.ik_request.pose_stamped.pose.position.x = tpose.pose.position.x;
  gpik_req.ik_request.pose_stamped.pose.position.y = tpose.pose.position.y;
  gpik_req.ik_request.pose_stamped.pose.position.z = tpose.pose.position.z;
  gpik_req.ik_request.pose_stamped.pose.orientation.x = tpose.pose.orientation.x;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = tpose.pose.orientation.y;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = tpose.pose.orientation.z;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = tpose.pose.orientation.w;

  /*std::cout<<"Req IK position: "<<gpik_req.ik_request.pose_stamped.pose.position.x<<" "<<
		             gpik_req.ik_request.pose_stamped.pose.position.y<<" "<<
                             gpik_req.ik_request.pose_stamped.pose.position.z<<" "<<
		             ". Orient: "<<
gpik_req.ik_request.pose_stamped.pose.orientation.x<<" "<<
gpik_req.ik_request.pose_stamped.pose.orientation.y<<" "<<
gpik_req.ik_request.pose_stamped.pose.orientation.z<<" "<<std::endl;
*/

  //tpose.header=prepush_pose.header;


  ros::service::waitForService("pr2_right_arm_kinematics/get_constraint_aware_ik");
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("pr2_right_arm_kinematics/get_constraint_aware_ik");



  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i
].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
  }
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
      {        ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
      }
      return true;
    }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Inverse kinematics service call failed");
    return false;
  }


  }

bool PlaceEndEffector(bool use_right_arm, ArmActionClient *arm_ac_client_, arm_navigation_msgs::SimplePoseConstraint &desired_pose,bool disable_gripper)
{


  std::cout<<"Req IK position(PlaceEndEffector):" <<desired_pose.pose.position.x<<" "<<
		             desired_pose.pose.position.y<<" "<<
                             desired_pose.pose.position.z<<" "<<
		             ". Orient: "<<
desired_pose.pose.orientation.x<<" "<<
desired_pose.pose.orientation.y<<" "<<
desired_pose.pose.orientation.z<<" "<<std::endl;



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
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);  
  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = desired_pose.header.frame_id;
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = desired_pose.link_name;
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_gripper_tool_frame";

  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = desired_pose.pose.position.x;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = desired_pose.pose.position.y;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z =desired_pose.pose.position.z;
goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.05);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";
  //goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = desired_pose.header.frame_id;
//  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_gripper_tool_frame";

  //goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = desired_pose.link_name;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = desired_pose.pose.orientation.x;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = desired_pose.pose.orientation.y;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = desired_pose.pose.orientation.z;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = desired_pose.pose.orientation.w;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.05;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.05;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.05;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

/*
  motion_planning_msgs::PositionConstraint pc;
  pc.header.stamp = ros::Time::now();
  pc.header.frame_id = "torso_lift_link";    
  pc.link_name = "r_elbow_flex_link";
  pc.position.x = desired_pose.pose.orientation.x;
  pc.position.y = desired_pose.pose.orientation.y;
  pc.position.z = 2.5;    
  pc.constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_orientation.w = 1.0;
  pc.weight = 1.0;
*/ 
//arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);


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

    //Transform it to base frame
    tf::StampedTransform transf;
    try{
      tf_listener.waitForTransform(base_frame_tf, raw_cloud.header.frame_id,
				   cloud_msg->header.stamp, ros::Duration(2.0));
      tf_listener.lookupTransform(base_frame_tf, raw_cloud.header.frame_id,
				  cloud_msg->header.stamp, transf);
    }
    catch(tf::TransformException ex)
    {
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
	pushVector.header.frame_id="torso_lift_link";
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
