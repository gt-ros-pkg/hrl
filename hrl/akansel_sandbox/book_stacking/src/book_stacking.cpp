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
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/CollisionOperation.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <arm_navigation_msgs/MoveArmGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


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


static const std::string RIGHT_ARM_IK_NAME = "pr2_right_arm_kinematics/get_constraint_aware_ik";
static const std::string RIGHT_ARM_FK_NAME = "pr2_right_arm_kinematics/get_fk";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef pcl::PointXYZ Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> ArmActionClient;

class book_stacking
{

private:
  ros::NodeHandle root_handle_;
  ros::ServiceClient ik_client_right;
  ros::ServiceClient fk_client_right;
  kinematics_msgs::GetKinematicSolverInfo::Response ik_solver_info;
  kinematics_msgs::GetKinematicSolverInfo::Response fk_solver_info;
  PointHeadClient* point_head_client_;
  TorsoClient *torso_client_;
  ros::Subscriber point_cloud_sub_;
  tf::TransformListener tf_listener;
  ArmActionClient *move_right_arm_client_;
  ArmActionClient *move_left_arm_client_;
  TrajClient *traj_right_arm_client_;

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
  pr2_controllers_msgs::JointTrajectoryGoal rightArmHomingTrajectory;
  
/*
  ros::ServiceClient set_planning_scene_diff_client;
  ros::ServiceClient ik_client;
  ros::ServiceClient query_client;
*/

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

  TestArm();
  TestArm2();
  traj_right_arm_client_->sendGoal(rightArmHomingTrajectory);
  traj_right_arm_client_->waitForResult();
 
//robot_initialized=true; 

//move_right_gripper(0.75,-0.188,0,-M_PI/2,0,0,1);
//shakeHead(2);
}

bool callQueryFK(kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
ros::NodeHandle z;
ros::service::waitForService("pr2_right_arm_kinematics/get_fk_solver_info");
ros::ServiceClient query_client = z.serviceClient<kinematics_msgs::GetKinematicSolverInfo>
    ("pr2_right_arm_kinematics/get_fk_solver_info");
kinematics_msgs::GetKinematicSolverInfo::Request request;
if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s", i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    return false;
  }

return true;
}


bool callQueryIK(kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  ros::NodeHandle z;
  ros::service::waitForService("pr2_right_arm_kinematics/get_ik_solver_info");
  ros::ServiceClient query_client = z.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("pr2_right_arm_kinematics/get_ik_solver_info");
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
	return true;
  }
  else
  {
    ROS_ERROR("Could not call query service");
        return false;
  }



}


 void InitializeRobot()
{
    ik_client_right = root_handle_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>(RIGHT_ARM_IK_NAME);
    fk_client_right = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>(RIGHT_ARM_FK_NAME);
/*
  set_planning_scene_diff_client = n_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
*/

 point_cloud_sub_=n_.subscribe("/camera/rgb/object_modeling_points_filtered",1,&book_stacking::KinectCallback,this);
 filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",1);
 plane_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("akans_plane_marker_array",1);
 obj_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obj_markers",1);
 vis_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker",1);
command_subscriber_=n_.subscribe<std_msgs::String>("/command_generator_PR2_topic",1,&book_stacking::commandCallback, this);
 
  traj_right_arm_client_=new TrajClient("r_arm_controller/joint_trajectory_action", true);
 while(!traj_right_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      //ROS_INFO("Waiting for the joint_trajectory_action server");
    }
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

 callQueryIK(ik_solver_info);
 callQueryFK(fk_solver_info);
  	
 moveTorsoToPosition(0.1);//0.3
 lookAt("base_link", 1.0, 0.0, 0.0);
  rightArmHomingTrajectory=createRightArmHomingTrajectory();

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
ROS_INFO("Subs/Pubs Initialized..");
}
 

pr2_controllers_msgs::JointTrajectoryGoal createRightArmTrajectoryFromAngles(double angles[7], double t)
{
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    goal.trajectory.points.resize(2);

    int ind = 0;
    // trajectory point 1
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =angles[0];
    goal.trajectory.points[ind].positions[1] =angles[1];
    goal.trajectory.points[ind].positions[2] =angles[2];
    goal.trajectory.points[ind].positions[3] =angles[3];
    goal.trajectory.points[ind].positions[4] =angles[4];
    goal.trajectory.points[ind].positions[5] =angles[5];
    goal.trajectory.points[ind].positions[6] =angles[6];
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    goal.trajectory.points[ind].time_from_start = ros::Duration(t);
    return goal;
}

pr2_controllers_msgs::JointTrajectoryGoal createRightArmHomingTrajectory()
{
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    goal.trajectory.points.resize(2);

    int ind = 0;

    // trajectory point 1
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.165998663278225;
    goal.trajectory.points[ind].positions[1] = -0.42760446970174038;
    goal.trajectory.points[ind].positions[2] = -1.6653232834640597;
    goal.trajectory.points[ind].positions[3] = -1.189691671368649;
    goal.trajectory.points[ind].positions[4] = -20.00850591;
    goal.trajectory.points[ind].positions[5] = -1.61968241508;
    goal.trajectory.points[ind].positions[6] = 12.0600056906631;
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.5);



    // trajectory point 2
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -1.0622191969072416;
    goal.trajectory.points[ind].positions[1] = -0.42760446970174038;
    goal.trajectory.points[ind].positions[2] = -1.6653232834640597;
    goal.trajectory.points[ind].positions[3] = -1.189691671368649;
    goal.trajectory.points[ind].positions[4] = -20.00850591;
    goal.trajectory.points[ind].positions[5] = -1.61968241508;
    goal.trajectory.points[ind].positions[6] = 11.00822105617;
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.5);

    return goal;
}



  void get_current_joint_angles(double current_angles[7])
 {
    int i;
    //get a single message from the topic 'r_arm_controller/state'
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = 
      ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("r_arm_controller/state");
    
    //extract the joint angles from it
    for(i=0; i<7; i++)
    {
      current_angles[i] = state_msg->actual.positions[i];
    }
  }

bool run_fk(double angles[7], geometry_msgs::PoseStamped &solution, kinematics_msgs::GetKinematicSolverInfo::Response fk_solver_info)
{
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  fk_request.header.frame_id = "torso_lift_link";
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = "r_wrist_roll_link";
  fk_request.robot_state.joint_state.position.resize(fk_solver_info.kinematic_solver_info.joint_names.size());
  fk_request.robot_state.joint_state.name = fk_solver_info.kinematic_solver_info.joint_names;

  for(unsigned int i=0;i<fk_solver_info.kinematic_solver_info.joint_names.size(); i++)
  {
    fk_request.robot_state.joint_state.position[i] = angles[i];
  }

  if(fk_client_right.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
      {
        ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
        ROS_INFO_STREAM("Position: " << 
          fk_response.pose_stamped[i].pose.position.x << "," <<  
          fk_response.pose_stamped[i].pose.position.y << "," << 
          fk_response.pose_stamped[i].pose.position.z);
        ROS_INFO("Orientation: %f %f %f %f",
          fk_response.pose_stamped[i].pose.orientation.x,
          fk_response.pose_stamped[i].pose.orientation.y,
          fk_response.pose_stamped[i].pose.orientation.z,
          fk_response.pose_stamped[i].pose.orientation.w);

      solution=fk_response.pose_stamped;
      } 
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
  }
return true;
}


bool run_ik(geometry_msgs::PoseStamped pose, double start_angles[7], 
              double solution[7], std::string link_name,kinematics_msgs::GetKinematicSolverInfo::Response ik_solver_info)
{
  kinematics_msgs::GetConstraintAwarePositionIK::Request  ik_request;
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response;
  ik_request.timeout = ros::Duration(5.0);
  ik_request.ik_request.ik_seed_state.joint_state.name = ik_solver_info.kinematic_solver_info.joint_names;
  ik_request.ik_request.ik_link_name = link_name; 
  ik_request.ik_request.pose_stamped = pose;
  ik_request.ik_request.ik_seed_state.joint_state.position.resize(ik_solver_info.kinematic_solver_info.joint_names.size());

  for(unsigned int i=0; i< ik_solver_info.kinematic_solver_info.joint_names.size(); i++)
  {    
  ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];
  }

    ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);


if(ik_client_right.isValid())
{
std::cout<<"Valid"<<std::endl;
}
else
{
std::cout<<"Not Valid"<<std::endl;
}
if(ik_client_right.exists())
{
std::cout<<"Exists"<<std::endl;
}
else
{
std::cout<<"Not Exists"<<std::endl;
}
    bool ik_service_call = ik_client_right.call(ik_request,ik_response);
    if(!ik_service_call)
    {
      ROS_ERROR("IK service call failed!");  
      return false;
    }
    if(ik_response.error_code.val == ik_response.error_code.SUCCESS)
    {
      for(int i=0; i<7; i++)
	{
        solution[i] = ik_response.solution.joint_state.position[i];
        }
      ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", 
             solution[0],solution[1], solution[2],solution[3],
             solution[4],solution[5],solution[6]);
      ROS_INFO("IK service call succeeded");
      return true;
    }
   else
    {
      ROS_ERROR("run_ik no solution");
      return false;
    }
return true;
}




double MoveEndEffectorLinear(double dx,double dy,double dz, double speed, bool move_as_much_you_can)
{
  double current_angles[7];
  double solution_angles[7];
  get_current_joint_angles(current_angles);
/*
std::cout<<"Current Joint Angles: ";
for(int i=0;i<7;i++)
{ std::cout<<current_angles[i]<<" "; }
std::cout<<std::endl;
*/
  geometry_msgs::PoseStamped current_pose;
  run_fk(current_angles,current_pose,fk_solver_info);
  geometry_msgs::PoseStamped target_pose=current_pose;
  target_pose.pose.x=current_pose.pose.x+dx;
  target_pose.pose.y=current_pose.pose.y+dy;
  target_pose.pose.z=current_pose.pose.z+dz;
  std::string link_name="r_wrist_roll_link";
  bool ik_result=run_ik(target_pose,current_angles,solution_angles,link_name,ik_solver_info);
  double path_distance=0.0;
  if(!ik_result)
    {
      if(!move_as_much_you_can)
	{
	  return 0.0;
	}
      const int num_inter_checks=10;
      for(int i=num_inter_checks-1;i>0;i--)
	{
	  double path_ratio=i/num_inter_checks;
	  target_pose.pose.x=current_pose.pose.x+dx*path_ratio;
	  target_pose.pose.y=current_pose.pose.y+dy*path_ratio;
	  target_pose.pose.z=current_pose.pose.z+dz*path_ratio;
	  ik_result=run_ik(target_pose,current_angles,solution_angles,link_name,ik_solver_info)
	    if(ik_result) 
	      {
		path_distance=(dx+dy+dz)*path_ratio;
		break;
	      }
	}
    }
  else
    {
      path_distance=dx+dy+dz;
    }

  if(ik_result)    
    {           
      pr2_controllers_msgs::JointTrajectoryGoal traj createRightArmTrajectoryFromAngles(solution_angles,);
      traj_right_arm_client_->sendGoal(traj);
      traj_right_arm_client_->waitForResult();
    }

  


  /*
    geometry_msgs::PoseStamped pose;
    pose.header.stamp=ros::Time::now();
    pose.header.frame_id="torso_lift_link";
    pose.pose.position.x = 0.75;
    pose.pose.position.y = -0.188;
    pose.pose.position.z = 0;
    pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,M_PI/2);    
    std::string link_name="r_wrist_roll_link";
    bool ik_result=run_ik(pose,current_angles,solution_angles,link_name,ik_solver_info);
  */


}

void commandCallback  (const std_msgs::String::ConstPtr& msg)
{
	std::cout<<"Command: "<<msg->data<<std::endl;
	char key0 = msg->data.c_str()[0];

                        kinematics_msgs::GetKinematicSolverInfo::Response response;
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
			case 'z':
                        callQueryIK(response);
			break;
                        case 'a':
                        TestArm();
			break;
                        case 'b':
                        TestArm2();
			break;
                        case 'c':
                        TestArm3();
			break;
			case 'e':
  			traj_right_arm_client_->sendGoal(rightArmHomingTrajectory);
                        traj_right_arm_client_->waitForResult();
			break;
			case 'f':
			  MoveEndEffectorLinear(0.0,0.1,0.0,true);
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
  //prepush_constraints.link_name = "r_gripper_tool_frame";
  prepush_constraints.link_name = "r_wrist_roll_link";

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
  prepush_constraints.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);
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


//move_right_gripper(prepush_constraints.pose.position.x,prepush_constraints.pose.position.y,prepush_constraints.pose.position.z,prepush_constraints.pose.orientation.x,prepush_constraints.pose.orientation.y,prepush_constraints.pose.orientation.z,prepush_constraints.pose.orientation.w);

/*
   sensor_msgs::JointState tempsolution;
  if (computeIK(tpose, link_name, tempsolution,response))
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
*/


 sensor_msgs::JointState solution;
 std::string link_name="r_wrist_roll_link";

kinematics_msgs::GetKinematicSolverInfo::Response response;
callQueryIK(response);
bool ik_sln_found=false;
for (double zOff=0.04; zOff<0.11;zOff=zOff+0.005)
{
tpose.pose.position.z=startZ+zOff;
for (double pitchOff=M_PI/2*1.3; pitchOff>M_PI/2*0.6;pitchOff=pitchOff-0.05)
{
tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,pitchOff,0.0);

  visualization_msgs::Marker mr;
  mr.header.frame_id=tpose.header.frame_id;
  mr.header.stamp=ros::Time::now();
  mr.type=visualization_msgs::Marker::ARROW;
  mr.action=visualization_msgs::Marker::ADD;
  mr.pose=tpose.pose;
  mr.scale.x=0.06;
  mr.scale.y=0.06;
  mr.scale.z=0.06;
  mr.color.a=1.0;
  mr.color.r=1.0;
  mr.color.g=0.3;
  mr.color.b=1.0;
  vis_pub_.publish(mr); 



tpose.pose.position.x=0.75;
tpose.pose.position.y=-0.188;
tpose.pose.position.z=0.0;

  if (computeIK(tpose, link_name, solution,response))
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
if(ik_sln_found)
{
PlaceEndEffector(true,move_right_arm_client_,prepush_constraints,false);
}
//Move the arm to the end position.
//PlaceEndEffector(true,move_right_arm_client_,end_pose,true);
return true;
}



  bool computeIK(const geometry_msgs::PoseStamped &tpose,  
                 const std::string &link_name, 
                 sensor_msgs::JointState &solution, kinematics_msgs::GetKinematicSolverInfo::Response response)
  {
  
    ros::NodeHandle rh;
    arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
    arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;
    
    ros::ServiceClient set_planning_scene_diff_client = rh.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);
    if(!set_planning_scene_diff_client.call(planning_scene_req, planning_scene_res)) {
      ROS_WARN("Can't get planning scene");
      return -1;
    }

  kinematics_msgs::GetConstraintAwarePositionIK::Request  gpik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gpik_res;

  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "r_wrist_roll_link";

  /*  gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.75;
  gpik_req.ik_request.pose_stamped.pose.position.y = -0.188;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
  */

  //gpik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  gpik_req.ik_request.pose_stamped.header.frame_id = tpose.header.frame_id;
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


  std::cout<<"Req IK position(PlaceEndEffector): (" <<desired_pose.pose.position.x<<" "<<
		             desired_pose.pose.position.y<<" "<<
                             desired_pose.pose.position.z<<")"<<
		             ". Orient: ("<<
desired_pose.pose.orientation.x<<" "<<
desired_pose.pose.orientation.y<<" "<<
desired_pose.pose.orientation.z<<" "<<
desired_pose.pose.orientation.w<<"). frame_id: "<<
desired_pose.header.frame_id<<" "<<
" link_name:"<< desired_pose.link_name<<std::endl;

/*
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.188;
  desired_pose.pose.position.z = 0.0;
  desired_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2,0.0);*/

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
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = desired_pose.header.frame_id;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = desired_pose.link_name;
  //goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";


  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = desired_pose.pose.position.x;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = desired_pose.pose.position.y;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = desired_pose.pose.position.z;


/*
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.70;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.18; desired_pose.pose.position.y;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = -0.05; desired_pose.pose.position.z;
*/
goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.04);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.04);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.04);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  //goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = desired_pose.header.frame_id;
  //goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = desired_pose.link_name;

  

goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);
/*  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = desired_pose.pose.orientation.x;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = desired_pose.pose.orientation.y;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = desired_pose.pose.orientation.z;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = desired_pose.pose.orientation.w;*/
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;
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
/*
int move_right_gripper(float x, float y, float z, float x_quat, float y_quat, float z_quat, float w_quat)
{
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server: right gripper");
  arm_navigation_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  nh.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));




  arm_navigation_msgs::CollisionOperation collisionOperation_1;
  collisionOperation_1.object1 = "r_end_effector";
  collisionOperation_1.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  collisionOperation_1.penetration_distance = 2.0;
  collisionOperation_1.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  //goalA.disable_collision_monitoring = true;
  //goalA.accept_invalid_goals = true;
  //goalA.accept_partial_plans = true;
  goalA.operations.collision_operations.resize(1);
  goalA.operations.collision_operations[0] = collisionOperation_1;



  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = x;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = y;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = z;
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = x_quat;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = y_quat;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = z_quat;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = w_quat;
    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;
  

  arm_navigation_msgs::PositionConstraint pc;
  pc.header.stamp = ros::Time::now();
  pc.header.frame_id = "torso_lift_link";    
  pc.link_name = "r_elbow_flex_link";
  pc.position.x = x;
  pc.position.y = y;
  pc.position.z = 2.5;    
  pc.constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_shape.dimensions.push_back(10.0);
  pc.constraint_region_orientation.w = 1.0;
  pc.weight = 1.0;


  goalA.motion_plan_request.goal_constraints.position_constraints.push_back(pc);

  //goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  //goalA.motion_plan_request.goal_constraints.position_constraints[0] = pc;

  if (nh.ok())
  {
    bool finished_within_time = false;
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

 
return true;
}
*/


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


/*  desired_pose.pose.orientation.x = 0.0;
  desired_pose.pose.orientation.y = 0.0;
  desired_pose.pose.orientation.z = 0.0;
  desired_pose.pose.orientation.w = 1.0;*/

  desired_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,M_PI/2);


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
void TestArm2()
{
ROS_INFO("In TestArm2()");

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
  desired_pose.pose.position.z = 0.0;
  desired_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0.0,-M_PI/2,0.0);

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

void TestArm3()
{
ROS_INFO("In TestArm3()");

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
  desired_pose.pose.position.z = 0.0;
  desired_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);

  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.02;

  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].type = 1; //1=header 2=link
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
{
return;
}
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
