#define DEBUG_DRAW_TABLE_MARKERS
#define DEBUG_DRAW_TABLETOP_OBJECTS


#define END_EFFECTOR_OFFSET 0.175 //0.193

#include <plane_extractor.h>
#include <move_omni_base.h>
//#include "/u/akansel/gt-ros-pkg/akansel_sandbox/omnix/include/omnix/move_omni_base.h"
#include <book_stacking_msgs/DragRequest.h>
#include <book_stacking_msgs/NavWaypoints.h>
#include <book_stacking_msgs/ObjectInfos.h>
#include <book_stacking_msgs/ObjectInfo.h>

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
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
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
#include <pr2_msgs/PressureState.h>
#include <pr2_msgs/LaserScannerSignal.h>
#include <laser_assembler/AssembleScans.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>


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
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

//VISUALIZATION
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct arm_config_7DOF
{
  double angles[7];
};
struct DragActionTemplate
{
  arm_config_7DOF q_0;
  arm_config_7DOF q_1;
  arm_config_7DOF q_2;  
  double dist;
};

static const std::string COLLISION_PROCESSING_SERVICE_NAME = "/tabletop_collision_map_processing/tabletop_collision_map_processing";
static const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
static const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
static const std::string TORSO_LIFT_LINK_STR = "torso_lift_link";
static const std::string R_WRIST_ROLL_LINK_STR = "r_wrist_roll_link";
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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class book_stacking
{

private:
  ros::NodeHandle root_handle_;
  ros::ServiceClient ik_client_right;
  ros::ServiceClient fk_client_right;
  ros::ServiceClient laser_assembler_client;
  ros::ServiceClient collision_processing_srv;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber r_fingertip_sub_;
  ros::Subscriber l_fingertip_sub_;
	ros::Subscriber tilt_timing_sub_;
  ros::Publisher tilting_pt_cloud_pub_;
  PointHeadClient* point_head_client_;
  TorsoClient *torso_client_;
  TrajClient *traj_right_arm_client_;
  ArmActionClient *move_right_arm_client_;
  ArmActionClient *move_left_arm_client_;
  MoveBaseClient *move_base_client_;
  MoveBaseClient *omnix_move_base_client_;
	PickupClient *pickup_client_;
	GripperClient *right_gripper_client_; 
	GripperClient *left_gripper_client_; 
  tf::TransformListener tf_listener;

  int filter_outliers_meank;
  int min_plane_inliers_;
  int max_sac_iterations_;
  int max_planes_;
  bool filter_outliers; 
  bool downsample_cloud;
  bool concave_hull_mode_;
  bool use_normal_seg_;
  bool use_omp_;
  bool filter_spatial;
  double plane_distance_thresh_;
  double normal_search_radius_;
  double downsample_grid_size_;
  double sac_probability_;
  double filter_outliers_stddev_thresh;
  double filter_spatial_xmin, filter_spatial_xmax;
  double filter_spatial_ymin, filter_spatial_ymax;
  double filter_spatial_zmin, filter_spatial_zmax;
	double table_obj_detector_upper_z,table_obj_detector_lower_z;
	double optimal_workspace_wrt_torso_x, optimal_workspace_wrt_torso_y,  optimal_workspace_wrt_torso_z;
	double optimal_workspace_wrt_torso_x_grasping, optimal_workspace_wrt_torso_y_grasping,  optimal_workspace_wrt_torso_z_grasping;
	double init_torso_position;
	double predrag_dist;	
	double pregrasp_dist;
	double preplace_dist;
	double pregrasp_dist_vertical;	
	double diff_drag_force;

  kinematics_msgs::GetKinematicSolverInfo::Response ik_solver_info;
  kinematics_msgs::GetKinematicSolverInfo::Response fk_solver_info;
  pr2_controllers_msgs::JointTrajectoryGoal rightArmHomingTrajectory;
  std::vector<short int> right_arm_l_finger_tip_nominals;
  std::vector<short int> right_arm_r_finger_tip_nominals;
  std::string workspace_frame;
  std::string base_frame_tf;
  bool left_arm_fingertips_sensing;
  bool right_arm_fingertips_sensing;
  bool right_arm_fingertips_contacted;
  bool right_arm_fingertips_nominals;
  bool right_arm_end_eff_goal_resulted;
  bool robot_initialized;
	bool latest_pt_cloud_ready;
	bool test_arms;
	bool enable_nav;
	bool got_init_table;
	XYZPointCloud latest_cloud;
  double tilt_period;
  double FINGERTIP_CONTACT_THRESHOLD;
	double nav_waypoint_offset;
	pr2_msgs::LaserScannerSignal tilt_minima_msg, tilt_maxima_msg;
  std::ofstream logFile;

  book_stacking_msgs::PlaneInfo init_table_plane_info; //in odom_combined
  pcl::PointCloud<Point> init_table_hull; //in odom_combined
  geometry_msgs::Point32 init_table_centroid; //in odom_combined
  pcl::PointCloud<Point> nav_waypoints; //in odom_combined
	book_stacking_msgs::NavWaypoints nav_waypoint_goals;
	book_stacking_msgs::ObjectInfos all_objects_wrt_odom_combined;
std::vector<double> all_objects_x_wrt_odom_combined;
std::vector<double> all_objects_y_wrt_odom_combined;
std::vector<double> all_objects_z_wrt_odom_combined;
double grasp_offset_from_com;
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

if(test_arms)
{
  TestArm();
	HomeRightArm();
}

robot_initialized=true; 

//move_right_gripper(0.75,-0.188,0,-M_PI/2,0,0,1);
//shakeHead(2);
}

void commandCallback  (const std_msgs::String::ConstPtr& msg)
{
  bool stop_when_contact=true;
  pr2_controllers_msgs::JointTrajectoryGoal output_traj;
  double path_distance,dt;
  double speed=0.15;
  arm_config_7DOF q_solution;
  std::cout<<"Command: "<<msg->data<<std::endl;
  char key0 = msg->data.c_str()[0];   kinematics_msgs::GetKinematicSolverInfo::Response response;
  std::vector<arm_config_7DOF> qs;
  std::vector<double> ts;
  move_base_msgs::MoveBaseGoal goal;
  arm_config_7DOF q_current;
  double odom_x,odom_y,yaw;

switch (key0)
  {
	case 'b': //message is to follower module.

		if(msg->data.length()>2)
		{
			char key2 = msg->data.c_str()[2];
			switch (key2)
			{
                        case 'a':
                        TestArm();
			break;
                        case 'b':
                        HomeRightArm();
			break;
                        case 'c':
                        got_init_table=getInitTable(false);
			break;		
			case 'd':
			plan0();
			break;
			case 'e':
			  HomeRightArm();
			break;
			case 'f':
			  ExploreLinearMoveAction(0.0,0.15,0.0,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'g':
			  ExploreLinearMoveAction(0.0,-0.15,0.0,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'h':
			  ExploreLinearMoveAction(0.15,0.0,0.0,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'i':
			  ExploreLinearMoveAction(-0.15,0.0,0.0,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'j':
			  ExploreLinearMoveAction(0.0,0.0,0.15,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'k':
			  ExploreLinearMoveAction(0.0,0.0,-0.15,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  dt=path_distance/speed;
			  qs.push_back(q_solution);
			  ts.push_back(dt);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,stop_when_contact);
			  break;
			case 'l':
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = 0.25;
			  goal.target_pose.pose.position.y = 0.0;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);				
			  break;
			case 'm':
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = -0.25;
			  goal.target_pose.pose.position.y = 0.0;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);
			  break;
			case 'n':
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = 0.0;
			  goal.target_pose.pose.position.y = 0.25;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);
			  break;
			case 'o':
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = 0.0;
			  goal.target_pose.pose.position.y = -0.25;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);
			  break;
			case 'p':
			  log_IK();
			break;
			case 'r':
			q_current=GetCurrentRightArmJointAngles();
			for(unsigned int i=0;i<7;i++)
			{
			std::cout<<"angles["<<i<<"]: "<<q_current.angles[i]<<std::endl;
			}
			case 's':
			  getOdomPose(odom_x,odom_y,yaw,ros::Time::now());
			  break;
			case 'q':
			OpenRightGripper();
			break;
			case 't':
			CloseRightGripper();
			break;
			case 'u':
			OpenLeftGripper();
			break;
			case 'w':
			CloseLeftGripper();
			break;
			case 'v':
			if(checkIfRightGripperHoldsObject())
			std::cout<<"HAND FULL"<<std::endl;
			else
			std::cout<<"HAND EMPTY"<<std::endl;			
			break;
			case'y':
			GetTiltingPointCloud(true);
			break;
			default:
			  break;
			}
		}
	default:
	  break;
	}
}



void drawHull(pcl::PointCloud<Point> hull_pts,const ros::Publisher& plane_pub, float r, float g, float b)
{
  visualization_msgs::MarkerArray markers;
      visualization_msgs::Marker marker;
      marker.header.frame_id="odom_combined";
      marker.header.stamp=ros::Time::now();
      marker.ns = "nav_waypoints";
      marker.id =5;
      marker.type = visualization_msgs::Marker::LINE_STRIP;//SPHERE_LIST;//LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.5;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.lifetime = ros::Duration(60.0*30.0);

      for(size_t j = 0; j < hull_pts.points.size(); j++)
	{
          geometry_msgs::Point pt;
          pt.x = hull_pts.points[j].x;
          pt.y = hull_pts.points[j].y;
          pt.z = hull_pts.points[j].z;
          marker.points.push_back(pt);
      }

      geometry_msgs::Point pt;
      pt.x = hull_pts.points[0].x;
      pt.y = hull_pts.points[0].y;
      pt.z = hull_pts.points[0].z;
      marker.points.push_back(pt);
      
      markers.markers.push_back(marker);
      plane_pub.publish(markers);
  return;
}

bool getInitTable(bool refreshScan)
{
	ros::Rate loop_rate(10);
	latest_pt_cloud_ready=false;
	GetTiltingPointCloud(refreshScan);
	while (!latest_pt_cloud_ready)
	{		
			ros::spinOnce();
		  loop_rate.sleep();
	}
	if(getTablePlane(latest_cloud,init_table_plane_info,true))
	{
	std::cout<<"frame_id: "<<init_table_plane_info.header.frame_id<<std::endl;
	std::cout<<"hull frame_id: "<<init_table_plane_info.hull.header.frame_id<<std::endl;
	
	XYZPointCloud table_hull_wrt_base_link;
	pcl::fromROSMsg(init_table_plane_info.hull,table_hull_wrt_base_link);
    tf::StampedTransform transf;
    try{
      tf_listener.waitForTransform("odom_combined", init_table_plane_info.header.frame_id,
				  init_table_plane_info.hull.header.stamp, ros::Duration(2.0));
      tf_listener.lookupTransform("odom_combined", init_table_plane_info.header.frame_id,
				  init_table_plane_info.hull.header.stamp, transf);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("getInitTable TF error:%s", ex.what());
      return false;
    }    
    tf::Vector3 v3 = transf.getOrigin();
    tf::Quaternion quat = transf.getRotation();
    Eigen::Quaternionf rot(quat.w(), quat.x(), quat.y(), quat.z());
    Eigen::Vector3f offset(v3.x(), v3.y(), v3.z());
    pcl::transformPointCloud(table_hull_wrt_base_link,init_table_hull,offset,rot);
    init_table_hull.header = table_hull_wrt_base_link.header;
    init_table_hull.header.frame_id = "odom_combined";
   
    init_table_centroid = calcCentroid(init_table_hull);
    std::cout<<"CENTROID | x: "<<init_table_centroid.x<<" y: "<<init_table_centroid.y<< " z: "<<init_table_centroid.z<<std::endl;
	nav_waypoints.clear();
    for(unsigned int i=0;i<init_table_hull.points.size();i++) //calculate Centroid
      {
			geometry_msgs::Point32 p;
			p.x = init_table_hull.points[i].x;
			p.y = init_table_hull.points[i].y;
			p.z = init_table_hull.points[i].z;
			//std::cout<<"P "<<i<<" | x: "<<p.x<<" y: "<<p.y<< " z: "<<p.z<<std::endl;		
			geometry_msgs::Point32 v;
			v.x=p.x-init_table_centroid.x;
			v.y=p.y-init_table_centroid.y;
			double angle=atan2(v.y,v.x);
			Point p_new;
			p_new.x=p.x+nav_waypoint_offset*cos(angle);
			p_new.y=p.y+nav_waypoint_offset*sin(angle);
			p_new.z=0.0;
			nav_waypoints.push_back(p_new);	


	
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id="odom_combined";			  
			goal.target_pose.pose.position.x = p_new.x;
			goal.target_pose.pose.position.y = p_new.y;
			goal.target_pose.pose.position.z = 0.0;
			geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromYaw(atan2(-v.y,-v.x));
			goal.target_pose.pose.orientation = quat;
			goal.target_pose.header.stamp=ros::Time::now();
			nav_waypoint_goals.nav_goals.push_back(goal);


/*
			if(i>0)
			{
			Point p_previous=nav_waypoints[nav_waypoints.size()-1];
			Point p_med;
			p_med.x=(p_new.x+p_previous.x)/2;
			p_med.y=(p_new.y+p_previous.y)/2;
			p_med.z=(p_new.z+p_previous.z)/2;
			nav_waypoints.push_back(p_med);			

			move_base_msgs::MoveBaseGoal goal_mid;
			goal_mid.target_pose.header.frame_id="odom_combined";			  
			goal_mid.target_pose.pose.position.x = p_med.x;
			goal_mid.target_pose.pose.position.y = p_med.y;
			goal_mid.target_pose.pose.position.z = 0.0;
			geometry_msgs::Quaternion quat_mid=tf::createQuaternionMsgFromYaw(atan2(-v.y,-v.x));
			goal_mid.target_pose.pose.orientation = quat_mid;
			goal_mid.target_pose.header.stamp=ros::Time::now();
			nav_waypoint_goals.nav_goals.push_back(goal_mid);
			}
*/
			
			

      }
    	drawHull(nav_waypoints,plane_marker_pub_,0.0,0.0,1.0);

		std::cout<<"NAV_WAYPOInts size: "<<nav_waypoints.size()<<std::endl;
    return true;
	}
	else
	  {
	    return false;
	  }	
}



double calculateHowMuchToPush (book_stacking_msgs::ObjectInfo objInfo)
{
	
  return 0.1;
}

double getEuclidianDist(double x1, double y1, double x2, double y2)
{
double result=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
//std::cout<<"d: "<<result<<std::endl;
return result;
}

void ExecuteNavWaypoints(book_stacking_msgs::NavWaypoints goals)
{
	for(unsigned int i=0;i<goals.nav_goals.size();i++)
	{	std::cout<<"-Goal #"<<i<<std::endl;
		std::cout<<"x: "<<goals.nav_goals[i].target_pose.pose.position.x<<std::endl;
		std::cout<<"y: "<<goals.nav_goals[i].target_pose.pose.position.y<<std::endl;
		OmnixMoveBasePosition(goals.nav_goals[i]);			
	}
}

void TestCollision()
{

	std::cout<<"init_table_hull.points.size(): "<<init_table_hull.points.size()<<std::endl;
	std::cout<<"nav_waypoints.points.size(): "<<nav_waypoints.points.size()<<std::endl;
    Point p_new;
    p_new.x=init_table_centroid.x;
    p_new.y=init_table_centroid.y;
    p_new.z=0.0;

    if(pcl::isPointIn2DPolygon(p_new,init_table_hull))
		{
	  std::cout<<"IN POLYGON"<<std::endl;
		}
		else
		{
		std::cout<<"NOT IN POLYGON"<<std::endl;
		}
return;
}

double DetermineDragDistance(book_stacking_msgs::ObjectInfo objInfo)
{
  if(!got_init_table)
    {
      ROS_WARN("No initial table!");
      return -1.0;
    }
  
  geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
  if(objInfo.header.frame_id.compare("odom_combined") == 0) //already in odom_combined
    {
		//std::cout<<"IN ODOM COMBINED"<<std::endl;

      obj_centroid_wrt_odom_combined.point.x=objInfo.centroid.x;
      obj_centroid_wrt_odom_combined.point.y=objInfo.centroid.y;
      obj_centroid_wrt_odom_combined.point.z=objInfo.centroid.z;
    }
  else //get the object centroid in odom_combined
    {
		//std::cout<<"NOT IN ODOM COMBINED"<<std::endl;
      geometry_msgs::PointStamped input_point; 
      input_point.header.frame_id=objInfo.header.frame_id;
      input_point.point.x=objInfo.centroid.x;
      input_point.point.y=objInfo.centroid.y;
      input_point.point.z=objInfo.centroid.z;
      tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);      
    }

  double odom_x,odom_y,yaw;
  getOdomPose(odom_x,odom_y,yaw,ros::Time::now());

  Point p_robot(odom_x,odom_y,0.0);
  Point p_obj(obj_centroid_wrt_odom_combined.point.x,obj_centroid_wrt_odom_combined.point.y,0.0);
  double d_robot_obj=getEuclidianDist(p_robot.x,p_robot.y,p_obj.x,p_obj.y);  
  double res=0.01;	

  geometry_msgs::Point32 v;
  v.x=p_robot.x-p_obj.x;
  v.y=p_robot.y-p_obj.y;
  //double angle=atan2(v.y,v.x);
  double angle=yaw+M_PI;
	/*
	std::cout<<"p_robot| x: "<<p_robot.x<<" y: "<<p_robot.y<<" z: "<<p_robot.z<<std::endl;
	std::cout<<"p_obj| x: "<<p_obj.x<<" y: "<<p_obj.y<<" z: "<<p_obj.z<<std::endl;
	std::cout<<"v| x: "<<v.x<<" y: "<<v.y<<std::endl;
	std::cout<<"angle: "<<angle<<std::endl;
*/
  for(double d=0.0;d<d_robot_obj;d=d+res) //gradually move towards robot and see where it goes out of the table hull
    {
      Point p_new;
      p_new.x=p_obj.x+d*cos(angle);
      p_new.y=p_obj.y+d*sin(angle);
      p_new.z=0.0;
/*
	std::cout<<"d: "<<d<<std::endl;
	std::cout<<"p_new| x: "<<p_new.x<<" y: "<<p_new.y<<" z: "<<p_new.z<<std::endl;
*/
    if(!pcl::isPointIn2DPolygon(p_new,init_table_hull))
		{
		//std::cout<<"PT OUT"<<std::endl;
	  return d;
		}
		else
		{
		//std::cout<<"PT IN"<<std::endl;
		}

    }

  return -1.0;
}

bool NavigateToObject(book_stacking_msgs::ObjectInfo objInfo)
{
  geometry_msgs::PointStamped input_point;
  geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
  input_point.header.frame_id=objInfo.header.frame_id;
  input_point.point.x=objInfo.centroid.x;
  input_point.point.y=objInfo.centroid.y;
  input_point.point.z=objInfo.centroid.z;
  tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);

	double odom_x,odom_y,yaw;
  getOdomPose(odom_x,odom_y,yaw,ros::Time::now());

	double d_robot_min=999.0;
	double d_object_min=999.0;
	unsigned int min_robot_index=0;
	unsigned int min_object_index=0;

	for(unsigned int i=0;i<nav_waypoint_goals.nav_goals.size();i++) //find q_start and q_end
	{
	double d_robot=getEuclidianDist(nav_waypoint_goals.nav_goals[i].target_pose.pose.position.x, nav_waypoint_goals.nav_goals[i].target_pose.pose.position.y, odom_x,odom_y);
	double d_object=getEuclidianDist(nav_waypoint_goals.nav_goals[i].target_pose.pose.position.x, nav_waypoint_goals.nav_goals[i].target_pose.pose.position.y ,obj_centroid_wrt_odom_combined.point.x, obj_centroid_wrt_odom_combined.point.y);
		if(d_robot<d_robot_min)
		{
		d_robot_min=d_robot;
		min_robot_index=i;
		}
		if(d_object<d_object_min)
		{
		d_object_min=d_object;
		min_object_index=i;
		}
	}

	std::cout<<"Closest_robot| x: "<<nav_waypoint_goals.nav_goals[min_robot_index].target_pose.pose.position.x <<
" y: "<< nav_waypoint_goals.nav_goals[min_robot_index].target_pose.pose.position.y <<std::endl;
  std::cout<<"min_robot_index: "<<min_robot_index<<std::endl;

	std::cout<<"Closest_object| x: "<<nav_waypoint_goals.nav_goals[min_object_index].target_pose.pose.position.x <<
" y: "<< nav_waypoint_goals.nav_goals[min_object_index].target_pose.pose.position.y  <<std::endl;
  std::cout<<"min_object_index: "<<min_object_index<<std::endl;

  if(min_robot_index==min_object_index) //robot doesn't need to move.
    return true;


	book_stacking_msgs::NavWaypoints clockwise_goals;
	book_stacking_msgs::NavWaypoints c_clockwise_goals;

	int waypoint_size=nav_waypoint_goals.nav_goals.size();
	std::cout<<"Waypoint_size: "<<waypoint_size<<std::endl;

bool	clockwise_found=false;
bool	c_clockwise_found=false;
int clockwise_index;
int c_clockwise_index;
	for(int i=0;i<waypoint_size;i++)
	{

		if(!clockwise_found)
		{
				clockwise_index=min_robot_index+i;
				if(clockwise_index>=waypoint_size)
				{
				clockwise_index=clockwise_index-waypoint_size;
				}
				else if(clockwise_index<0)
				{
				clockwise_index=clockwise_index+waypoint_size;
				}
				clockwise_goals.nav_goals.push_back(nav_waypoint_goals.nav_goals[clockwise_index]);	
				if(clockwise_index==min_object_index)
				{
				clockwise_found=true;
				}
		}

		if(!c_clockwise_found)
		{
				c_clockwise_index=min_robot_index-i;
				if(c_clockwise_index>=waypoint_size)
				{
				c_clockwise_index=c_clockwise_index-waypoint_size;
				}
				else if(c_clockwise_index<0)
				{
				c_clockwise_index=c_clockwise_index+waypoint_size;
				}
				c_clockwise_goals.nav_goals.push_back(nav_waypoint_goals.nav_goals[c_clockwise_index]);
				if(c_clockwise_index==min_object_index)
				{
				c_clockwise_found=true;
				}
		}
/*
		std::cout<<"i: "<<i<<std::endl;
		std::cout<<"clockwise_index: "<<clockwise_index<<std::endl;
		std::cout<<"c_clockwise_index: "<<c_clockwise_index<<std::endl;
*/
		if(clockwise_found && c_clockwise_found)
		break;		
	}

std::cout<<"clockwise_goals size: "<<clockwise_goals.nav_goals.size()<<std::endl;
std::cout<<"c_clockwise_goals size: "<<c_clockwise_goals.nav_goals.size()<<std::endl;
	
if(clockwise_goals.nav_goals.size()<c_clockwise_goals.nav_goals.size())
{
ExecuteNavWaypoints(clockwise_goals);
}
else
{
ExecuteNavWaypoints(c_clockwise_goals);
}	
  return true;
}

bool getOdomPose(double &x,double &y,double &yaw, const ros::Time& t)
{
  tf_listener.waitForTransform("odom_combined", "base_link", ros::Time::now(), ros::Duration(2.0));

  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), t, "base_link");
  tf::Stamped<btTransform> odom_pose;
  try
  {
    tf_listener.transformPose("odom_combined", ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  yaw = tf::getYaw(odom_pose.getRotation());
  x=odom_pose.getOrigin().x();
  y=odom_pose.getOrigin().y();
  //std::cout<<"Odom Pose| x:  "<<x<<" y: "<<y<<" yaw: "<<yaw<<std::endl;
    
    return true;
}


unsigned int ScanAndGetTableAndObjects(book_stacking_msgs::PlaneInfo &latest_table_plane_info, book_stacking_msgs::ObjectInfos &allObjectInfos,bool refreshScan)
{
  ros::Rate rate10(10);
  while(true) //get the latest table
    {
			ros::spinOnce();
      latest_pt_cloud_ready=false;
      GetTiltingPointCloud(refreshScan);
      while (!latest_pt_cloud_ready)
			{
				ros::spinOnce();
				rate10.sleep();
			}   
      bool got_current_table=getTablePlane(latest_cloud,latest_table_plane_info,false);
      if(got_current_table)
	break;
    }
  allObjectInfos = getObjectsOverPlane(latest_table_plane_info, latest_cloud, table_obj_detector_lower_z, table_obj_detector_upper_z); //get the latest objects.
#ifdef DEBUG_DRAW_TABLETOP_OBJECTS
  drawObjectPrisms(allObjectInfos,obj_marker_pub_,latest_table_plane_info,0.0f,1.0f,0.0f);
#endif
 ROS_INFO("# OF OBJS: %d",(int)(allObjectInfos.objects.size()));
  return (allObjectInfos.objects.size());
}

void SaveAllObjectsInOdomCombined(book_stacking_msgs::ObjectInfos allObjectInfos)
{
  all_objects_x_wrt_odom_combined.clear();
  all_objects_y_wrt_odom_combined.clear();
	all_objects_z_wrt_odom_combined.clear();
  for(unsigned int i=0; i<allObjectInfos.objects.size();i++) //save objs in odom_combined frame
    {
      int cloud_size=allObjectInfos.objects[i].cloud.data.size();
      std::cout<<"OBJ "<<i<<" : "<<cloud_size<<" pts"<<std::endl;
      
      geometry_msgs::PointStamped input_point;
      geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
      input_point.header.frame_id=allObjectInfos.objects[i].header.frame_id;
      input_point.point.x=allObjectInfos.objects[i].centroid.x;
      input_point.point.y=allObjectInfos.objects[i].centroid.y;
      input_point.point.z=allObjectInfos.objects[i].centroid.z;
      tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);
      all_objects_x_wrt_odom_combined.push_back(obj_centroid_wrt_odom_combined.point.x);
      all_objects_y_wrt_odom_combined.push_back(obj_centroid_wrt_odom_combined.point.y);
			all_objects_z_wrt_odom_combined.push_back(obj_centroid_wrt_odom_combined.point.z);
    }

  return;
}



bool TrackObject(int object_index, book_stacking_msgs::ObjectInfos allObjectInfos, double max_matching_dist)
{
				int min_ind=99;
				double min_dist=999999.9;
				double bestx=0.0;
				double besty=0.0;
				double bestz=0.0;
				for(int j=0;j<allObjectInfos.objects.size();j++) //get the object with minimum distance to previous position
				{
					book_stacking_msgs::ObjectInfo objInfo=allObjectInfos.objects[j];
					geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
					if(objInfo.header.frame_id.compare("odom_combined") == 0) //already in odom_combined
					{
						obj_centroid_wrt_odom_combined.point.x=objInfo.centroid.x;
						obj_centroid_wrt_odom_combined.point.y=objInfo.centroid.y;
						obj_centroid_wrt_odom_combined.point.z=objInfo.centroid.z;
					}
					else //get the object centroid in odom_combined
					{
								geometry_msgs::PointStamped input_point; 
								input_point.header.frame_id=objInfo.header.frame_id;
								input_point.point.x=objInfo.centroid.x;
								input_point.point.y=objInfo.centroid.y;
								input_point.point.z=objInfo.centroid.z;
								tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);      
					}
					double dist=getEuclidianDist(obj_centroid_wrt_odom_combined.point.x, obj_centroid_wrt_odom_combined.point.y, all_objects_x_wrt_odom_combined[object_index], all_objects_y_wrt_odom_combined[object_index]);
					if(dist<min_dist)
					{
					min_dist=dist;
					min_ind=j;
					bestx=obj_centroid_wrt_odom_combined.point.x;
					besty=obj_centroid_wrt_odom_combined.point.y;
					bestz=obj_centroid_wrt_odom_combined.point.z;
					}	
				}
				if(min_dist<max_matching_dist) //if we can track object, break the while loop
				{
				all_objects_x_wrt_odom_combined[object_index]=bestx;
				all_objects_y_wrt_odom_combined[object_index]=besty;
				all_objects_z_wrt_odom_combined[object_index]=bestz;
				return true;
				}
	return false;
}

bool PickUpBook(int object_index)
{
  book_stacking_msgs::PlaneInfo latest_table_plane_info;
  book_stacking_msgs::ObjectInfos allObjectInfos;

	bool	drag_overall_success=false;	
	while(!drag_overall_success)
	{
	book_stacking_msgs::ObjectInfo book_info;
  book_info.header.frame_id="odom_combined";
  book_info.centroid.x=all_objects_x_wrt_odom_combined[object_index];
  book_info.centroid.y=all_objects_y_wrt_odom_combined[object_index];
	book_info.centroid.z=all_objects_z_wrt_odom_combined[object_index];

	if(enable_nav)
	{
  NavigateToObject(book_info); //if can't be reached, navigate to object
	//getInitTable(true);	
	}

  double drag_dist=DetermineDragDistance(book_info);
  std::cout<<"drag_dist: "<<drag_dist<<std::endl;

    if(drag_dist<0.11 && drag_dist>=0.0) //check if we are done dragging.
    {
      std::cout<<"Book is on the edge!"<<std::endl;
			drag_overall_success=true;
    }



	bool partial_success=false;
  if(test_arms && !drag_overall_success)
    {
		double effective_desired_dist;
	
			if(drag_dist>0.055)
			{
			effective_desired_dist=drag_dist-0.055;
			}
			else
			{
			effective_desired_dist=drag_dist-0.02;
			}

      book_stacking_msgs::DragRequest drag_req;
      //drag_req.object=pushedObjectInfo;
      drag_req.object=book_info;
      drag_req.dir.header.frame_id="torso_lift_link";
      drag_req.dir.header.stamp=ros::Time::now();
      drag_req.dir.vector.x=-1.0;
      drag_req.dir.vector.y=0.0;
      drag_req.dir.vector.z=0.0;
      drag_req.dist=effective_desired_dist;
      double output_dist;
      partial_success=DragObject(drag_req,output_dist);
      if(partial_success)
			{
				HomeRightArm();
				//drag_overall_success=true;
			}
		}

		while(!drag_overall_success)
		{
	  unsigned int object_count=ScanAndGetTableAndObjects(latest_table_plane_info, allObjectInfos,true);
			if(object_count > 0)
			{
			double max_tracking_dist;
			if(partial_success)
			max_tracking_dist=0.5;
			else
			max_tracking_dist=0.2;

			if(TrackObject(object_index,allObjectInfos,max_tracking_dist))
			break;
			}
		}
  
	//TODO: update the prediction of the pushed object. Track object  
} //end of while loop - drag_overall_success


	//dragging successful. now grasp! ----
		bool grasp_overall_success=false;	
	while(!grasp_overall_success)
	{

  	if(test_arms)
    {
		double output_dist;
			if(GraspObject(object_index, allObjectInfos,output_dist))
			{
			HomeRightArm();
				if(checkIfRightGripperHoldsObject())
				{			
				grasp_overall_success=true;
				}
			}

			if(!grasp_overall_success)
			{
				while(true)
				{
				unsigned int object_count=ScanAndGetTableAndObjects(latest_table_plane_info, allObjectInfos,true);
					if(object_count > 0)
					{
					double max_tracking_dist=0.22;
					if(TrackObject(object_index,allObjectInfos,max_tracking_dist))
					break;
					}
				}
			}
		} 

		}//end of while !grasp_overall_success loop


//we grasped the object. place it.


  return true;
}



bool plan0()
{
got_init_table=getInitTable(false);
  if(!got_init_table)
    {
      ROS_WARN("No initial table!");
      return false;
    }
  
  book_stacking_msgs::PlaneInfo latest_table_plane_info;
  book_stacking_msgs::ObjectInfos allObjectInfos;
  unsigned int object_count=ScanAndGetTableAndObjects(latest_table_plane_info, allObjectInfos,false);
  if(object_count < 1)
    return false;

  SaveAllObjectsInOdomCombined(allObjectInfos);
	  //int target_object_index=getShortestObjectIndex(allObjectInfos);
  int target_object_index=0;
 
  //book_stacking_msgs::ObjectInfo pushedObjectInfo=allObjectInfos.objects[target_object_index];

  PickUpBook(target_object_index);
	


	double output_dist;
	bool place_overall_success=false;	
	while(!place_overall_success)
	{

		while(true)
		{
		unsigned int object_count= ScanAndGetTableAndObjects(latest_table_plane_info, allObjectInfos, true);
			if(object_count > 0)
			{
			break;
			}
		}
	SaveAllObjectsInOdomCombined(allObjectInfos);
	
	//int host_object_index=0;
	int host_object_index=getTallestObjectIndex(allObjectInfos);
	place_overall_success=PlaceObject(host_object_index,output_dist);
	}


	std::cout<<"PLAN SUCCEEDED!"<<std::endl;
	//GraspObject(target_object_index, allObjectInfos, output_dist);	
	//PlaceObject(target_object_index,output_dist);

  //get a fresh scan, determine drop location
  //navWaypoint to drop location, then move base
  //release gripper
  return false;
	
}

int getTallestObjectIndex(book_stacking_msgs::ObjectInfos allObjectInfos)
{
	double max_height=-999.0;
	int max_height_index=0;
  for(unsigned int i=0; i<allObjectInfos.objects.size();i++)
    {
      geometry_msgs::PointStamped input_point;
      geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
      input_point.header.frame_id=allObjectInfos.objects[i].header.frame_id;
      input_point.point.x=allObjectInfos.objects[i].centroid.x;
      input_point.point.y=allObjectInfos.objects[i].centroid.y;
      input_point.point.z=allObjectInfos.objects[i].centroid.z;
      tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);
			double h=obj_centroid_wrt_odom_combined.point.z;
			if(h>max_height)
			{
			max_height=h;
			max_height_index=i;
			}
	}
	return max_height_index;
}


int getShortestObjectIndex(book_stacking_msgs::ObjectInfos allObjectInfos)
{
	double min_height=999.0;
	int min_height_index=0;
	double diff_with_second=0.0;

  for(unsigned int i=0; i<allObjectInfos.objects.size();i++)
    {
      geometry_msgs::PointStamped input_point;
      geometry_msgs::PointStamped obj_centroid_wrt_odom_combined;
      input_point.header.frame_id=allObjectInfos.objects[i].header.frame_id;
      input_point.point.x=allObjectInfos.objects[i].centroid.x;
      input_point.point.y=allObjectInfos.objects[i].centroid.y;
      input_point.point.z=allObjectInfos.objects[i].centroid.z;
      tf_listener.transformPoint("odom_combined", input_point, obj_centroid_wrt_odom_combined);
			double h=obj_centroid_wrt_odom_combined.point.z;
			if(h<min_height)
			{
			min_height=h;
			min_height_index=i;
			diff_with_second=min_height-h;
			}
	}

	if(diff_with_second<0.01)
	{
	return 0;
	}
	else
	{
	return min_height_index;
	}
}


void HomeRightArm()
{
  traj_right_arm_client_->sendGoal(rightArmHomingTrajectory);
  traj_right_arm_client_->waitForResult();
  return;
}

  bool MoveBasePosition(move_base_msgs::MoveBaseGoal goal)
  {
    ROS_INFO("Sending move_base goal");
    move_base_client_->sendGoal(goal);
    move_base_client_->waitForResult(ros::Duration(3.0));
    if(move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else
      return false;
  }



bool OmnixAdjustBaseForOptimalWorkspaceForGrasping(geometry_msgs::PoseStamped queried_pose_wrt_torso_lift_link)
{
	geometry_msgs::Vector3 optimal_offset;
	optimal_offset.x=-optimal_workspace_wrt_torso_x_grasping+ queried_pose_wrt_torso_lift_link.pose.position.x;
	optimal_offset.y=-optimal_workspace_wrt_torso_y_grasping+queried_pose_wrt_torso_lift_link.pose.position.y;
	optimal_offset.z=-optimal_workspace_wrt_torso_z_grasping+queried_pose_wrt_torso_lift_link.pose.position.z;
	/*
	std::cout<<"optimal_offset.x: "<<optimal_offset.x<<std::endl;
	std::cout<<"optimal_offset.y: "<<optimal_offset.y<<std::endl;
	std::cout<<"optimal_offset.z: "<<optimal_offset.z<<std::endl;
	*/
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id=TORSO_LIFT_LINK_STR;			  
	goal.target_pose.pose.position.x = optimal_offset.x;
	goal.target_pose.pose.position.y = optimal_offset.y;
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;
	goal.target_pose.header.stamp=ros::Time::now();
	OmnixMoveBasePosition(goal);
	return true;
}

bool OmnixAdjustBaseForOptimalWorkspace(geometry_msgs::PoseStamped queried_pose_wrt_torso_lift_link)
{
	geometry_msgs::Vector3 optimal_offset;
	optimal_offset.x=-optimal_workspace_wrt_torso_x+queried_pose_wrt_torso_lift_link.pose.position.x;
	optimal_offset.y=-optimal_workspace_wrt_torso_y+queried_pose_wrt_torso_lift_link.pose.position.y;
	optimal_offset.z=-optimal_workspace_wrt_torso_z+queried_pose_wrt_torso_lift_link.pose.position.z;
	/*
	std::cout<<"optimal_offset.x: "<<optimal_offset.x<<std::endl;
	std::cout<<"optimal_offset.y: "<<optimal_offset.y<<std::endl;
	std::cout<<"optimal_offset.z: "<<optimal_offset.z<<std::endl;
	*/
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id=TORSO_LIFT_LINK_STR;			  
	goal.target_pose.pose.position.x = optimal_offset.x;
	goal.target_pose.pose.position.y = optimal_offset.y;
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;
	goal.target_pose.header.stamp=ros::Time::now();
	OmnixMoveBasePosition(goal);
	return true;
}


bool OmnixMoveBasePosition(move_base_msgs::MoveBaseGoal goal) //goal is in base_link. Converting to /odom_combined
  {
  move_base_msgs::MoveBaseGoal output_goal;  
  geometry_msgs::PoseStamped output_pose_stamped;  
  
	 if (goal.target_pose.header.frame_id.compare("odom_combined") == 0) //already in odom_combined
	{
	omnix_move_base_client_->sendGoal(goal);
	}
	else
	{
    //Transform it to odom_combined
    tf::StampedTransform transf;
    try{
      tf_listener.waitForTransform("odom_combined", goal.target_pose.header.frame_id,
				   goal.target_pose.header.stamp, ros::Duration(2.0));
      tf_listener.lookupTransform("odom_combined", goal.target_pose.header.frame_id,
				  goal.target_pose.header.stamp, transf);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("can't transform to torso_lift_link:%s", ex.what());
      return false;
    }

		tf_listener.transformPose("odom_combined", goal.target_pose, output_pose_stamped);
    output_goal.target_pose=output_pose_stamped;

    //ROS_INFO("Sending omnix_move_base goal");
    omnix_move_base_client_->sendGoal(output_goal);
	}

    omnix_move_base_client_->waitForResult(ros::Duration(20.0));
    if(omnix_move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
			ROS_INFO("Omni move_base Goal Succeeded");
			return true;
			}
    else
		{
			ROS_INFO("Omni move_base Goal Aborted");
      return false;
		}
  }

  void log_IK()
  {
    logFile.open("./result.txt");
    logFile.precision(3);
    arm_config_7DOF q_seed=GetExampleRightArmJointAngles();
    arm_config_7DOF q_solution;
    geometry_msgs::PoseStamped target_pose;
    double x_res=0.04;
    double y_res=0.04;
    double z_res=0.04;
    for(double ix=0.1;ix<0.9;ix=ix+x_res)
      {
		std::cout<<(double)ix<<std::endl;
	for(double iy=-1.0;iy<0.5;iy=iy+y_res)
	  {
	    for(double iz=-0.8;iz<0.5;iz=iz+z_res)
	      {
		geometry_msgs::PoseStamped tpose;
		tpose.header.frame_id=TORSO_LIFT_LINK_STR;
		tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);
		tpose.pose.position.x=ix;
		tpose.pose.position.y=iy;
		tpose.pose.position.z=iz;
		bool ik_result=run_ik(tpose,q_seed,q_solution,R_WRIST_ROLL_LINK_STR,ik_solver_info);
		logFile<<(double)ix<<"\t"<<(double)iy<<"\t"<<(double)iz<<"\t"<<ik_result<<"\n";
	      }	    
	  }
      }
    logFile.close();
    ROS_INFO("Log IK finished");
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

  while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(5.0)) && root_handle_.ok() ) 
  {
    ROS_INFO("Waiting for collision processing service to come up");
  }
  collision_processing_srv = root_handle_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
    (COLLISION_PROCESSING_SERVICE_NAME, true);


 point_cloud_sub_=n_.subscribe("/assembled_pt_cloud2_self_filtered",1,&book_stacking::KinectCallback,this);
 filtered_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_cloud",1);
 plane_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("akans_plane_marker_array",1);
 obj_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obj_markers",1);
 vis_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker",1);
command_subscriber_=n_.subscribe<std_msgs::String>("/command_generator_PR2_topic",1,&book_stacking::commandCallback, this);
r_fingertip_sub_=n_.subscribe("/pressure/r_gripper_motor",1, &book_stacking::FingertipRightCallback,this);
l_fingertip_sub_=n_.subscribe("/pressure/l_gripper_motor",1, &book_stacking::FingertipLeftCallback,this);
tilt_timing_sub_=n_.subscribe("/laser_tilt_controller/laser_scanner_signal",2, &book_stacking::TiltTimingCallback,this);
  traj_right_arm_client_=new TrajClient("r_arm_controller/joint_trajectory_action", true);
 while(!traj_right_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
    while(!torso_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the torso action server to come up");
    }

  point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
  while(!point_head_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }

  move_right_arm_client_ = new ArmActionClient("move_right_arm",true);  
  while(!move_right_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_right_arm server to come up");
    }

  move_left_arm_client_ = new ArmActionClient("move_left_arm",true);
  while(!move_left_arm_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_left_arm server to come up");
    }

 move_base_client_ = new MoveBaseClient("move_base_local",true);  
  while(!move_base_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base server to come up");
    }

 omnix_move_base_client_ = new MoveBaseClient("move_omni_base",true);  
  while(!move_base_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_omni_base server to come up");
    }
pickup_client_=new PickupClient("/object_manipulator/object_manipulator_pickup", true);
  while(!pickup_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the PickupClient server to come up");
    }
right_gripper_client_=new GripperClient("r_gripper_sensor_controller/gripper_action", true);
    while(!right_gripper_client_->waitForServer(ros::Duration(5.0)))
		{
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
left_gripper_client_=new GripperClient("l_gripper_sensor_controller/gripper_action", true);
    while(!left_gripper_client_->waitForServer(ros::Duration(5.0)))
		{
      ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }

    laser_assembler_client = root_handle_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    tilting_pt_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud> ("/assembled_pt_cloud_raw", 5);

 callQueryIK(ik_solver_info);
 callQueryFK(fk_solver_info);

 moveTorsoToPosition(init_torso_position);//0.3
 lookAt("base_link", 1.0, 0.0, 0.0);
 OpenRightGripper();
 
	rightArmHomingTrajectory=createRightArmHomingTrajectory();
	got_init_table=false;
	right_arm_fingertips_sensing=true;
	right_arm_fingertips_nominals=true;
	right_arm_fingertips_contacted=false;
	right_arm_end_eff_goal_resulted=false;
	left_arm_fingertips_sensing=false;
	FINGERTIP_CONTACT_THRESHOLD=150.0;

ROS_INFO("Subs/Pubs Initialized..");
}
 

  pr2_controllers_msgs::JointTrajectoryGoal createRightArmTrajectoryFromAngles(std::vector<arm_config_7DOF> qs, std::vector<double> ts)
{
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    goal.trajectory.points.resize(qs.size());

    for(unsigned int ind=0;ind<qs.size();ind++)
      {
	goal.trajectory.points[ind].positions.resize(7);
	goal.trajectory.points[ind].positions[0] =qs[ind].angles[0];
	goal.trajectory.points[ind].positions[1] =qs[ind].angles[1];
	goal.trajectory.points[ind].positions[2] =qs[ind].angles[2];
	goal.trajectory.points[ind].positions[3] =qs[ind].angles[3];
	goal.trajectory.points[ind].positions[4] =qs[ind].angles[4];
	goal.trajectory.points[ind].positions[5] =qs[ind].angles[5];
	goal.trajectory.points[ind].positions[6] =qs[ind].angles[6];
	goal.trajectory.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j)
	  {
	    goal.trajectory.points[ind].velocities[j] = 0.0;
	  }
	goal.trajectory.points[ind].time_from_start = ros::Duration(ts[ind]);
      }
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

   	goal.trajectory.points[ind].positions[0] = -0.66;
    goal.trajectory.points[ind].positions[1] = -0.173;
    goal.trajectory.points[ind].positions[2] = -0.70;
    goal.trajectory.points[ind].positions[3] = -1.52;
    goal.trajectory.points[ind].positions[4] = 4.64;
    goal.trajectory.points[ind].positions[5] = -1.52;
    goal.trajectory.points[ind].positions[6] = -9.32;

    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);



    // trajectory point 2
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
   	goal.trajectory.points[ind].positions[0] = -1.367;
    goal.trajectory.points[ind].positions[1] = -0.173;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = -1.52;
    goal.trajectory.points[ind].positions[4] = 4.64;
    goal.trajectory.points[ind].positions[5] = -1.52;
    goal.trajectory.points[ind].positions[6] = -9.32;
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    goal.trajectory.points[ind].time_from_start = ros::Duration(5.5);

    return goal;
}


arm_config_7DOF GetExampleRightArmJointAngles()
{
arm_config_7DOF q_current;
int i;
  for(i=0; i<7; i++)
    {
      q_current.angles[i] = (ik_solver_info.kinematic_solver_info.limits[i].min_position + ik_solver_info.kinematic_solver_info.limits[i].max_position)/2.0;
    }
return q_current;
}

arm_config_7DOF GetCurrentRightArmJointAngles()
{
  arm_config_7DOF q_current;
  int i;
  //get a single message from the topic 'r_arm_controller/state'
  pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = 
    ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
    ("r_arm_controller/state");
  


  for(i=0; i<7; i++)
    {
      q_current.angles[i] = state_msg->actual.positions[i];
    }
/*
  for(i=0; i<7; i++)
    {
      q_current.angles[i] = (ik_solver_info.kinematic_solver_info.limits[i].min_position + ik_solver_info.kinematic_solver_info.limits[i].max_position)/2.0;
    }
*/

  return q_current;
}

bool run_fk(arm_config_7DOF q, geometry_msgs::PoseStamped &solution, kinematics_msgs::GetKinematicSolverInfo::Response fk_solver_info)
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
    fk_request.robot_state.joint_state.position[i] = q.angles[i];
  }

  if(fk_client_right.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
     
        /*ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[0].c_str());
        ROS_INFO_STREAM("Position: " << 
          fk_response.pose_stamped[0].pose.position.x << "," <<  
          fk_response.pose_stamped[0].pose.position.y << "," << 
          fk_response.pose_stamped[0].pose.position.z);
        ROS_INFO("Orientation: %f %f %f %f",
          fk_response.pose_stamped[0].pose.orientation.x,
          fk_response.pose_stamped[0].pose.orientation.y,
          fk_response.pose_stamped[0].pose.orientation.z,
          fk_response.pose_stamped[0].pose.orientation.w);
*/
      solution=fk_response.pose_stamped[0];
       
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


  bool run_ik(geometry_msgs::PoseStamped pose, arm_config_7DOF  q_seed,  arm_config_7DOF &q_solution, std::string link_name,kinematics_msgs::GetKinematicSolverInfo::Response ik_solver_info)
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
  ik_request.ik_request.ik_seed_state.joint_state.position[i] = q_seed.angles[i];
  }

    //ROS_INFO("IK request: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    ROS_INFO("IK request: %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
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
        q_solution.angles[i] = ik_response.solution.joint_state.position[i];
        }
      /*ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", 
             q_solution.angles[0],q_solution.angles[1], q_solution.angles[2],q_solution.angles[3],
             q_solution.angles[4],q_solution.angles[5], q_solution.angles[6]);*/

      ROS_INFO("IK YES");
      return true;
    }
   else
    {
      ROS_ERROR("IK NO");
      return false;
    }
return true;
}

bool checkIfRightGripperHoldsObject()
{
	FINGERTIP_CONTACT_THRESHOLD=600;     
	right_arm_fingertips_sensing=true;

	
	ros::Rate loop_rate(5);
	ros::spinOnce();
	loop_rate.sleep();
	ros::spinOnce();
	 		if(right_arm_fingertips_contacted)
			{
			right_arm_fingertips_contacted=false;			
			right_arm_fingertips_sensing=false;		
			FINGERTIP_CONTACT_THRESHOLD=150.0;     
			return true;	
			}
			else
			{
			right_arm_fingertips_sensing=false;
			FINGERTIP_CONTACT_THRESHOLD=150.0;     		
			return false;	
			}
		
}

bool ExploreLinearMoveAction(double dx,double dy,double dz, arm_config_7DOF q_seed, bool move_as_much_you_can, double &path_distance, arm_config_7DOF &q_solution)
{
   /*
    std::cout<<"Current Joint Angles: ";
    for(int i=0;i<7;i++)
    { std::cout<<q_current.angles[i]<<" "; }
    std::cout<<std::endl;
  */
  geometry_msgs::PoseStamped current_pose;
  run_fk(q_seed,current_pose,fk_solver_info);
  geometry_msgs::PoseStamped target_pose=current_pose;
  target_pose.pose.position.x=current_pose.pose.position.x+dx;
  target_pose.pose.position.y=current_pose.pose.position.y+dy;
  target_pose.pose.position.z=current_pose.pose.position.z+dz;
  std::string link_name="r_wrist_roll_link";
  
  bool ik_result=run_ik(target_pose,GetExampleRightArmJointAngles(),q_solution,link_name,ik_solver_info);
  if(!ik_result)
    {
      if(!move_as_much_you_can)
			{
	  return false;
			}
      const int num_inter_checks=5;
      for(int i=num_inter_checks-1;i>0;i--)
			{
			double path_ratio=((double)(i))/num_inter_checks;
			target_pose.pose.position.x=current_pose.pose.position.x+dx*path_ratio;
			target_pose.pose.position.y=current_pose.pose.position.y+dy*path_ratio;
			target_pose.pose.position.z=current_pose.pose.position.z+dz*path_ratio;
			ik_result=run_ik(target_pose,GetExampleRightArmJointAngles(),q_solution,link_name,ik_solver_info);
	    if(ik_result)
	      {
				path_distance=sqrt(dx*dx+dy*dy+dz*dz)*path_ratio;
				break;
	      }
			}
    }
  else
    {
      path_distance=sqrt(dx*dx+dy*dy+dz*dz);
    }

      /*
	std::cout<<"Solution Angles(Linear): ";
	for(int i=0;i<7;i++)
	{ std::cout<<q_solution.angles[i]<<" "; }
	std::cout<<std::endl;
      */

  return ik_result;
}


bool SendRightEndEffectorTrajectory(pr2_controllers_msgs::JointTrajectoryGoal traj, bool stop_when_contact)
{
	
	if(stop_when_contact)
	{

	right_arm_fingertips_sensing=true;
	traj_right_arm_client_->sendGoal(traj);
		while(true)
		{
	 		if(right_arm_fingertips_contacted)
			{
			std::cout<<"R FINGER CONTACT!"<<std::endl;
			right_arm_fingertips_contacted=false;
			traj_right_arm_client_->cancelAllGoals();
			right_arm_fingertips_sensing=false;			
			return false;
			}
			actionlib::SimpleClientGoalState state = traj_right_arm_client_->getState();
			if(state==actionlib::SimpleClientGoalState::SUCCEEDED ||
			state==actionlib::SimpleClientGoalState::ABORTED ||
			state==actionlib::SimpleClientGoalState::REJECTED ||
			state==actionlib::SimpleClientGoalState::LOST)
			{
			right_arm_fingertips_sensing=false;
			return true;
			}
			ros::spinOnce();
		}
	}
	else
	{
        traj_right_arm_client_->sendGoal(traj);
        traj_right_arm_client_->waitForResult();
	actionlib::SimpleClientGoalState state = traj_right_arm_client_->getState();
		if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
		{
		return true;
		}
		else
		{
		return false;
		}
	}
}  


void TiltTimingCallback(const pr2_msgs::LaserScannerSignal::ConstPtr& msg)
{
	if(msg->signal==1) //Tilt at minima
	{
	tilt_minima_msg=*msg;
	}
	else //Tilt at maxima
	{
	tilt_maxima_msg=*msg;
	}
	return;
}

void SendLeftGripperCommand(double pos, double effort) //if -1, don't limit. 50.0 for gentle
{
pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = pos;
    open.command.max_effort = effort;  // Do not limit effort (negative)
left_gripper_client_->sendGoal(open);
    left_gripper_client_->waitForResult();
    if(left_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper command succeeded!");
    else
      ROS_INFO("The gripper failed to open.");
}

void OpenLeftGripper()
{
SendLeftGripperCommand(0.08,-1.0);
}

void CloseLeftGripper()
{
SendLeftGripperCommand(0.0,50.0);
}

void SendRightGripperCommand(double pos, double effort) //if -1, don't limit. 50.0 for gentle
{
pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = pos;
    open.command.max_effort = effort;  // Do not limit effort (negative)
right_gripper_client_->sendGoal(open);
    right_gripper_client_->waitForResult();
    if(right_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper command succeeded!");
    else
      ROS_INFO("The gripper failed to open.");
}

void OpenRightGripper()
{
SendRightGripperCommand(0.08,-1.0);
}

void CloseRightGripper()
{
SendRightGripperCommand(0.0,50.0);
}


void FingertipLeftCallback(const pr2_msgs::PressureState::ConstPtr& msg)
{

if(left_arm_fingertips_sensing)
{
ROS_INFO("L");
//int[] intArray=msg->data.l_finger_tip;
}

}

void FingertipRightCallback(const pr2_msgs::PressureState::ConstPtr& msg)
{
	if(right_arm_fingertips_sensing)
	{
//	ROS_INFO("R");
		if(right_arm_fingertips_nominals)
		{
		right_arm_fingertips_nominals=false;
		right_arm_fingertips_sensing=false;
		right_arm_l_finger_tip_nominals=msg->l_finger_tip;
		right_arm_r_finger_tip_nominals=msg->r_finger_tip;	
		}
		else
		{
		std::vector<short int> l_finger_tip_arr=msg->l_finger_tip;
		std::vector<short int> r_finger_tip_arr=msg->r_finger_tip;
		
		int counter=0;
		for(unsigned int i=0;i<l_finger_tip_arr.size();i++)
		{
		int diff_l=l_finger_tip_arr[i]-right_arm_l_finger_tip_nominals[i];
		int diff_r=r_finger_tip_arr[i]-right_arm_r_finger_tip_nominals[i];

//std::cout<<"diff_l: "<<diff_l<< " diff_r: "<<diff_r<<std::endl;			
			if(diff_r>FINGERTIP_CONTACT_THRESHOLD)
			{
			counter++;
			}	
			if(diff_l>FINGERTIP_CONTACT_THRESHOLD)
			{
			counter++;
			}
		if(counter>2)
		{
			right_arm_fingertips_contacted=true;
		}

		}

		}

	}
}

bool GetTiltingPointCloud(bool refreshScan)
{
ros::Time t_minima=tilt_minima_msg.header.stamp;
ros::Time t_maxima=tilt_maxima_msg.header.stamp;
if(refreshScan)
{
ros::Duration dt_latest_msg;

if(t_minima>t_maxima)
{
dt_latest_msg=ros::Time::now()-t_minima;
}
else
{
dt_latest_msg=ros::Time::now()-t_maxima;
}
ros::Duration wait_sec=ros::Duration(tilt_period)-dt_latest_msg+ros::Duration(0.5);
if(wait_sec.toSec()>ros::Duration(tilt_period).toSec())
{
wait_sec=ros::Duration(tilt_period);
}
std::cout<<"Waiting for "<<wait_sec.toSec()<<" s."<<std::endl;
//wait_sec.sleep();

	ros::Rate loop_rate(10);
	int counter_max=(int)(wait_sec.toSec()*10)+2;
	int counter=0;
	while (counter<counter_max)
	{		
			ros::spinOnce();
		  loop_rate.sleep();
			counter++;
	}
t_minima=tilt_minima_msg.header.stamp;
t_maxima=tilt_maxima_msg.header.stamp;

}

/*
if(&tilt_minima_msg=!NULL && &tilt_minima_msg=!NULL)
{
ROS_WARN("Tilt timing messages not received yet");
return false;
}*/


laser_assembler::AssembleScans srv;
if(t_minima>t_maxima)
{
srv.request.begin=t_maxima;
srv.request.end =t_minima;
}
else
{
srv.request.begin=t_minima;
srv.request.end=t_maxima;
}

 if (laser_assembler_client.call(srv))
   {
     printf("Assembled cloud: %u points\n", srv.response.cloud.points.size());
		 //ros::spinOnce();
     tilting_pt_cloud_pub_.publish(srv.response.cloud);
		 //ros::spinOnce();
   }
 else
   printf("Service call failed\n");

return true;
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
    n_.param("tilt_period", tilt_period, 15.0);

		n_.param("table_obj_detector_lower_z",table_obj_detector_lower_z,-0.2);
		n_.param("table_obj_detector_upper_z",table_obj_detector_upper_z,-0.015);
    n_.param("test_arms",test_arms,true);
    n_.param("enable_nav",enable_nav,true);

		n_.param("optimal_workspace_wrt_torso_x",optimal_workspace_wrt_torso_x,0.6);
		n_.param("optimal_workspace_wrt_torso_y",optimal_workspace_wrt_torso_y,-0.188);
		n_.param("optimal_workspace_wrt_torso_z",optimal_workspace_wrt_torso_z,-0.28);
		n_.param("optimal_workspace_wrt_torso_x_grasping",optimal_workspace_wrt_torso_x_grasping,0.6);
		n_.param("optimal_workspace_wrt_torso_y_grasping",optimal_workspace_wrt_torso_y_grasping, -0.188);
		n_.param("optimal_workspace_wrt_torso_z_grasping",optimal_workspace_wrt_torso_z_grasping , -0.28);
		n_.param("init_torso_position",init_torso_position,0.3);
		n_.param("predrag_dist",predrag_dist,0.07);
		n_.param("pregrasp_dist",pregrasp_dist,0.1);
		n_.param("preplace_dist",preplace_dist,0.1);
		n_.param("pregrasp_dist_vertical",pregrasp_dist_vertical,0.2);
		n_.param("diff_drag_force",diff_drag_force,600.0);
		n_.param("nav_waypoint_offset",nav_waypoint_offset,0.75);
		n_.param("grasp_offset_from_com",grasp_offset_from_com,0.05);


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

bool ExecuteDragAction(geometry_msgs::Vector3Stamped desired_dir,double desired_dist,DragActionTemplate drag_act)
{
	OpenRightGripper();

  double speed=0.12;
  std::vector<arm_config_7DOF> qs;
  std::vector<double> ts;
  qs.push_back(drag_act.q_0);
  ts.push_back(3.0);
  pr2_controllers_msgs::JointTrajectoryGoal traj0=createRightArmTrajectoryFromAngles(qs,ts);
  SendRightEndEffectorTrajectory(traj0,false); //moving to pre-drag

  double path_distance;
  arm_config_7DOF q_1,q_2;

  ExploreLinearMoveAction(0.0,0.0,-(predrag_dist),GetCurrentRightArmJointAngles(),true,path_distance,q_1);
  qs.clear();
  ts.clear();
  ts.push_back(2.0);
  //qs.push_back(q_1);
  qs.push_back(drag_act.q_1);
	pr2_controllers_msgs::JointTrajectoryGoal traj1=createRightArmTrajectoryFromAngles(qs,ts);
      
	FINGERTIP_CONTACT_THRESHOLD=diff_drag_force;     
  SendRightEndEffectorTrajectory(traj1,true); //move down
  FINGERTIP_CONTACT_THRESHOLD=150.0;

/*
				move_base_msgs::MoveBaseGoal goal;
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = desired_dir.vector.x*desired_dist;
			  goal.target_pose.pose.position.y = desired_dir.vector.y*desired_dist;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);		
*/
		

	  qs.clear();
	  ts.clear();
	  ts.push_back(3.0);
	  qs.push_back(drag_act.q_2);
	  pr2_controllers_msgs::JointTrajectoryGoal traj2=createRightArmTrajectoryFromAngles(qs,ts);
	  SendRightEndEffectorTrajectory(traj2,false); //drag

/*
      if(ExploreLinearMoveAction(desired_dir.vector.x*desired_dist ,desired_dir.vector.y*desired_dist ,desired_dir.vector.z*desired_dist,GetCurrentRightArmJointAngles(),true, path_distance,q_2)) //drag
	{
	  qs.clear();
	  ts.clear();
	  ts.push_back(1.0);
	  qs.push_back(q_2);
	  pr2_controllers_msgs::JointTrajectoryGoal traj2=createRightArmTrajectoryFromAngles(qs,ts);
	  SendRightEndEffectorTrajectory(traj2,false);
	}
*/


/*
		 arm_config_7DOF q_3;
     if(ExploreLinearMoveAction(0.0,0.0,0.03,GetCurrentRightArmJointAngles(),true, path_distance,q_3)) //move up
			{
			qs.clear();
			ts.clear();
			ts.push_back(1.0);
			qs.push_back(q_3);
			pr2_controllers_msgs::JointTrajectoryGoal traj3=createRightArmTrajectoryFromAngles(qs,ts);
			SendRightEndEffectorTrajectory(traj3,false);
			}
*/	    
  return true;
}

bool ExploreDragAction(book_stacking_msgs::ObjectInfo objInfo,geometry_msgs::Vector3Stamped desired_dir,double desired_dist, double &output_dist,DragActionTemplate &drag_act, geometry_msgs::PoseStamped &queried_pose_wrt_torso_lift_link) //desired dir is in torso lift link
{
  geometry_msgs::PointStamped input_point;
  geometry_msgs::PointStamped obj_centroid_wrt_torso_lift_link;
  input_point.header.frame_id=objInfo.header.frame_id;
  input_point.point.x=objInfo.centroid.x;
  input_point.point.y=objInfo.centroid.y;
  input_point.point.z=objInfo.centroid.z;
  tf_listener.transformPoint(TORSO_LIFT_LINK_STR, input_point, obj_centroid_wrt_torso_lift_link);
  
  geometry_msgs::PoseStamped tpose;
  tpose.header.frame_id=TORSO_LIFT_LINK_STR;
  tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);
  tpose.pose.position.x=obj_centroid_wrt_torso_lift_link.point.x;
  tpose.pose.position.y=obj_centroid_wrt_torso_lift_link.point.y;
  tpose.pose.position.z=obj_centroid_wrt_torso_lift_link.point.z+END_EFFECTOR_OFFSET;
  
	queried_pose_wrt_torso_lift_link=tpose;

	std::cout<<"q_1 :Drag pose"<<std::endl;
  arm_config_7DOF q_1;
  if (run_ik(tpose,GetExampleRightArmJointAngles(),q_1,R_WRIST_ROLL_LINK_STR,ik_solver_info))
    {
      arm_config_7DOF q_2;
	std::cout<<"q_2 :After Drag pose"<<std::endl;
      ExploreLinearMoveAction(desired_dir.vector.x*desired_dist,desired_dir.vector.y*desired_dist,desired_dir.vector.z*desired_dist,q_1,true,output_dist,q_2);

      tpose.pose.position.z=obj_centroid_wrt_torso_lift_link.point.z+(END_EFFECTOR_OFFSET+predrag_dist);
      arm_config_7DOF q_0;
	std::cout<<"q_0 :Pre-drag pose"<<std::endl;
			if(run_ik(tpose,GetExampleRightArmJointAngles(),q_0,R_WRIST_ROLL_LINK_STR,ik_solver_info))
			{
				drag_act.q_0=q_0;
				drag_act.q_1=q_1;
				drag_act.q_2=q_2;
				drag_act.dist=output_dist;
				return true;
			}
      else
			{
				return false;
			}

    }
  else
    {
      return false;
    }
}

bool ExecutePlaceAction(DragActionTemplate drag_act)
{

  std::vector<arm_config_7DOF> qs;
  std::vector<double> ts;
  qs.push_back(drag_act.q_0);
  ts.push_back(3.0);
  pr2_controllers_msgs::JointTrajectoryGoal traj0=createRightArmTrajectoryFromAngles(qs,ts);
  SendRightEndEffectorTrajectory(traj0,false); //moving to pre-place

  double path_distance;
  arm_config_7DOF q_1,q_2;

/*
  qs.clear();
  ts.clear();
  ts.push_back(3.0);
  qs.push_back(drag_act.q_1);
	pr2_controllers_msgs::JointTrajectoryGoal traj1=createRightArmTrajectoryFromAngles(qs,ts);  
  SendRightEndEffectorTrajectory(traj1,false); 
*/
	OpenRightGripper();

  qs.clear();
  ts.clear();
  ts.push_back(0.5);
  qs.push_back(drag_act.q_2);
	pr2_controllers_msgs::JointTrajectoryGoal traj2=createRightArmTrajectoryFromAngles(qs,ts);  
  SendRightEndEffectorTrajectory(traj2,false); 

				move_base_msgs::MoveBaseGoal goal;
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = -0.1;
			  goal.target_pose.pose.position.y = 0.0;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);		
	
	HomeRightArm();	

  return true;
}

bool ExecuteGraspAction(DragActionTemplate drag_act)
{
OpenRightGripper();

  std::vector<arm_config_7DOF> qs;
  std::vector<double> ts;
  qs.push_back(drag_act.q_0);
  ts.push_back(3.0);
  pr2_controllers_msgs::JointTrajectoryGoal traj0=createRightArmTrajectoryFromAngles(qs,ts);
  SendRightEndEffectorTrajectory(traj0,false); //moving to pre-grasp

  double path_distance;
  arm_config_7DOF q_1,q_2;


  qs.clear();
  ts.clear();
  ts.push_back(3.0);
  qs.push_back(drag_act.q_1);
	pr2_controllers_msgs::JointTrajectoryGoal traj1=createRightArmTrajectoryFromAngles(qs,ts);
      
	FINGERTIP_CONTACT_THRESHOLD=100.0;     
  SendRightEndEffectorTrajectory(traj1,true); //move in +x, towards grasp
  FINGERTIP_CONTACT_THRESHOLD=150.0;



/*
				move_base_msgs::MoveBaseGoal goal;
			  goal.target_pose.header.frame_id="base_link";			  
			  goal.target_pose.pose.position.x = pregrasp_dist-0.02;
			  goal.target_pose.pose.position.y = 0.0;
			  goal.target_pose.pose.position.z = 0.0;
			  goal.target_pose.pose.orientation.x = 0.0;
			  goal.target_pose.pose.orientation.y = 0.0;
			  goal.target_pose.pose.orientation.z = 0.0;
			  goal.target_pose.pose.orientation.w = 1.0;
			  goal.target_pose.header.stamp=ros::Time::now();
			  OmnixMoveBasePosition(goal);		
*/
	
	SendRightGripperCommand(0.02,50.0);
/*	  arm_config_7DOF q_solution;
	pr2_controllers_msgs::JointTrajectoryGoal output_traj;
			  ExploreLinearMoveAction(0.0,0.0,pregrasp_dist_vertical,GetCurrentRightArmJointAngles(),true,path_distance,q_solution);
			  qs.push_back(q_solution);
			  ts.push_back(3.0);
			  output_traj=createRightArmTrajectoryFromAngles(qs,ts);
			  SendRightEndEffectorTrajectory(output_traj,false);//go up!
*/


	  qs.clear();
	  ts.clear();
	  ts.push_back(3.0);
	  qs.push_back(drag_act.q_2);
	  pr2_controllers_msgs::JointTrajectoryGoal traj2=createRightArmTrajectoryFromAngles(qs,ts);
	  SendRightEndEffectorTrajectory(traj2,false); //move up

  return true;
}

bool ExplorePlaceAction(book_stacking_msgs::ObjectInfo objInfo,double &output_dist, DragActionTemplate &grasp_act, geometry_msgs::PoseStamped &queried_pose_wrt_torso_lift_link)
{
  geometry_msgs::PointStamped input_point;
  geometry_msgs::PointStamped obj_centroid_wrt_torso_lift_link;
  input_point.header.frame_id=objInfo.header.frame_id;
  input_point.point.x=objInfo.centroid.x;
  input_point.point.y=objInfo.centroid.y;
  input_point.point.z=objInfo.centroid.z;
  tf_listener.transformPoint(TORSO_LIFT_LINK_STR, input_point, obj_centroid_wrt_torso_lift_link);
  
  geometry_msgs::PoseStamped tpose;
  tpose.header.frame_id=TORSO_LIFT_LINK_STR;
  tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/4,0.0);
  tpose.pose.position.x=obj_centroid_wrt_torso_lift_link.point.x-(END_EFFECTOR_OFFSET+0.06);
  tpose.pose.position.y=obj_centroid_wrt_torso_lift_link.point.y;
  tpose.pose.position.z=obj_centroid_wrt_torso_lift_link.point.z+(0.04);
	queried_pose_wrt_torso_lift_link=tpose;

	std::cout<<"q_1 :Place pose"<<std::endl;
  arm_config_7DOF q_1;
  if (run_ik(tpose,GetExampleRightArmJointAngles(),q_1,R_WRIST_ROLL_LINK_STR,ik_solver_info))
    {
      arm_config_7DOF q_2;
			std::cout<<"q_2 :After Place pose"<<std::endl;
      ExploreLinearMoveAction(-0.10,0.0,0.10,q_1,true,output_dist,q_2);

      tpose.pose.position.z=obj_centroid_wrt_torso_lift_link.point.z+(0.04+preplace_dist);
      arm_config_7DOF q_0;
			std::cout<<"q_0 :Pre-place pose"<<std::endl;
			if(run_ik(tpose,GetExampleRightArmJointAngles(),q_0,R_WRIST_ROLL_LINK_STR,ik_solver_info))
			{
				grasp_act.q_0=q_0;
				grasp_act.q_1=q_1;
				grasp_act.q_2=q_2;
				grasp_act.dist=output_dist;
				return true;
			}
      else
			{
				return false;
			}
    }
  else
    {
      return false;
    }
	
return false;
}

bool ExploreGraspAction(book_stacking_msgs::ObjectInfo objInfo,double &output_dist, DragActionTemplate &grasp_act, geometry_msgs::PoseStamped &queried_pose_wrt_torso_lift_link)
{
  geometry_msgs::PointStamped input_point;
  geometry_msgs::PointStamped obj_centroid_wrt_torso_lift_link;
  input_point.header.frame_id=objInfo.header.frame_id;
  input_point.point.x=objInfo.centroid.x;
  input_point.point.y=objInfo.centroid.y;
  input_point.point.z=objInfo.centroid.z;
  tf_listener.transformPoint(TORSO_LIFT_LINK_STR, input_point, obj_centroid_wrt_torso_lift_link);
  
  geometry_msgs::PoseStamped tpose;
  tpose.header.frame_id=TORSO_LIFT_LINK_STR;
  tpose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0.0,0.0);
  tpose.pose.position.x=obj_centroid_wrt_torso_lift_link.point.x-(END_EFFECTOR_OFFSET+grasp_offset_from_com);
  tpose.pose.position.y=obj_centroid_wrt_torso_lift_link.point.y;
  tpose.pose.position.z=obj_centroid_wrt_torso_lift_link.point.z-0.02;
	queried_pose_wrt_torso_lift_link=tpose;

	std::cout<<"q_1 :Grasp pose"<<std::endl;
  arm_config_7DOF q_1;
  if (run_ik(tpose,GetExampleRightArmJointAngles(),q_1,R_WRIST_ROLL_LINK_STR,ik_solver_info))
    {
      arm_config_7DOF q_2;
			std::cout<<"q_2 :After Grasp pose"<<std::endl;
      ExploreLinearMoveAction(0.0,0.0,pregrasp_dist_vertical,q_1,true,output_dist,q_2);

      tpose.pose.position.x=obj_centroid_wrt_torso_lift_link.point.x-(END_EFFECTOR_OFFSET+ grasp_offset_from_com+pregrasp_dist);
      arm_config_7DOF q_0;
			std::cout<<"q_0 :Pre-drag pose"<<std::endl;
			if(run_ik(tpose,GetExampleRightArmJointAngles(),q_0,R_WRIST_ROLL_LINK_STR,ik_solver_info))
			{
				grasp_act.q_0=q_0;
				grasp_act.q_1=q_1;
				grasp_act.q_2=q_2;
				grasp_act.dist=output_dist;
				return true;
			}
      else
			{
				return false;
			}
    }
  else
    {
      return false;
    }
	
return false;
}
bool PlaceObject(int object_index,double &output_dist)
{

	book_stacking_msgs::ObjectInfo book_info;
  book_info.header.frame_id="odom_combined";
  book_info.centroid.x=all_objects_x_wrt_odom_combined[object_index];
  book_info.centroid.y=all_objects_y_wrt_odom_combined[object_index];
	book_info.centroid.z=all_objects_z_wrt_odom_combined[object_index];

	if(enable_nav)
	{
	NavigateToObject(book_info); //if can't be reached, navigate to object
	}

	//you may wanna get a new scan here.

	DragActionTemplate output_grasp_act={};	
	bool explore_grasp_success=false;
  geometry_msgs::PoseStamped queried_pose_wrt_torso_lift_link;

	explore_grasp_success=ExplorePlaceAction(book_info, output_dist, output_grasp_act, queried_pose_wrt_torso_lift_link);
	
	if(explore_grasp_success)
	  {
	    ROS_INFO("place Action Feasible");
	    if(ExecutePlaceAction(output_grasp_act))
	      {
				ROS_INFO("place Action Executed Successfully");
				return true;
	      }
	    else
	      {
				ROS_INFO("place Action Execution wasn't successful");
				return false;
	      }
	  }
	else
	  {
	    ROS_INFO("place Action Not Feasible. Move the base to optimal location");
		  OmnixAdjustBaseForOptimalWorkspace(queried_pose_wrt_torso_lift_link);
			explore_grasp_success=ExplorePlaceAction(book_info, output_dist, output_grasp_act, queried_pose_wrt_torso_lift_link);
				if(explore_grasp_success)
					{
						ROS_INFO("2.place Action Feasible");
						
						if(ExecutePlaceAction(output_grasp_act))
							{
							ROS_INFO("2. place Action Executed Successfully");
							return true;
							}
						else
							{
							ROS_INFO("2. place Action Execution wasn't successful");
							return false;
							}
					}
				else
					{
						ROS_INFO("2. place Action Not Feasible.");
						return false;
					}
	  }

return true;
}

bool GraspObject(int object_index, book_stacking_msgs::ObjectInfos allObjectInfos, double &output_dist)
{
	
	//first track and get the latest positions.

  book_stacking_msgs::ObjectInfo book_info;
  book_info.header.frame_id="odom_combined";
  book_info.centroid.x=all_objects_x_wrt_odom_combined[object_index];
  book_info.centroid.y=all_objects_y_wrt_odom_combined[object_index];
	book_info.centroid.z=all_objects_z_wrt_odom_combined[object_index];


	DragActionTemplate output_grasp_act={};	
	bool explore_grasp_success=false;
  geometry_msgs::PoseStamped queried_pose_wrt_torso_lift_link;


	explore_grasp_success= ExploreGraspAction(book_info, output_dist, output_grasp_act, queried_pose_wrt_torso_lift_link);
	
	if(explore_grasp_success)
	  {
	    ROS_INFO("Grasp Action Feasible");
	    if(ExecuteGraspAction(output_grasp_act))
	      {
				ROS_INFO("Grasp Action Executed.");
				return true;
	      }
	    else
	      {
				ROS_INFO("Grasp Action Execution wasn't successful");
				return false;
	      }
	  }
	else
	  {
	    ROS_INFO("Grasp Action Not Feasible. Move the base to optimal location");
		  OmnixAdjustBaseForOptimalWorkspaceForGrasping(queried_pose_wrt_torso_lift_link);
			explore_grasp_success=ExploreGraspAction(book_info, output_dist, output_grasp_act, queried_pose_wrt_torso_lift_link);
				if(explore_grasp_success)
					{
						ROS_INFO("2.Grasp Action Feasible");
						
						if(ExecuteGraspAction(output_grasp_act))
							{
							ROS_INFO("2. Grasp Action Executed Successfully");
							return true;
							}
						else
							{
							ROS_INFO("2. Grasp Action Execution wasn't successful");
							return false;
							}
					}
				else
					{
						ROS_INFO("2. Drag Action Not Feasible.");
						return false;
					}

	  }




}

bool DragObject(book_stacking_msgs::DragRequest drag_req, double &output_dist)
{
	DragActionTemplate output_drag_act={};	
	bool explore_drag_success=false;
	geometry_msgs::PoseStamped queried_pose_wrt_torso_lift_link;

	explore_drag_success=ExploreDragAction(drag_req.object, drag_req.dir, drag_req.dist, output_dist, output_drag_act,queried_pose_wrt_torso_lift_link);
	/*if(explore_drag_success)
	  {
	    ROS_INFO("Drag Action Feasible");
	    if(ExecuteDragAction(drag_req.dir,drag_req.dist,output_drag_act))
	      {
				ROS_INFO("Drag Action Executed Successfully");
				return true;
	      }
	    else
	      {
				ROS_INFO("Drag Action Execution wasn't successful");
				return false;
	      }
	  }
	else
	  {
	    ROS_INFO("Drag Action Not Feasible. Move the base to optimal location");
*/
		  OmnixAdjustBaseForOptimalWorkspace(queried_pose_wrt_torso_lift_link);

			explore_drag_success=ExploreDragAction(drag_req.object, drag_req.dir, drag_req.dist, output_dist,output_drag_act,queried_pose_wrt_torso_lift_link);
				if(explore_drag_success)
					{
						ROS_INFO("2.Drag Action Feasible");
						
						if(ExecuteDragAction(drag_req.dir,drag_req.dist,output_drag_act))
							{
							ROS_INFO("2. Drag Action Executed Successfully");
							return true;
							}
						else
							{
							ROS_INFO("2. Drag Action Execution wasn't successful");
							return false;
							}
					}
				else
					{
						ROS_INFO("2. Drag Action Not Feasible.");
						return false;
					}

	  //}

   return false;
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

bool SetMoveArmGoal(double x,double y,double z)
{
  arm_navigation_msgs::MoveArmGoal goalA;
  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 3;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);  
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "torso_lift_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = x;
  desired_pose.pose.position.y = y;
  desired_pose.pose.position.z = z;
	desired_pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,M_PI/2,0.0);
  desired_pose.absolute_position_tolerance.x = 0.02;
  desired_pose.absolute_position_tolerance.y = 0.02;
  desired_pose.absolute_position_tolerance.z = 0.04;
  desired_pose.absolute_roll_tolerance = 0.04;
  desired_pose.absolute_pitch_tolerance = 0.04;
  desired_pose.absolute_yaw_tolerance = 0.04;
  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);
  if (n_.ok())
  {
    bool finished_within_time = false; 
     move_right_arm_client_->sendGoal(goalA);
    finished_within_time =   move_right_arm_client_->waitForResult(ros::Duration(10.0));
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
			return success;
    }
	
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
{/*
if(!robot_initialized )
{
return;
}*/
 ROS_INFO("PT CLOUD");
 XYZPointCloud raw_cloud;
 XYZPointCloud cloud;
 pcl::fromROSMsg(*cloud_msg,raw_cloud);
/*
	std::cout<<"Original frame_id: "<<raw_cloud.header.frame_id<<std::endl;
	std::cout<<"Target frame_id: "<<base_frame_tf<<std::endl;
*/
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

		latest_cloud=cloud;
    //ROS_INFO("Pt cloud transform succeeded");			
		latest_pt_cloud_ready=true;
		return;
}

bool getTablePlane(XYZPointCloud& cloud, book_stacking_msgs::PlaneInfo &pl_info, bool draw_marker)
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
    //ROS_INFO("Publishing filtered cloud...");
    filtered_cloud_pub_.publish(filtered_msg);

    book_stacking_msgs::PlaneInfos plane_infos= getPlanesByNormals(cloud,4,true,concave_hull_mode_,use_omp_,plane_distance_thresh_,max_sac_iterations_,sac_probability_,min_plane_inliers_,normal_search_radius_,0.1);
    plane_infos.header.stamp=cloud.header.stamp; 

    if(draw_marker && plane_infos.planes.size()>0)
    {
    drawPlaneMarkers(plane_infos,plane_marker_pub_,1.0,0.0,0.0);
    }

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
