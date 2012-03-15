//
// This file has been copied from the move_arm tutorial:
// http://www.ros.org/wiki/move_arm/Tutorials/MoveArmPoseGoalComplex
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  nh.param<std::string>("planner_id",goalA.motion_plan_request.planner_id,std::string(""));
  nh.param<std::string>("planner_service_name",goalA.planner_service_name,std::string("ompl_planning/plan_kinematic_path"));
  goalA.motion_plan_request.goal_constraints.set_position_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = 0.75;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = -0.188;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = 0;
    
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = geometric_shapes_msgs::Shape::BOX;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

  goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;

  goalA.motion_plan_request.goal_constraints.set_orientation_constraints_size(1);
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = 0.0;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = 1.0;
    
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

  goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;

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
  ros::shutdown();
}





