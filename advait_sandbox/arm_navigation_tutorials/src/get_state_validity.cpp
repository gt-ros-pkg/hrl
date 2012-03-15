

#include <ros/ros.h>
#include <planning_environment_msgs/GetStateValidity.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_state_validity_test");
  ros::NodeHandle rh;

  ros::service::waitForService("environment_server_right_arm/get_state_validity");
  ros::ServiceClient check_state_validity_client_ = rh.serviceClient<planning_environment_msgs::GetStateValidity>("environment_server_right_arm/get_state_validity");

  planning_environment_msgs::GetStateValidity::Request req;
  planning_environment_msgs::GetStateValidity::Response res;

  req.robot_state.joint_state.name.push_back("r_shoulder_pan_joint");
  req.robot_state.joint_state.name.push_back("r_shoulder_lift_joint");
  req.robot_state.joint_state.name.push_back("r_upper_arm_roll_joint");
  req.robot_state.joint_state.name.push_back("r_elbow_flex_joint");
  req.robot_state.joint_state.name.push_back("r_forearm_roll_joint");
  req.robot_state.joint_state.name.push_back("r_wrist_flex_joint");
  req.robot_state.joint_state.name.push_back("r_wrist_roll_joint");
  req.robot_state.joint_state.position.resize(7,0.0);

  //these set whatever non-zero joint angle values are desired
  //req.robot_state.joint_state.position[0] = 0.0;
  //req.robot_state.joint_state.position[0] = -.55;
  req.robot_state.joint_state.position[0] = 0.40;
  req.robot_state.joint_state.position[3] = -0.40;
 
  req.robot_state.joint_state.header.stamp = ros::Time::now();
  req.check_collisions = true;
  if(check_state_validity_client_.call(req,res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
      ROS_INFO("Requested state is not in collision");
    else
      ROS_INFO("Requested state is in collision. Error code: %d",res.error_code.val);
  }
  else
  {
    ROS_ERROR("Service call to check state validity failed %s",check_state_validity_client_.getService().c_str());
    return false;
  }
  ros::shutdown();
}






