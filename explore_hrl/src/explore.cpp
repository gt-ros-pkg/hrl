/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *
 *********************************************************************/

#include <explore_hrl/explore.h>
#include <explore_hrl/explore_frontier.h>
#include <explore_hrl/isDone_srv.h>  // service type

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace costmap_2d;
using namespace navfn;
using namespace visualization_msgs;
using namespace geometry_msgs;
using namespace explore_hrl;

namespace explore {

double sign(double x){
  return x < 0.0 ? -1.0 : 1.0;
}

Explore::Explore() :
  node_(),
  tf_(ros::Duration(10.0)),
  explore_costmap_ros_(NULL),
  move_base_client_("move_base"),
  planner_(NULL),
  done_exploring_(false),
  preempt_(false),
  explorer_(NULL),
  goal_pose_last_(),
  goal_pose_last_defined_(false)
{
  ros::NodeHandle private_nh("~");

  // Travis hack -- to replace with actionlib later.
  if (! ros::service::exists("~explore_done", false)){  
	isDone_srv_ = private_nh.advertiseService("explore_done", &Explore::isDone, this );
  }

  marker_publisher_ = node_.advertise<Marker>("visualization_marker",10);
  marker_array_publisher_ = node_.advertise<MarkerArray>("visualization_marker_array",10);
  map_publisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  map_server_ = private_nh.advertiseService("explore_map", &Explore::mapCallback, this);

  private_nh.param("navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("graph_update_frequency", graph_update_frequency_, 1.0);
  private_nh.param("visualize", visualize_, 1);
  double loop_closure_addition_dist_min;
  double loop_closure_loop_dist_min;
  double loop_closure_loop_dist_max;
  double loop_closure_slam_entropy_max;
  private_nh.param("loop_closure_addition_dist_min", loop_closure_addition_dist_min, 2.5);
  private_nh.param("loop_closure_loop_dist_min", loop_closure_loop_dist_min, 6.0);
  private_nh.param("loop_closure_loop_dist_max", loop_closure_loop_dist_max, 20.0);
  private_nh.param("loop_closure_slam_entropy_max", loop_closure_slam_entropy_max, 3.0);
  private_nh.param("potential_scale", potential_scale_, 1e-3);
  private_nh.param("orientation_scale", orientation_scale_, .318);
  private_nh.param("gain_scale", gain_scale_, 1.0);

  explore_costmap_ros_ = new Costmap2DROS(std::string("explore_costmap"), tf_);
  explore_costmap_ros_->clearRobotFootprint();

  planner_ = new navfn::NavfnROS(std::string("explore_planner"), explore_costmap_ros_);
  explorer_ = new ExploreFrontier();
  loop_closure_ = new LoopClosure(loop_closure_addition_dist_min, 
                                  loop_closure_loop_dist_min,
                                  loop_closure_loop_dist_max,
                                  loop_closure_slam_entropy_max,
                                  graph_update_frequency_,
                                  move_base_client_,
                                  *explore_costmap_ros_,
                                  client_mutex_,
                                  boost::bind(&Explore::makePlan, this));
  ROS_INFO( "Done making Explore" );
}

Explore::~Explore() {

  // Reset markers (if applicable)
  if (visualize_ and explorer_ != NULL) {
    ROS_INFO("explore: Destroy old markers");
    std::vector<Marker> markers;
    explorer_->getVisualizationMarkers(markers);
    visualization_msgs::MarkerArray marker_array;
    marker_array.set_markers_size(markers.size());
    for (unsigned int i=0; i < markers.size(); i++){
      marker_array.markers[i] = markers[i];
      marker_array.markers[i].action = Marker::DELETE;
    }
    marker_array_publisher_.publish(marker_array);
    ROS_INFO("explore: Done.");
  }

  if(loop_closure_ != NULL){
    ROS_INFO("explore: Action Destroying old explore loop_closure");
    delete loop_closure_;
    ROS_INFO("Done.");
  }

  if(planner_ != NULL) {
    ROS_INFO("explore: Action Destroying old explore planner");
    delete planner_;
    ROS_INFO("Done.");
  }

  if(explorer_ != NULL) {
    ROS_INFO("explore: Action Destroying old explore explorer");
    delete explorer_;
    ROS_INFO("Done.");
  }

  if(explore_costmap_ros_ != NULL){
    ROS_INFO("explore: Action Destroying old explore costmap (WARN: New one might not have probs)");
    delete explore_costmap_ros_;
    ROS_INFO("Done.");
  }
}

bool Explore::isDone( explore_hrl::isDone_srv::Request &req, explore_hrl::isDone_srv::Response &res ){
  res.status = done_exploring_;
  return true;
}


bool Explore::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  ROS_DEBUG("mapCallback");
  Costmap2D explore_costmap;
  explore_costmap_ros_->getCostmapCopy(explore_costmap);

  res.map.info.width = explore_costmap.getSizeInCellsX();
  res.map.info.height = explore_costmap.getSizeInCellsY();
  res.map.info.resolution = explore_costmap.getResolution();
  res.map.info.origin.position.x = explore_costmap.getOriginX();
  res.map.info.origin.position.y = explore_costmap.getOriginY();
  res.map.info.origin.position.z = 0;
  res.map.info.origin.orientation.x = 0;
  res.map.info.origin.orientation.y = 0;
  res.map.info.origin.orientation.z = 0;
  res.map.info.origin.orientation.w = 1;

  int size = res.map.info.width * res.map.info.height;
  const unsigned char* map = explore_costmap.getCharMap();

  res.map.set_data_size(size);
  for (int i=0; i<size; i++) {
    if (map[i] == NO_INFORMATION)
      res.map.data[i] = -1;
    else if (map[i] == LETHAL_OBSTACLE)
      res.map.data[i] = 100;
    else
      res.map.data[i] = 0;
    // if (map[i] == LETHAL_OBSTACLE )
    //   res.map.data[i] = 100;
    // else if (map[i] == FREE_SPACE)
    //   res.map.data[i] = 0;
    // else
    //   res.map.data[i] = -1;
  }

  return true;
}

void Explore::publishMap() {
  nav_msgs::OccupancyGrid map;
  map.header.stamp = ros::Time::now();

  Costmap2D explore_costmap;
  explore_costmap_ros_->getCostmapCopy(explore_costmap);

  map.info.width = explore_costmap.getSizeInCellsX();
  map.info.height = explore_costmap.getSizeInCellsY();
  map.info.resolution = explore_costmap.getResolution();
  map.info.origin.position.x = explore_costmap.getOriginX();
  map.info.origin.position.y = explore_costmap.getOriginY();
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;

  int size = map.info.width * map.info.height;
  const unsigned char* char_map = explore_costmap.getCharMap();

  map.set_data_size(size);
  for (int i=0; i<size; i++) {
    if (char_map[i] == NO_INFORMATION)
      map.data[i] = -1;
    else if (char_map[i] == LETHAL_OBSTACLE)
      map.data[i] = 100;
    else
      map.data[i] = 0;
  }
  // for (int i=0; i<size; i++) {
  //   if (char_map[i] == NO_INFORMATION)
  //     map.data[i] = -1;
  //   else if (char_map[i] == LETHAL_OBSTACLE)
  //     map.data[i] = 100;
  //   else
  //     map.data[i] = 0;
  // }

  map_publisher_.publish(map);
}

void Explore::publishGoal(const geometry_msgs::Pose& goal){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "explore_goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.pose = goal;
  marker.scale.x = 0.5;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(5);
  marker_publisher_.publish(marker);
}

void Explore::getRobotPose(std::string frame, tf::Stamped<tf::Pose>& pose){
  tf::Stamped<tf::Pose> robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();

  try{
    tf_.transformPose(frame, robot_pose, pose);
  }
  catch(tf::LookupException& ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return;
  }
  catch(tf::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return;
  }
  catch(tf::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
  }
}


void Explore::makePlan() {
  //since this gets called on handle activate
  if(explore_costmap_ros_ == NULL)
    return;

  //check to see if we have control of the action client
  //if(client_mutex_.try_lock()){
  if (true){
    tf::Stamped<tf::Pose> robot_pose;
    getRobotPose(explore_costmap_ros_->getGlobalFrameID(), robot_pose);
    geometry_msgs::Point robot_point;
    robot_point.x = robot_pose.getOrigin().x();
    robot_point.y = robot_pose.getOrigin().y();
    robot_point.z = robot_pose.getOrigin().z();

    std::vector<geometry_msgs::Pose> goals;
    explore_costmap_ros_->clearRobotFootprint();
    explorer_->getExplorationGoals(*explore_costmap_ros_, robot_point, planner_, goals, potential_scale_, orientation_scale_, gain_scale_);

    bool valid_plan = false;
    std::vector<geometry_msgs::PoseStamped> plan;
    PoseStamped goal_pose, robot_pose_msg;
    tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);

    bool found_exploration_goal = false;
    goal_pose.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
    goal_pose.header.stamp = ros::Time::now();
    int blacklist_count = 0;
    for (unsigned int i=0; i<goals.size(); i++) {
      goal_pose.pose = goals[i];

      valid_plan = ((planner_->makePlan(robot_pose_msg, goal_pose, plan)) &&
          (!plan.empty()));

      if (valid_plan) {
        if(!goalOnBlacklist(goal_pose)){
          found_exploration_goal = true;
          break;
        }
        else{
          //just so we can say how many goals are too close to a blacklisted one
          blacklist_count++;
        }
      }
    }

    done_exploring_ = !found_exploration_goal;

    // publish visualization markers
    if (visualize_) {
      std::vector<Marker> markers;
      explorer_->getVisualizationMarkers(markers);
      visualization_msgs::MarkerArray marker_array;
      marker_array.set_markers_size(markers.size());
      for (unsigned int i=0; i < markers.size(); i++){
        marker_array.markers[i] = markers[i];
      }
      marker_array_publisher_.publish(marker_array);
    }

    if (valid_plan) {
      //explore_costmap_ros_->clearRobotFootprint(); // help prevent phantom obstacles, especially at startup
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = goal_pose;

      if (isRepeatGoal( goal.target_pose )){
	ROS_INFO("Explore: This appears to be a duplicate goal.");
	frontier_blacklist_.push_back( goal.target_pose );
	if (!done_exploring_){
	  makePlan();
	}
      }
      else {
	move_base_client_.sendGoal(goal, boost::bind(&Explore::reachedGoal, this, _1, _2, goal_pose));
      }

      if (visualize_) {
        publishGoal(goal_pose.pose);
        publishMap();
      }
    } else {
      ROS_WARN("Done exploring with %d goals left that could not be reached. There are %d goals on our blacklist, and %d of the frontier goals are too close to them to pursue. The rest had global planning fail to them. \n", (int)goals.size(), (int)frontier_blacklist_.size(), blacklist_count);
      ROS_INFO("We finsihed exploring the map. Hooray.");
    }
    

    if (visualize_) {
        publishMap();
    }
    //make sure to unlock the mutex when we're done
    client_mutex_.unlock();
  }
  else {
    ROS_WARN("Explore: Mutex acquire failed!");
  }
}

bool Explore::isRepeatGoal(const geometry_msgs::PoseStamped& goal){
  // Prevent duplicate goal selections.  This tends to happen at
  // startup if there is unknown space that cannot be cleared by LRF
  // on PR2.

  // Skip over the first time this is called.
  if (!goal_pose_last_defined_){
    goal_pose_last_defined_ = true;
    goal_pose_last_ = goal;
    return false;
  }

  double x_diff = fabs(goal.pose.position.x - goal_pose_last_.pose.position.x);
  double y_diff = fabs(goal.pose.position.y - goal_pose_last_.pose.position.y);
  double ang_diff = tf::getYaw( goal.pose.orientation ) - tf::getYaw( goal_pose_last_.pose.orientation );

  // Update last pose.
  goal_pose_last_ = goal;

  while (ang_diff < -3.14159) ang_diff += 2.0 * 3.14159;
  while (ang_diff > 3.14159) ang_diff -= 2.0 * 3.14159;

  ROS_INFO( "Differences: %3.2f %3.2f %3.2f", x_diff, y_diff, ang_diff );

  if (x_diff < 2 * explore_costmap_ros_->getResolution() && 
      y_diff < 2 * explore_costmap_ros_->getResolution() &&
      fabs( ang_diff ) < 5.0 * (3.14158 / 180.0)){
    return true;
  }
  else {
    return false;
  }
}

bool Explore::goalOnBlacklist(const geometry_msgs::PoseStamped& goal){
  //check if a goal is on the blacklist for goals that we're pursuing
  for(unsigned int i = 0; i < frontier_blacklist_.size(); ++i){
    double x_diff = fabs(goal.pose.position.x - frontier_blacklist_[i].pose.position.x);
    double y_diff = fabs(goal.pose.position.y - frontier_blacklist_[i].pose.position.y);

    if(x_diff < 2 * explore_costmap_ros_->getResolution() && y_diff < 2 * explore_costmap_ros_->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status, 
    const move_base_msgs::MoveBaseResultConstPtr& result, geometry_msgs::PoseStamped frontier_goal){
  if(status == actionlib::SimpleClientGoalState::ABORTED){
    frontier_blacklist_.push_back(frontier_goal);
    ROS_ERROR("Adding current goal to blacklist");
  }

  // if(!done_exploring_){
  //   //create a plan from the frontiers left and send a new goal to move_base
  //   makePlan();
  // }
  // else{
  //   ROS_INFO("We finsihed exploring the map. Hooray.");
  // }
}

void Explore::execute() {
  while (! move_base_client_.waitForServer(ros::Duration(5,0)))
    ROS_WARN("Waiting to connect to move_base server");

  ROS_INFO("Connected to move_base server");
  goal_pose_last_defined_ = false;

  // This call sends the first goal, and sets up for future callbacks.
  ROS_INFO("explore: ONE");
  makePlan();
  ROS_INFO("explore: TWO");

  //ros::Rate r(graph_update_frequency_);
  ros::Rate r(5.0);
  ros::Time time_last_moving = ros::Time::now();
  tf::Stamped<tf::Pose> robot_pose;
  geometry_msgs::PoseStamped new_pose, ref_pose;
  
  getRobotPose(explore_costmap_ros_->getGlobalFrameID(), robot_pose);
  ROS_INFO("explore: THREE");

  // I don't know how to use bullet tf...
  tf::poseStampedTFToMsg( robot_pose, new_pose );
  ref_pose = new_pose;
  ROS_INFO("explore: FOUR");

  while (node_.ok() && (!done_exploring_) && (!preempt_)) {
    // Get the current pose and pass it to loop closure
    getRobotPose(explore_costmap_ros_->getGlobalFrameID(), robot_pose);
    tf::poseStampedTFToMsg( robot_pose, new_pose );
    loop_closure_->updateGraph(robot_pose);
    //ROS_INFO("explore: FIVE");
    
    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE){
      //ROS_INFO("explore: SIX");
      float dx,dy,da;
      // Check to see if we've moved...
      dx = new_pose.pose.position.x - ref_pose.pose.position.x;
      dy = new_pose.pose.position.y - ref_pose.pose.position.y;
      da = tf::getYaw(new_pose.pose.orientation) - tf::getYaw(ref_pose.pose.orientation);
      if (dx*dx+dy*dy > 0.02 or da*da > 5 * 3.14159 / 180.0){ // Yep, so reset time and new reference position
	time_last_moving = ros::Time::now();
	ref_pose = new_pose;
      }

      // If we haven't moved in 5 seconds, ditch this goal!
      if (ros::Time::now() - time_last_moving > ros::Duration( 5 )){
	ROS_INFO( "We appear to not be moving... aborting goal." );
	move_base_client_.cancelAllGoals();
      }
    }
    else{ // Time to send a new frontier.
      makePlan();
      ref_pose = new_pose;
      time_last_moving = ros::Time::now();
    }
    r.sleep();
  }
  move_base_client_.cancelAllGoals();
}

void Explore::spin() {
  ROS_INFO("Entering Explore:spin()");
  ros::spinOnce();
  boost::thread t(boost::bind( &Explore::execute, this ));
  ros::spin();
  t.join();
}

void Explore::setPreemptFlag( bool state ) {
  ROS_INFO("Explore preempt state set to %d.", state);
  preempt_ = state;
}

bool Explore::doneExploring( ) {
  return done_exploring_;
}

}

int main(int argc, char** argv){
  ros::init(argc, argv, "explore");

  explore::Explore explore;
  explore.spin();

  return(0);
}

