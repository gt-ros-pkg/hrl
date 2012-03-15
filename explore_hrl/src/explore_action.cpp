#include <explore_hrl/explore_action.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace explore_hrl;

namespace explore {

ExploreAction::ExploreAction( std::string name ) :
  node_(),
  e_( NULL ),
  action_name_( name ),
  as_( nh_, name, boost::bind( &ExploreAction::executeCB, this, _1 ), false )
{
  ros::NodeHandle private_nh("~");
  //e_ = new Explore();
  as_.start();
}

ExploreAction::~ExploreAction() {
  if(e_ != NULL)
    delete e_;
}

void ExploreAction::executeCB( const explore_hrl::ExploreGoalConstPtr &goal ){
  ROS_INFO( "%s: Action Initiated with radius %3.2f", action_name_.c_str(), goal->radius );
  bool success = true;

  if(e_ != NULL)
    ROS_INFO("%s: Action Destroying old explore", action_name_.c_str());
    delete e_;

  ROS_INFO( "%s: Action modifying parameters", action_name_.c_str(), goal->radius );

  ros::NodeHandle private_nh("~");
  private_nh.setParam("explore_costmap/raytrace_range", goal->radius + 0.1 );
  private_nh.setParam("explore_costmap/obstacle_range", goal->radius );

  // Give everything a second to settle before reinitializing.
  ros::Duration(1.0).sleep();

  ROS_INFO("%s: Action Creating new explore", action_name_.c_str());
  e_ = new Explore();

  ros::spinOnce();
  e_ -> setPreemptFlag( false );
  boost::thread t(boost::bind( &Explore::execute, e_ ));

  ros::Rate r(10.0);
  while (node_.ok() && (!e_->doneExploring())) {
    if (as_.isPreemptRequested() || !ros::ok()){
        ROS_INFO("%s: Action Preempted", action_name_.c_str());
        as_.setPreempted();
	e_ -> setPreemptFlag( true );
        success = false;
        break;
    }
    r.sleep();
  }

  if (success){
    ROS_INFO("%s: Action Succeeded", action_name_.c_str());
    as_.setSucceeded();
  }

  t.join();

  ROS_INFO("%s: Action Exiting", action_name_.c_str());
  //ros::spin();
  //e_ -> spin();
  //as_.setSucceeded();
}

}

int main(int argc, char** argv){
  ros::init(argc, argv, "explore");

  explore::ExploreAction explore_action( ros::this_node::getName() );
  ros::spin();

  return(0);
}

