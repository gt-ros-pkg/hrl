#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <explore_hrl/ExploreAction.h>
#include <explore_hrl/explore.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <navfn/navfn_ros.h>
#include <explore_hrl/explore_frontier.h>
#include <explore_hrl/loop_closure.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>

using namespace explore_hrl;

namespace explore {

class ExploreAction {
public:
  ExploreAction( std::string name );
  virtual ~ExploreAction();
  void executeCB( const explore_hrl::ExploreGoalConstPtr &goal );

private:
  ros::NodeHandle node_;
  Explore* e_;

  // Action Server additions (follows ExecuteCallbackMethod tutorial):
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<explore_hrl::ExploreAction> as_;
  std::string action_name_;

};

}
