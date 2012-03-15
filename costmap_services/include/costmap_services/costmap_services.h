#ifndef COSTMAP_SERVICES_COSTMAP_SERVICES_H_
#define COSTMAP_SERVICES_COSTMAP_SERVICES_H_
#include <ros/ros.h>
#include <costmap_services/GetCost.h>  // service type
#include <costmap_services/ScoreTraj.h> // service type
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace costmap_services {
  class CostmapServices {
    public:
      CostmapServices( std::string name );
      ~CostmapServices();

      bool getCost( costmap_services::GetCost::Request  &req,
		    costmap_services::GetCost::Response &res );


      bool scoreTraj( costmap_services::ScoreTraj::Request  &req,
		      costmap_services::ScoreTraj::Response &res );

    private:
      tf::TransformListener tf_;
      costmap_2d::Costmap2DROS costmap_ros_;

      base_local_planner::TrajectoryPlannerROS planner_;
      geometry_msgs::Twist cmd_vel_;
      double controller_frequency_;
      double theta_range_;
      int num_th_samples_, num_x_samples_;

      ros::ServiceServer costmap_srv_;
      ros::ServiceServer scoreTraj_srv_;

      boost::mutex mutex_;
      ros::Publisher pub_;
      ros::Publisher pub_score_;
      ros::Subscriber sub_;
  };
};
#endif
