#include <costmap_services/costmap_services.h>

namespace costmap_services {
  CostmapServices::CostmapServices( std::string name ) : costmap_ros_("costmap", tf_) {
    ros::NodeHandle private_nh( "~/" + name );

    // Initialization.
    private_nh.param("controller_frequency", controller_frequency_, 10.0);
    private_nh.param("num_th_samples", num_th_samples_, 20);
    private_nh.param("num_x_samples", num_x_samples_, 10);
    private_nh.param("theta_range", theta_range_, 0.7);
    planner_.initialize("planner", &tf_, &costmap_ros_);

    // Setup ROS services.
    if (! ros::service::exists("~costmap_getcost", false)){  // Avoid doublely advertizing if multiple instances of this library
	costmap_srv_ = private_nh.advertiseService("costmap_getcost", &CostmapServices::getCost, this );
    }
    if (! ros::service::exists("~costmap_scoretraj", false)){  // Avoid doublely advertizing if multiple instances of this library
	scoreTraj_srv_ = private_nh.advertiseService("costmap_scoretraj", &CostmapServices::scoreTraj, this );
    }
  }


  CostmapServices::~CostmapServices(){
    return;
  }


  bool CostmapServices::getCost( costmap_services::GetCost::Request  &req,
				 costmap_services::GetCost::Response &res ){
    // Get a copy of the current costmap to test. (threadsafe)
    costmap_2d::Costmap2D costmap;
    costmap_ros_.getCostmapCopy( costmap ); 

    // Coordinate transform.
    unsigned int cell_x, cell_y;
    if( !costmap.worldToMap( req.x, req.y, cell_x, cell_y )){
      res.cost = -1.0;
      return false;
    }

    res.cost = double( costmap.getCost( cell_x, cell_y ));
    return true;
  }


  bool CostmapServices::scoreTraj( costmap_services::ScoreTraj::Request  &req,
				   costmap_services::ScoreTraj::Response &res ){
    res.cost = planner_.scoreTrajectory( req.vx, req.vy, req.vtheta, true ); // also updates map.
    return true;
  } 


};


int main(int argc, char** argv){
  ros::init(argc, argv, "costmap_services");
  costmap_services::CostmapServices at( "cs" );
  ros::spin();
  return 0;
}
