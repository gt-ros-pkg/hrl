#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include "sensor_msgs/PointCloud2.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <hrl_table_detect/GetTableApproaches.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <std_srvs/Empty.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using namespace sensor_msgs;
using namespace std;
namespace hrl_table_detect {
    
    class TableApproaches {
        public:
            ros::NodeHandle nh;
            ros::ServiceServer table_appr_srv;
            tf::TransformListener tf_listener;
            costmap_2d::Costmap2DROS* costmap_ros;

            void onInit();
            bool tableApprCallback(hrl_table_detect::GetTableApproaches::Request& req, 
                                   hrl_table_detect::GetTableApproaches::Response& resp);
    };

    void TableApproaches::onInit() {
        table_appr_srv = nh.advertiseService("/detect_table_approaches", &TableApproaches::tableApprCallback, this);
        costmap_ros = new costmap_2d::Costmap2DROS("table_costmap", tf_listener);
        costmap_ros->start();
        ros::Duration(1.0).sleep();
    }

    bool TableApproaches::tableApprCallback(hrl_table_detect::GetTableApproaches::Request& req,
                                   hrl_table_detect::GetTableApproaches::Response& resp) {
        //std::vector<geometry_msgs::Point> table_poly = req.table_hull.points;
        costmap_2d::Costmap2D costmap;
        costmap_ros->getCostmapCopy(costmap);
        base_local_planner::CostmapModel* world_model
                           = new base_local_planner::CostmapModel(costmap);
        geometry_msgs::Point curpt;
        curpt.x = 0; curpt.y = 0;
        double foot_cost = world_model->footprintCost(curpt, 
                                      costmap_ros->getRobotFootprint(), 
                                      costmap_ros->getInscribedRadius(), 
                                      costmap_ros->getCircumscribedRadius());
        cout << foot_cost << endl;
        return true;
    }
};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_segmentation");
    TableApproaches ta;
    ta.onInit();
    ros::spin();
    return 0;
}



