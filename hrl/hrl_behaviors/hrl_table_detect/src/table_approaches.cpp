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
#include <base_local_planner/trajectory_planner_ros.h>
#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>

#define DIST(x1,y1,x2,y2) (std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)))

using namespace sensor_msgs;
using namespace std;
namespace hrl_table_detect {
    
    class TableApproaches {
        public:
            ros::NodeHandle nh;
            ros::ServiceServer table_appr_srv;
            ros::Publisher valid_pub, close_pub, invalid_pub, approach_pub;
            tf::TransformListener tf_listener;
            costmap_2d::Costmap2DROS* costmap_ros;
            base_local_planner::TrajectoryPlannerROS traj_planner;
            vector<geometry_msgs::Point> footprint_model;
            base_local_planner::CostmapModel* world_model;
            costmap_2d::Costmap2D costmap;

            TableApproaches();
            ~TableApproaches();
            void onInit();
            bool tableApprCallback(hrl_table_detect::GetTableApproaches::Request& req, 
                                   hrl_table_detect::GetTableApproaches::Response& resp);
            double footprintCost(const Eigen::Vector3f& pos, double scale);
    };

    TableApproaches::TableApproaches()  {
    }

    TableApproaches::~TableApproaches()  {
        delete costmap_ros;
    }

    void TableApproaches::onInit() {
        table_appr_srv = nh.advertiseService("/table_detection/detect_table_approaches", &TableApproaches::tableApprCallback, this);
        costmap_ros = new costmap_2d::Costmap2DROS("table_costmap", tf_listener);
        traj_planner.initialize("table_traj_planner", &tf_listener, costmap_ros);
        footprint_model = costmap_ros->getRobotFootprint();
        valid_pub = nh.advertise<geometry_msgs::PoseArray>("valid_poses", 1);
        invalid_pub = nh.advertise<geometry_msgs::PoseArray>("invalid_poses", 1);
        approach_pub = nh.advertise<geometry_msgs::PoseArray>("approach_poses", 1);
        close_pub = nh.advertise<geometry_msgs::PoseArray>("close_poses", 1);
        
        //costmap_ros->start();
        ros::Duration(1.0).sleep();
    }

    bool pose_dist_comp(geometry_msgs::Pose& p1, geometry_msgs::Pose& p2, geometry_msgs::Point& app_pt) {
        return DIST(p1.position.x, p1.position.y, app_pt.x, app_pt.y) < DIST(p2.position.x, p2.position.y, app_pt.x, app_pt.y);
    }

    bool pose_dist_thresh(geometry_msgs::Pose pose, geometry_msgs::Point pt, double thresh) {
        ROS_INFO("dist: %f, thresh: %f, val %d", DIST(pose.position.x, pose.position.y, pt.x, pt.y), thresh, DIST(pose.position.x, pose.position.y, pt.x, pt.y) < thresh);
        return DIST(pose.position.x, pose.position.y, pt.x, pt.y) < thresh;
    }

    bool TableApproaches::tableApprCallback(hrl_table_detect::GetTableApproaches::Request& req,
                                   hrl_table_detect::GetTableApproaches::Response& resp) {
        double pose_step = 0.01, start_dist = 0.25, max_dist = 1.2, min_cost = 250;
        double close_thresh = 0.10;


        // TODO should this be transformed?
        std::vector<geometry_msgs::Point> table_poly = req.table.points;
        geometry_msgs::PointStamped approach_pt = req.approach_pt;
        /*
        double xsize = 1.0, ysize = 1.0, xoff = 2.5, yoff = 0.0;
        geometry_msgs::Point pt; 
        pt.x = xoff-xsize/2; pt.y = yoff-ysize/2;
        table_poly.push_back(pt);
        pt.x = xoff+xsize/2; pt.y = yoff-ysize/2;
        table_poly.push_back(pt);
        pt.x = xoff+xsize/2; pt.y = yoff+ysize/2;
        table_poly.push_back(pt);
        pt.x = xoff-xsize/2; pt.y = yoff+ysize/2;
        table_poly.push_back(pt);
        geometry_msgs::PointStamped approach_pt;
        approach_pt.header.frame_id = "/base_link";
        approach_pt.header.stamp = ros::Time::now();
        approach_pt.point.x = 2.2; approach_pt.point.y = 0.3; 
        */

        costmap_ros->getCostmapCopy(costmap);
        world_model = new base_local_planner::CostmapModel(costmap);
        geometry_msgs::PoseArray valid_locs;
        valid_locs.header.frame_id = "/base_link";
        valid_locs.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray invalid_locs;
        invalid_locs.header.frame_id = "/base_link";
        invalid_locs.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray close_locs;
        close_locs.header.frame_id = "/base_link";
        close_locs.header.stamp = ros::Time::now();
        for(int i=0;i<4000;i++) {
            geometry_msgs::PoseStamped cpose, odom_pose;
            cpose.header.frame_id = "/base_link";
            cpose.header.stamp = ros::Time(0);
            cpose.pose.position.x = (rand()%8000) / 1000.0 -4 ; 
            cpose.pose.position.y = (rand()%8000) / 1000.0 - 4;
            double rot = (rand()%10000) / 10000.0 * 2 * 3.14;
            cpose.pose.orientation = tf::createQuaternionMsgFromYaw(rot);
            cout << cpose.pose.orientation.z << " " << cpose.pose.orientation.w << " " << rot << endl;
            tf_listener.transformPose("/odom_combined", cpose, odom_pose);
            //uint32_t x, y;
            //if(!costmap.worldToMap(odom_pose.pose.position.x, odom_pose.pose.position.y, x, y))
                //continue;
            Eigen::Vector3f pos(odom_pose.pose.position.x, odom_pose.pose.position.y, tf::getYaw(odom_pose.pose.orientation));
            //cout << x << ", " << y << ":" << curpt.point.x << "," << curpt.point.y << "$";
            //cout << double(costmap.getCost(x,y)) << endl;
            double foot_cost = footprintCost(pos, 1);
            if(foot_cost == 0) 
                valid_locs.poses.push_back(cpose.pose);
            else if(foot_cost != -1)
                close_locs.poses.push_back(cpose.pose);
            else 
                invalid_locs.poses.push_back(cpose.pose);
            cout << foot_cost << endl;
        }
        cout << costmap_ros->getRobotFootprint().size() << endl;
        valid_pub.publish(valid_locs);
        invalid_pub.publish(invalid_locs);
        close_pub.publish(close_locs);

        geometry_msgs::PoseArray dense_table_poses;
        dense_table_poses.header.frame_id = "/base_link";
        dense_table_poses.header.stamp = ros::Time::now();
        uint32_t i2;
        for(uint32_t i=0;i<table_poly.size();i++) {
            i2 = i+1;
            if(i2 == table_poly.size())
                i2 = 0;
            double diffx = table_poly[i2].x-table_poly[i].x;
            double diffy = table_poly[i2].y-table_poly[i].y;
            double len = std::sqrt(diffx*diffx + diffy*diffy);
            double ang = std::atan2(diffy, diffx) - 3.14/2;
            double incx = std::cos(ang)*0.01, incy = std::sin(ang)*0.01;
            for(double t=0;t<len;t+=pose_step) {
                double polyx = table_poly[i].x + t*diffx;
                double polyy = table_poly[i].y + t*diffy;
                geometry_msgs::PoseStamped test_pose, odom_test_pose;
                bool found_pose = false;
                for(int k=start_dist/0.01;k<max_dist/0.01;k++) {
                    test_pose.header.frame_id = "/base_link";
                    test_pose.header.stamp = ros::Time(0);
                    test_pose.pose.position.x = polyx + incx*k;
                    test_pose.pose.position.y = polyy + incy*k;
                    test_pose.pose.orientation = tf::createQuaternionMsgFromYaw(ang+3.14);
                    tf_listener.transformPose("/odom_combined", test_pose, odom_test_pose);
                    Eigen::Vector3f pos(odom_test_pose.pose.position.x, 
                                        odom_test_pose.pose.position.y, 
                                        tf::getYaw(odom_test_pose.pose.orientation));
                    double foot_cost = footprintCost(pos, 1.0);
                    // found a valid pose
                    if(foot_cost >= 0 && foot_cost <= min_cost) {
                        found_pose = true;
                        break;
                    }
                    uint32_t mapx, mapy;
                    // break if we go outside the grid
                    if(!costmap.worldToMap(odom_test_pose.pose.position.x, 
                                           odom_test_pose.pose.position.y, mapx, mapy))
                        break;
                    double occ_map = double(costmap.getCost(mapx, mapy));
                    // break if we come across and obstacle
                    if(occ_map == costmap_2d::LETHAL_OBSTACLE ||
                       occ_map == costmap_2d::NO_INFORMATION)
                        break;
                }
                if(found_pose)
                    dense_table_poses.poses.push_back(test_pose.pose);
            }
        }
        ROS_INFO("POLY: %d, denseposes: %d", table_poly.size(), dense_table_poses.poses.size());

        // downsample and sort dense pose possibilties by distance to
        // approach point and thresholded distance to each other
        geometry_msgs::PoseArray downsampled_table_poses;
        boost::function<bool(geometry_msgs::Pose&, geometry_msgs::Pose&)> dist_comp
                          = boost::bind(&pose_dist_comp, _1, _2, approach_pt.point);
        while(ros::ok() && !dense_table_poses.poses.empty()) {
            // add the closest valid pose to the approach location on the table
            geometry_msgs::Pose new_pose = *std::min_element(
                        dense_table_poses.poses.begin(), dense_table_poses.poses.end(), 
                        dist_comp);
            downsampled_table_poses.poses.push_back(new_pose);
            // remove all poses in the dense sampling which are close to
            // the newest added pose
            boost::function<bool(geometry_msgs::Pose)> rem_thresh
                          = boost::bind(&pose_dist_thresh, _1, new_pose.position, 
                                        close_thresh);
            dense_table_poses.poses.erase(std::remove_if(
                                          dense_table_poses.poses.begin(), 
                                          dense_table_poses.poses.end(),
                                          rem_thresh),
                                          dense_table_poses.poses.end());
            ROS_INFO("denseposes: %d", dense_table_poses.poses.size());
        }
        downsampled_table_poses.header.frame_id = "/base_link";
        downsampled_table_poses.header.stamp = ros::Time::now();
        approach_pub.publish(downsampled_table_poses);
        resp.approach_poses = downsampled_table_poses;
        ROS_INFO("POLY: %d, poses: %d", table_poly.size(), downsampled_table_poses.poses.size());

        delete world_model;
        return true;
    }

    double TableApproaches::footprintCost(const Eigen::Vector3f& pos, double scale){
        double cos_th = cos(pos[2]);
        double sin_th = sin(pos[2]);

        std::vector<geometry_msgs::Point> scaled_oriented_footprint;
        for(unsigned int i  = 0; i < footprint_model.size(); ++i){
            geometry_msgs::Point new_pt;
            new_pt.x = pos[0] + (scale * footprint_model[i].x * cos_th - scale * footprint_model[i].y * sin_th);
            new_pt.y = pos[1] + (scale * footprint_model[i].x * sin_th + scale * footprint_model[i].y * cos_th);
            scaled_oriented_footprint.push_back(new_pt);
        }

        geometry_msgs::Point robot_position;
        robot_position.x = pos[0];
        robot_position.y = pos[1];

        //check if the footprint is legal
        double footprint_cost = world_model->footprintCost(robot_position, scaled_oriented_footprint, costmap.getInscribedRadius(), costmap.getCircumscribedRadius());

        return footprint_cost;
    }
};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "table_approaches");
    TableApproaches ta;
    ta.onInit();
    ros::spin();
    return 0;
}



