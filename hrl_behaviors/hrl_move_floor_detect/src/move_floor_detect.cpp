#include <numeric>
#include <ros/ros.h>
#include <algorithm>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/make_shared.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <hrl_move_floor_detect/SegmentFloor.h>

using namespace sensor_msgs;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
namespace hrl_move_floor_detect {

    class MoveFloorDetect {
        public:
            ros::NodeHandle nh;
            tf::TransformListener tf_listener;
            ros::Publisher pc_pub, hull_pub;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_geometry::PinholeCameraModel cam_model;
            ros::ServiceServer seg_floor_srv;
            costmap_2d::Costmap2D costmap;
            costmap_2d::Costmap2DROS* costmap_ros;
            boost::mt19937 rand_gen;
            vector<geometry_msgs::Point> footprint_model;
            base_local_planner::CostmapModel* world_model;

            MoveFloorDetect();
            void onInit();
            int randomInt(int a, int b=-1);
            bool segFloorCallback(SegmentFloor::Request& req, SegmentFloor::Response& resp);
            void cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg);
            double footprintCost(const Eigen::Vector3f& pos, double scale);
    };

    MoveFloorDetect::MoveFloorDetect() : img_trans(nh)  {
        onInit();
    }

    void MoveFloorDetect::onInit() {
        costmap_ros = new costmap_2d::Costmap2DROS("table_costmap", tf_listener);
        footprint_model = costmap_ros->getRobotFootprint();

        camera_sub = img_trans.subscribeCamera<MoveFloorDetect>
                                              ("/kinect_head/rgb/image_color", 1, 
                                               &MoveFloorDetect::cameraCallback, this);
        seg_floor_srv = nh.advertiseService("move_floor_detect", &MoveFloorDetect::segFloorCallback, this);
        
        pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("move_floor_pc", 1);
        hull_pub = nh.advertise<visualization_msgs::Marker>("floor_hull", 1);
        ros::Duration(1.0).sleep();
        ROS_INFO("[move_floor_detect] move_floor_detect loaded.");
    }

    void MoveFloorDetect::cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cam_model.fromCameraInfo(info_msg);
    }

    int MoveFloorDetect::randomInt(int a, int b) {
        if(b == -1) { b = a; a = 0; }
        boost::uniform_int<> dist(a, b-1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > vgen(rand_gen, dist);
        return vgen();
    }
    
    bool MoveFloorDetect::segFloorCallback(SegmentFloor::Request& req, SegmentFloor::Response& resp) {
        int min_cost = 250;
        double surf_clust_dist = 0.10, surf_clust_min_size = 30;

        costmap_ros->getCostmapCopy(costmap);
        world_model = new base_local_planner::CostmapModel(costmap);

        pcl::PointCloud<pcl::PointXYZI>::Ptr move_floor_pc (new pcl::PointCloud<pcl::PointXYZI>);
        cv::Size res = cam_model.fullResolution();
        double start_time = ros::Time::now().toSec();
        while(ros::Time::now().toSec() - start_time < 1.0 && ros::ok()) {
            // pick a random point on the image
            cv::Point2d img_pt(randomInt(res.width), randomInt(res.height));

            // project the point onto the floor
            cv::Point3d img_ray = cam_model.projectPixelTo3dRay(img_pt);
            geometry_msgs::Vector3Stamped img_vec, img_vec_trans;
            img_vec.header.frame_id = cam_model.tfFrame();
            img_vec.header.stamp = ros::Time(0);
            img_vec.vector.x = img_ray.x; img_vec.vector.y = img_ray.y; img_vec.vector.z = img_ray.z; 
            tf_listener.transformVector("/odom_combined", img_vec, img_vec_trans);
            tf::StampedTransform cam_trans;
            tf_listener.lookupTransform("/odom_combined", cam_model.tfFrame(), ros::Time(0), 
                                        cam_trans);
            // p = p0 + t * v, pz = 0
            double t = - cam_trans.getOrigin().z() / img_vec_trans.vector.z;

            // find the cost of this position on the floor
            double floor_pos_x = cam_trans.getOrigin().x()+t*img_vec_trans.vector.x;
            double floor_pos_y = cam_trans.getOrigin().y()+t*img_vec_trans.vector.y;
            Eigen::Vector3f floor_pos(floor_pos_x,
                                      floor_pos_y,
                                      std::atan2(floor_pos_y, floor_pos_x));
            double foot_cost = footprintCost(floor_pos, 0.5);
            // throw out unmoveable positions
            if(foot_cost < 0 || foot_cost > min_cost)
                continue;

            // Add point to point cloud of move positions
            pcl::PointXYZI pc_pt;
            pc_pt.x = floor_pos_x; pc_pt.y = floor_pos_y; pc_pt.z = 0;
            pc_pt.intensity = (256 - foot_cost)/256;
            move_floor_pc->points.push_back(pc_pt);
        }
        if(move_floor_pc->points.empty())
            return false;
        move_floor_pc->header.frame_id = "/odom_combined";
        move_floor_pc->header.stamp = ros::Time::now();
        pc_pub.publish(move_floor_pc);

        // Cluster into distinct surfaces
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> surf_clust;
        pcl::KdTree<pcl::PointXYZI>::Ptr clust_tree (new pcl::KdTreeFLANN<pcl::PointXYZI> ());
        surf_clust.setClusterTolerance(surf_clust_dist);
        surf_clust.setMinClusterSize(surf_clust_min_size);
        surf_clust.setInputCloud(move_floor_pc);
        surf_clust.setSearchMethod(clust_tree);
        std::vector<pcl::PointIndices> surf_clust_list;
        surf_clust.extract(surf_clust_list);

        for(uint32_t i =0;i<surf_clust_list.size();i++) {
            // find concave hull of surface
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr voronoi_centers (new pcl::PointCloud<pcl::PointXYZI>);
            std::vector<pcl::Vertices> hull_verts;
            pcl::ConcaveHull<pcl::PointXYZI> concave_hull;
            concave_hull.setInputCloud(move_floor_pc);
            concave_hull.setIndices(boost::make_shared<pcl::PointIndices>(surf_clust_list[i]));
            concave_hull.setAlpha(0.1);
            concave_hull.setVoronoiCenters(voronoi_centers);
            concave_hull.reconstruct(*cloud_hull, hull_verts);

            // create polygon of floor surface
            visualization_msgs::Marker hull_poly;
            hull_poly.type = visualization_msgs::Marker::LINE_STRIP; 
            hull_poly.action = visualization_msgs::Marker::ADD;
            hull_poly.ns = "floor_hull";
            hull_poly.header.frame_id = "/odom_combined";
            hull_poly.header.stamp = ros::Time::now();
            hull_poly.id = i;
            hull_poly.pose.orientation.w = 1;
            hull_poly.scale.x = 0.01; hull_poly.scale.y = 0.01; hull_poly.scale.z = 0.01; 
            hull_poly.color.r = 1; hull_poly.color.a = 1;
            for(uint32_t j=0;j<cloud_hull->points.size();j++) {
                geometry_msgs::Point n_pt;
                n_pt.x = cloud_hull->points[j].x; 
                n_pt.y = cloud_hull->points[j].y; 
                n_pt.z = cloud_hull->points[j].z; 
                hull_poly.points.push_back(n_pt);
            }
            if(!hull_poly.points.empty()) {
                hull_poly.points.push_back(hull_poly.points[0]);
                resp.surfaces.push_back(hull_poly);
                hull_pub.publish(hull_poly);
            }
        }
        ROS_INFO("[move_floor_detect] Number of floor surfaces: %d", (int) resp.surfaces.size());

        delete world_model;
        return true;
    }

    double MoveFloorDetect::footprintCost(const Eigen::Vector3f& pos, double scale){
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

using namespace hrl_move_floor_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_floor_detect");
    MoveFloorDetect mfd;
    ros::spin();
    return 0;
}



