#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/transforms.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <hrl_table_detect/ObjectButtonDetect.h>

namespace hrl_table_detect {

    class ObjectButtonDetector {
        public:
            ros::NodeHandle nh;
            tf::TransformListener tf_listener;
            ros::ServiceServer table_obj_srv;
            ros::ServiceClient table_seg_srv;
            ros::Publisher hull_pub;
            std::string camera_frame;

            ObjectButtonDetector();
            void onInit();
            bool detectCallback(ObjectButtonDetect::Request& req, ObjectButtonDetect::Response& resp);

    };

    ObjectButtonDetector::ObjectButtonDetector() {
    }

    void ObjectButtonDetector::onInit() {
        nh.param<std::string>("camera_frame", camera_frame, "/openni_rgb_optical_frame");

        table_obj_srv = nh.advertiseService("object_button_detect", 
                                            &ObjectButtonDetector::detectCallback, this);
        table_seg_srv = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(
                               "table_segmentation", false);
        hull_pub = nh.advertise<visualization_msgs::Marker>("object_hull", 1);
        ROS_INFO("[object_button_detector] ObjectButtonDetector loaded.");
        ros::Duration(1).sleep();
    }

    bool ObjectButtonDetector::detectCallback(ObjectButtonDetect::Request& req, 
                                              ObjectButtonDetect::Response& resp) {
        // call tabletop segmentation
        tabletop_object_detector::TabletopSegmentation::Request table_seg_req;
        tabletop_object_detector::TabletopSegmentation::Response table_seg_resp;
        table_seg_srv.call(table_seg_req, table_seg_resp);


        // get table height in image frame
        geometry_msgs::PointStamped table_pt, table_base_frame, cam_pt, cam_base_frame;
        table_pt.header.frame_id = "/torso_lift_link";
        table_pt.header.stamp = table_seg_resp.table.pose.header.stamp;
        table_pt.point.x = table_seg_resp.table.pose.pose.position.x;
        table_pt.point.y = table_seg_resp.table.pose.pose.position.y;
        table_pt.point.z = table_seg_resp.table.pose.pose.position.z;
        cam_pt.header.frame_id = camera_frame;
        cam_pt.header.stamp = ros::Time(0);
        cam_pt.point.x = 0; cam_pt.point.y = 0; cam_pt.point.z = 0; 

        tf_listener.transformPoint("/base_footprint", table_pt, table_base_frame);
        tf_listener.transformPoint("/base_footprint", cam_pt, cam_base_frame);
        float table_height = table_base_frame.point.z;
        for(uint32_t i=0;i<table_seg_resp.clusters.size();i++) {
            // transform object into image frame
            sensor_msgs::PointCloud obj_img_frame;
            table_seg_resp.clusters[i].header.frame_id = "/torso_lift_link";
            tf_listener.transformPointCloud("/base_footprint", table_seg_resp.clusters[i], 
                                                                 obj_img_frame);
            // project object silhouette onto table
            pcl::PointCloud<pcl::PointXYZ>::Ptr table_proj_obj 
                                                      (new pcl::PointCloud<pcl::PointXYZ>);
            BOOST_FOREACH(const geometry_msgs::Point32& pt, obj_img_frame.points) {
                if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                    continue;
                geometry_msgs::Vector3Stamped ray, ray_trans;
                ray.header.frame_id = camera_frame;
                ray.vector.x = pt.x - cam_base_frame.point.x; 
                ray.vector.y = pt.y - cam_base_frame.point.y; 
                ray.vector.z = pt.z - cam_base_frame.point.z; 
                tf_listener.transformVector("/base_footprint", ray, ray_trans);
                float t = (table_height - cam_base_frame.point.z) / ray.vector.z;
                pcl::PointXYZ proj_pt;
                proj_pt.x = cam_base_frame.point.x + ray.vector.x * t; 
                proj_pt.y = cam_base_frame.point.y + ray.vector.y * t; 
                proj_pt.z = table_height; 
                table_proj_obj->points.push_back(proj_pt);
            }

            // Find concave_hull of resulting points
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull 
                                                      (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr voronoi_centers 
                                                      (new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::Vertices> hull_verts;
            pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
            concave_hull.setInputCloud(table_proj_obj);
            concave_hull.setAlpha(0.1);
            concave_hull.setVoronoiCenters(voronoi_centers);
            concave_hull.reconstruct(*cloud_hull, hull_verts);

            // create polygon of projected object
            visualization_msgs::Marker hull_poly;
            hull_poly.type = visualization_msgs::Marker::LINE_STRIP; 
            hull_poly.action = visualization_msgs::Marker::ADD;
            hull_poly.ns = "object_hull";
            hull_poly.header.frame_id = "/base_footprint";
            hull_poly.header.stamp = ros::Time::now();
            hull_poly.id = i;
            hull_poly.pose.orientation.w = 1;
            hull_poly.scale.x = 0.01; hull_poly.scale.y = 0.01; hull_poly.scale.z = 0.01; 
            hull_poly.color.r = 1; hull_poly.color.b = 1; hull_poly.color.a = 1;
            BOOST_FOREACH(const pcl::PointXYZ& pt, cloud_hull->points) {
                geometry_msgs::Point n_pt; 
                n_pt.x = pt.x; n_pt.y = pt.y; n_pt.z = pt.z; 
                hull_poly.points.push_back(n_pt);
            }
            hull_poly.points.push_back(hull_poly.points[0]);
            resp.objects.push_back(hull_poly);
            hull_pub.publish(hull_poly);
        }
        return true;
    }

};


using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_button_detector");
    ObjectButtonDetector obd;
    obd.onInit();
    ros::spin();
    return 0;
}
