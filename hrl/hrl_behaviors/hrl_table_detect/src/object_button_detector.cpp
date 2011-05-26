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

namespace hrl_table_detect {

    class ObjectButtonDetector {
        public:
            ros::NodeHandle nh;
            tf::TransformListener tf_listener;
            ros::Subscriber pc_sub;
            ros::Publisher pt3d_pub;
            ros::ServiceServer table_obj_srv;
            ros::ServiceClient table_seg_srv;
            std::string camera_frame;

            ObjectButtonDetector();
            void onInit();
            bool detectCallback(ObjectButtonDetect::Request& req, ObjectButtonDetect::Response& resp);

    };

    ObjectButtonDetector::ObjectButtonDetector() : img_trans(nh) {
    }

    void ObjectButtonDetector::onInit() {
        nh.param<std::string>("camera_frame", camera_frame, "/openni_rgb_optical_frame");

        table_obj_srv = nh.advertiseService("object_button_detect", 
                                            &ObjectButtonDetector::detectCallback, this);
        table_seg_srv = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(
                               "/object_detection", false);
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
        geometry_msgs::PoseStamped table_img_frame;
        tf_listener.transformPose(camera_frame, table_seg_resp.table, table_img_frame);
        float table_height = table_img_frame.pose.position.z;
        for(uint32_t i=0;i<table_seg_resp.clusters.size();i++) {
            // transform object into image frame
            sensor_msgs::PointCloud obj_img_frame;
            tf_listener.transformPointCloud(camera_frame, table_seg_resp.clusters[i], 
                                                                 obj_img_frame);
            // project object silhouette onto table
            pcl::PointCloud<pcl::PointXYZ>::Ptr table_proj_obj 
                                                      (new pcl::PointCloud<pcl::PointXYZ>);
            table_proj_obj->header.frame_id = camera_frame;
            BOOST_FOREACH(const geometry_msgs::Point32& pt, table_seg_resp.clusters[i].points) {
                if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                    continue;
                float t = pt.z / table_height;
                geometry_msgs::Point32 proj_pt;
                proj_pt.x = pt.x * t; proj_pt.y = pt.y * t; proj_pt.z = table_height; 
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
            hull_poly.header.frame_id = table_proj_obj.header.frame_id;
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
