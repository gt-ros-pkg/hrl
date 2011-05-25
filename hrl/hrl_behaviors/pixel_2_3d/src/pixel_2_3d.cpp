#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/transforms.h>

#include <pixel_2_3d/Pixel23d.h>

namespace pixel_2_3d {

    class Pixel23dServer {
        public:
            ros::NodeHandle nh;
            tf::TransformListener tf_listener;
            ros::Subscriber pc_sub;
            ros::Publisher pt3d_pub;
            ros::ServiceServer pix_srv;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_geometry::PinholeCameraModel cam_model;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_pc;

            Pixel23dServer();
            void onInit();
            void cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg);
            void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg);
            bool pixCallback(Pixel23d::Request& req, Pixel23d::Response& resp);

    };

    Pixel23dServer::Pixel23dServer() : img_trans(nh),
                                       cur_pc(new pcl::PointCloud<pcl::PointXYZRGB>) {
        onInit();
    }

    void Pixel23dServer::onInit() {
        camera_sub = img_trans.subscribeCamera<Pixel23dServer>
                                              ("image", 1, 
                                               &Pixel23dServer::cameraCallback, this);
        pc_sub = nh.subscribe("point_cloud", 1, &Pixel23dServer::pcCallback, this);
        pix_srv = nh.advertiseService("pixel_2_3d", &Pixel23dServer::pixCallback, this);
        pt3d_pub = nh.advertise<geometry_msgs::PoseStamped>("pixel3d", 1);
        ros::Duration(1).sleep();
    }

    void Pixel23dServer::cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cam_model.fromCameraInfo(info_msg);
    }

    void Pixel23dServer::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        pcl::fromROSMsg(*pc_msg, *cur_pc);
        pcl_ros::transformPointCloud(cam_model.tfFrame(), *cur_pc, *cur_pc, tf_listener);
    }

    double pixDist(cv::Point2d& a, cv::Point2d& b) {
        double dist = std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
        if(dist != dist)
            return 1e8;
        return dist;
    }
    
    bool Pixel23dServer::pixCallback(Pixel23d::Request& req, Pixel23d::Response& resp) {
        cv::Point2d img_pix(req.pixel.point.x, req.pixel.point.y);
        std::vector<double> dists;
        for(uint32_t i=0;i<cur_pc->points.size();i++) {
            cv::Point3d pt3d(cur_pc->points[i].x, cur_pc->points[i].y, cur_pc->points[i].z);
            cv::Point2d pt_pix = cam_model.project3dToPixel(pt3d);
            dists.push_back(pixDist(pt_pix, img_pix));
        }
        uint32_t min_ind = std::min_element(dists.begin(), dists.end()) - dists.begin();
        resp.pixel3d.header.frame_id = cur_pc->header.frame_id;
        resp.pixel3d.header.stamp = cur_pc->header.stamp;
        resp.pixel3d.point.x = cur_pc->points[min_ind].x;
        resp.pixel3d.point.y = cur_pc->points[min_ind].y;
        resp.pixel3d.point.z = cur_pc->points[min_ind].z;
        geometry_msgs::PoseStamped pt3d;
        pt3d.header.frame_id = resp.pixel3d.header.frame_id;
        pt3d.header.stamp = ros::Time::now();
        pt3d.pose.position = resp.pixel3d.point;
        pt3d.pose.orientation.w = 1;
        pt3d_pub.publish(pt3d);
        return true;
    }

};


using namespace pixel_2_3d;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_2_3d");
    Pixel23dServer p3d;
    ros::spin();
    return 0;
}
