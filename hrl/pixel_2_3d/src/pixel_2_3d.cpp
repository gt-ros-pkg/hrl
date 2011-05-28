#include <numeric>
#include <ros/ros.h>
#include <algorithm>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/conditional_removal.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/transforms.h>

#include <pixel_2_3d/Pixel23d.h>

#define DIST3(x1,y1,z1,x2,y2,z2) (std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2)))
#define SQRCOMP(x) (std::sqrt(1 - x*x))
#define SQRCOMP2(x,y) (std::sqrt(1/(1+x*x/(y*y))))
typedef pcl::PointXYZRGB PRGB;

namespace pixel_2_3d {

    class Pixel23dServer {
        public:
            ros::NodeHandle nh;
            tf::TransformListener tf_listener;
            ros::Subscriber pc_sub, l_click_sub;
            ros::Publisher pt3d_pub;
            ros::Publisher pc_pub;
            ros::ServiceServer pix_srv;
            image_transport::ImageTransport img_trans;
            image_transport::CameraSubscriber camera_sub;
            image_geometry::PinholeCameraModel cam_model;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_pc;
            bool monitor_mode;

            Pixel23dServer();
            void onInit();
            void cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg);
            void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg);
            bool pixCallback(Pixel23d::Request& req, Pixel23d::Response& resp);
            void lClickCallback(const geometry_msgs::PointStamped& click_msg);

    };

    Pixel23dServer::Pixel23dServer() : nh("~"), img_trans(nh),
                                       cur_pc(new pcl::PointCloud<pcl::PointXYZRGB>) {
        onInit();
    }

    void Pixel23dServer::onInit() {
        nh.param<bool>("monitor_mode", monitor_mode, false);
        camera_sub = img_trans.subscribeCamera<Pixel23dServer>
                                              ("/image", 1, 
                                               &Pixel23dServer::cameraCallback, this);
        pc_sub = nh.subscribe("/point_cloud", 1, &Pixel23dServer::pcCallback, this);
        pix_srv = nh.advertiseService("/pixel_2_3d", &Pixel23dServer::pixCallback, this);
        pt3d_pub = nh.advertise<geometry_msgs::PoseStamped>("/pixel3d", 1);
        pc_pub = nh.advertise<pcl::PointCloud<PRGB> >("/pcnear", 1);
        if(monitor_mode) {
            l_click_sub = nh.subscribe("/l_mouse_click", 1, &Pixel23dServer::lClickCallback, this);
        }
        ROS_INFO("[pixel_2_3d] Pixel23dServer loaded");
        ros::Duration(1).sleep();
    }

    void Pixel23dServer::cameraCallback(const sensor_msgs::ImageConstPtr& img_msg,
                                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cam_model.fromCameraInfo(info_msg);
    }

    void Pixel23dServer::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        pcl::fromROSMsg(*pc_msg, *cur_pc);
    }

    double pixDist(cv::Point2d& a, cv::Point2d& b) {
        double dist = std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
        if(dist != dist)
            return 1e8;
        return dist;
    }
    
    bool Pixel23dServer::pixCallback(Pixel23d::Request& req, Pixel23d::Response& resp) {
        double voxel_size = 0.04;
int yo = 1;
ROS_INFO("HERE %d", yo++);

        pcl_ros::transformPointCloud(cam_model.tfFrame(), *cur_pc, *cur_pc, tf_listener);
        cv::Point2d img_pix(req.pixel.point.x, req.pixel.point.y);
        std::vector<double> dists;
        for(uint32_t i=0;i<cur_pc->points.size();i++) {
            cv::Point3d pt3d(cur_pc->points[i].x, cur_pc->points[i].y, cur_pc->points[i].z);
            cv::Point2d pt_pix = cam_model.project3dToPixel(pt3d);
            dists.push_back(pixDist(pt_pix, img_pix));
        }
        uint32_t min_ind = std::min_element(dists.begin(), dists.end()) - dists.begin();
        geometry_msgs::PointStamped pt3d, pt3d_trans;
        pt3d_trans.header.frame_id = cur_pc->header.frame_id;
        pt3d_trans.header.stamp = ros::Time::now();
        pt3d_trans.point.x = cur_pc->points[min_ind].x;
        pt3d_trans.point.y = cur_pc->points[min_ind].y;
        pt3d_trans.point.z = cur_pc->points[min_ind].z;

ROS_INFO("HERE %d", yo++);
        // Filter to only points in small voxel range
        pcl::ConditionAnd<PRGB>::Ptr near_cond(new pcl::ConditionAnd<PRGB>());
        pcl::PointCloud<PRGB>::Ptr near_pts(new pcl::PointCloud<PRGB>());
        pcl::ConditionalRemoval<PRGB> near_extract;
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "x", pcl::ComparisonOps::GT, pt3d_trans.point.x - voxel_size/2)));
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "x", pcl::ComparisonOps::LT, pt3d_trans.point.x + voxel_size/2)));
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "y", pcl::ComparisonOps::GT, pt3d_trans.point.y - voxel_size/2)));
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "y", pcl::ComparisonOps::LT, pt3d_trans.point.y + voxel_size/2)));
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "z", pcl::ComparisonOps::GT, pt3d_trans.point.z - voxel_size/2)));
        near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                                 "z", pcl::ComparisonOps::LT, pt3d_trans.point.z + voxel_size/2)));
        near_extract.setCondition(near_cond);
        near_extract.setKeepOrganized(false);
        near_extract.setInputCloud(cur_pc);
        near_extract.filter(*near_pts);
        std::vector<int> inds;
        pcl::removeNaNFromPointCloud<PRGB>(*near_pts, *near_pts, inds);
        pc_pub.publish(*near_pts);

ROS_INFO("HERE %d", yo++);
        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr normals_ptr(new pcl::PointCloud<pcl::Normal>());
        pcl::KdTree<PRGB>::Ptr normals_tree (new pcl::KdTreeFLANN<PRGB> ());
        pcl::PointCloud<PRGB> mls_points;
        pcl::MovingLeastSquares<PRGB, pcl::Normal> mls;
        normals_tree->setInputCloud(near_pts);
        mls.setOutputNormals(normals_ptr);
        mls.setInputCloud(near_pts);
        mls.setPolynomialFit(true);
        mls.setSearchMethod(normals_tree);
        mls.setSearchRadius(0.025);
        mls.reconstruct(mls_points);

ROS_INFO("HERE %d", yo++);
        // find closest point in new set
        double min_dist = 10000;
        uint32_t min_ind_near = 0;
        printf("size %d %d\n", near_pts->points.size(), cur_pc->points.size());
        for(uint32_t i=0;i<near_pts->points.size();i++) {
            double dist = DIST3(near_pts->points[i].x, near_pts->points[i].y, near_pts->points[i].z,
                                pt3d_trans.point.x, pt3d_trans.point.y, pt3d_trans.point.z);
            if(dist < min_dist) {
                min_ind_near = i;
                min_dist = dist;
            }
        }
        int new_ind = std::find(inds.begin(), inds.end(), min_ind) - inds.begin();
        printf("indss %d %d\n", min_ind_near, new_ind);

ROS_INFO("HERE %d", yo++);
        // convert normal to quaternion
        double nx = normals_ptr->points[min_ind_near].normal[0];
        double ny = normals_ptr->points[min_ind_near].normal[1];
        double nz = normals_ptr->points[min_ind_near].normal[2];
        double j = SQRCOMP2(ny, nz);
        double k = -ny*j/nz;
        btMatrix3x3 M (normals_ptr->points[min_ind_near].normal[0],
                       ny*k - nz*j,
                       0,
                       normals_ptr->points[min_ind_near].normal[1],
                       -nx*k,
                       j,
                       normals_ptr->points[min_ind_near].normal[2],
                       nx*j,
                       k);
        printf("%d %f %f %f\n", min_ind_near, normals_ptr->points[min_ind_near].normal[0],normals_ptr->points[min_ind_near].normal[1],normals_ptr->points[min_ind_near].normal[2]);
        btQuaternion quat;
        M.getRotation(quat);

ROS_INFO("HERE %d", yo++);
        //tf_listener.transformPoint("/base_footprint", pt3d, pt3d_trans);
        resp.pixel3d.header.frame_id = pt3d_trans.header.frame_id;
        resp.pixel3d.header.stamp = ros::Time::now();
        resp.pixel3d.point.x = pt3d_trans.point.x;
        resp.pixel3d.point.y = pt3d_trans.point.y;
        resp.pixel3d.point.z = pt3d_trans.point.z;
        geometry_msgs::PoseStamped pt3d_pose;
        pt3d_pose.header.frame_id = cam_model.tfFrame();
        pt3d_pose.header.stamp = ros::Time::now();
        pt3d_pose.pose.position.x = pt3d_trans.point.x;
        pt3d_pose.pose.position.y = pt3d_trans.point.y;
        pt3d_pose.pose.position.z = pt3d_trans.point.z;
        pt3d_pose.pose.orientation.x = quat.getX();
        pt3d_pose.pose.orientation.y = quat.getY();
        pt3d_pose.pose.orientation.z = quat.getZ();
        pt3d_pose.pose.orientation.w = quat.getW();
ROS_INFO("sssss %f", pt3d_pose.pose.orientation.w);
        pt3d_pub.publish(pt3d_pose);
ROS_INFO("HERE %d", yo++);
        return true;
    }

    void Pixel23dServer::lClickCallback(const geometry_msgs::PointStamped& click_msg) {
        Pixel23d::Request req; Pixel23d::Response resp;
        req.pixel.point.x = click_msg.point.x;
        req.pixel.point.y = click_msg.point.y;
        pixCallback(req, resp);
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
