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
#include <hrl_table_detect/DetectTableStart.h>
#include <hrl_table_detect/DetectTableStop.h>
#include <hrl_table_detect/DetectTableInst.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

using namespace sensor_msgs;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
namespace hrl_table_detect {
    
    typedef pcl::PointXYZRGB PRGB;
    typedef pcl::PointXYZRGBNormal PRGBN;

    class SurfaceSegmentation {
        public:
            ros::Subscriber pc_sub, cam_sub;
            ros::NodeHandle nh;
            ros::Publisher pc_pub, pc_pub2, pc_pub3;
            tf::TransformListener tf_listener;

            SurfaceSegmentation();
            void onInit();
            void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg);
            //void modelCallback(image_geometry::PinholeCameraModel::ConstPtr pin_msg);
    };

    SurfaceSegmentation::SurfaceSegmentation() {
    }

    void SurfaceSegmentation::onInit() {
        pc_pub = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis", 1);
        pc_pub2 = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis2", 1);
        pc_pub3 = nh.advertise<pcl::PointCloud<PRGB> >("/normal_vis3", 1);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &SurfaceSegmentation::pcCallback, this);
        //cam_sub = nh.subscribe("/kinect_head/rgb/camera_info", 1, &SurfaceSegmentation::modelCallback, this);
        ros::Duration(1.0).sleep();
    }

    void SurfaceSegmentation::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        double min_z_val = 0.1, max_z_val = 1.5;
        double norm_ang_thresh = 0.7;
        double surf_clust_dist = 0.02, surf_clust_min_size = 50;

        pcl::PointCloud<PRGB>::Ptr pc_full_ptr(new pcl::PointCloud<PRGB>());
        pcl::PointCloud<PRGB>::Ptr pc_full_frame_ptr(new pcl::PointCloud<PRGB>());
        pcl::fromROSMsg(*pc_msg, *pc_full_ptr);
        string base_frame("/base_link");
        ros::Time now = ros::Time::now();

        // Transform PC to base frame
        tf_listener.waitForTransform(pc_msg->header.frame_id, base_frame, now, ros::Duration(3.0));
        pcl_ros::transformPointCloud(base_frame, *pc_full_ptr, *pc_full_frame_ptr, tf_listener);

        // Filter floor and ceiling
        pcl::PointCloud<PRGB>::Ptr pc_filtered_ptr(new pcl::PointCloud<PRGB>());
        pcl::PassThrough<PRGB> z_filt;
        z_filt.setFilterFieldName("z");
        z_filt.setFilterLimits(min_z_val, max_z_val);
        z_filt.setInputCloud(pc_full_frame_ptr);
        z_filt.filter(*pc_filtered_ptr);

        if(pc_filtered_ptr->size() < 20)
            return;

        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>());
        pcl::KdTree<PRGB>::Ptr normals_tree (new pcl::KdTreeFLANN<PRGB> ());
        pcl::PointCloud<PRGB> mls_points;
        pcl::MovingLeastSquares<PRGB, pcl::Normal> mls;
        normals_tree->setInputCloud(pc_filtered_ptr);
        mls.setOutputNormals(cloud_normals_ptr);
        mls.setInputCloud(pc_filtered_ptr);
        mls.setPolynomialFit(true);
        mls.setSearchMethod(normals_tree);
        mls.setSearchRadius(0.02);
        mls.reconstruct(mls_points);
        
        /*pcl::NormalEstimation<PRGB, pcl::Normal> norm_est;
        norm_est.setKSearch(15);
        norm_est.setSearchMethod(normals_tree);
        norm_est.setInputCloud(pc_filtered_ptr);
        norm_est.compute(*cloud_normals_ptr);
        */
        
        pcl::PointIndices::Ptr flat_inds_ptr (new pcl::PointIndices());
        double ang;
        int i = 0;
        BOOST_FOREACH(const pcl::Normal& pt, cloud_normals_ptr->points) {
            ang = fabs((acos(pt.normal[2]) - CV_PI/2)/CV_PI*2);
            if(ang > norm_ang_thresh)
                flat_inds_ptr->indices.push_back(i);
            i++;
        }

        if(flat_inds_ptr->indices.size() < 20)
            return;

        // Cluster into distinct surfaces
        pcl::EuclideanClusterExtraction<PRGB> surf_clust;
        pcl::KdTree<PRGB>::Ptr clust_tree (new pcl::KdTreeFLANN<PRGB> ());
        surf_clust.setClusterTolerance(surf_clust_dist);
        surf_clust.setMinClusterSize(surf_clust_min_size);
        surf_clust.setIndices(flat_inds_ptr);
        surf_clust.setInputCloud(pc_filtered_ptr);
        surf_clust.setSearchMethod(clust_tree);
        std::vector<pcl::PointIndices> surf_clust_list;
        surf_clust.extract(surf_clust_list);

        // Fit planes to all of the surfaces
        std::vector<pcl::ModelCoefficients> surf_models;
        pcl::SACSegmentationFromNormals<PRGB, pcl::Normal> sac_seg;
        sac_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        sac_seg.setMethodType(pcl::SAC_MSAC);
        sac_seg.setDistanceThreshold(0.03);
        sac_seg.setMaxIterations(10000);
        sac_seg.setNormalDistanceWeight(0.1);
        sac_seg.setOptimizeCoefficients(true);
        sac_seg.setProbability(0.99);

        for(uint32_t i =0;i<surf_clust_list.size();i++) {
            sac_seg.setInputNormals(cloud_normals_ptr);
            sac_seg.setInputCloud(pc_filtered_ptr);
            pcl::PointIndices::Ptr surf_clust_list_ptr (new pcl::PointIndices());
            surf_clust_list_ptr->indices = surf_clust_list[i].indices;
            //std::copy(surf_clust_list[i].indices.begin(), surf_clust_list[i].indices.end(), surf_clust_list_ptr->indices.begin());
            surf_clust_list_ptr->header = surf_clust_list[i].header;
            sac_seg.setIndices(surf_clust_list_ptr);
            pcl::PointIndices surf_inliers;
            pcl::ModelCoefficients surf_coeffs;
            sac_seg.segment(surf_inliers, surf_coeffs);
            surf_models.push_back(surf_coeffs);
            uint32_t cur_color = 0xFF000000 | (rand() % 0x01000000);
            for(uint32_t j=0;j<surf_inliers.indices.size();j++)
                ((uint32_t*) &pc_filtered_ptr->points[surf_inliers.indices[j]].rgb)[0] = cur_color;

            for(uint32_t j=0;j<surf_coeffs.values.size();j++)
                cout << i << " " << j << " " << surf_coeffs.values[j] << endl;
        }
        pc_filtered_ptr->header.stamp = ros::Time::now();
        pc_filtered_ptr->header.frame_id = base_frame;
        pc_pub2.publish(pc_filtered_ptr);

        // define a plane to project onto
        geometry_msgs::PoseStamped plane_pose;
        plane_pose.pose.position.x=0; plane_pose.pose.position.y=0; plane_pose.pose.position.z=0.542; 
        plane_pose.pose.orientation.x=0; plane_pose.pose.orientation.y=0; 
        plane_pose.pose.orientation.z=0; plane_pose.pose.orientation.w=1; 
        plane_pose.header.stamp = now; plane_pose.header.frame_id = base_frame;
        tf_listener.transformPose(pc_msg->header.frame_id, plane_pose, plane_pose);
        btQuaternion plane_quat(plane_pose.pose.orientation.x,
                                plane_pose.pose.orientation.y,
                                plane_pose.pose.orientation.z,
                                plane_pose.pose.orientation.w);
        btMatrix3x3 plane_rot(plane_quat);
        btVector3 plane_norm = plane_rot.getColumn(2);
        btVector3 plane_pt(plane_pose.pose.position.x,plane_pose.pose.position.y,
                           plane_pose.pose.position.z);
        i=0;
        pcl::PointCloud<PRGB> flat_pc;
        BOOST_FOREACH(const PRGB& pt, pc_full_ptr->points) {
            i++;
            if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                continue;
            btVector3 pc_pt(pt.x,pt.y,pt.z);
            double t = plane_norm.dot(plane_pt)/plane_norm.dot(pc_pt);
            btVector3 surf_pt = pc_pt*t;
            PRGB n_pt;
            n_pt.x = surf_pt.x(); n_pt.y = surf_pt.y(); n_pt.z = surf_pt.z(); 
            n_pt.rgb = pt.rgb;
            if(t > 1.01) {
                uint32_t red = 0xFFFF0000;
                ((uint32_t*) &n_pt.rgb)[0] = red;
            } else if (t < 0.99) {
                uint32_t blue = 0xFF0000FF;
                ((uint32_t*) &n_pt.rgb)[0] = blue;
            } else {
                uint32_t green = 0xFF00FF00;
                ((uint32_t*) &n_pt.rgb)[0] = green;
            }
            flat_pc.push_back(n_pt);
        }
        flat_pc.header.stamp = ros::Time::now();
        flat_pc.header.frame_id = pc_full_ptr->header.frame_id;
        pc_pub3.publish(flat_pc);


    }

    //void SurfaceSegmentation::modelCallback(image_geometry::PinholeCameraModel::ConstPtr pin_msg) {
    //    cam_model.fromCameraInfo(pin_msg);
    //}
};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_normals");
    SurfaceSegmentation ta;
    ta.onInit();
    ros::spin();
    return 0;
}



