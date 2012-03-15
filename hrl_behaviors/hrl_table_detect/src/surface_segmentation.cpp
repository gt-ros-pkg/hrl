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
#include <hrl_table_detect/DetectTableStart.h>
#include <hrl_table_detect/DetectTableStop.h>
#include <hrl_table_detect/DetectTableInst.h>
#include <hrl_table_detect/SegmentSurfaces.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <std_srvs/Empty.h>

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
            ros::Publisher pc_pub, pc_pub2, pc_pub3, poly_pub;
            ros::ServiceServer get_pc_srv, get_table_srv;
            tf::TransformListener tf_listener;
            pcl::PointCloud<PRGB> accum_pc;
            geometry_msgs::PoseArray grasp_points;
            bool pc_captured, do_capture;

            SurfaceSegmentation();
            void onInit();
            void pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg);
            bool captureCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);
            bool surfSegCallback(hrl_table_detect::SegmentSurfaces::Request& req, 
                                 hrl_table_detect::SegmentSurfaces::Response& resp);
    };

    SurfaceSegmentation::SurfaceSegmentation() {
    }

    void SurfaceSegmentation::onInit() {
        pc_pub = nh.advertise<pcl::PointCloud<PRGB> >("normal_vis", 1);
        pc_pub2 = nh.advertise<pcl::PointCloud<PRGB> >("normal_vis2", 1);
        pc_pub3 = nh.advertise<pcl::PointCloud<PRGB> >("normal_vis3", 1);
        poly_pub = nh.advertise<visualization_msgs::Marker>("table_hull", 1);
        pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &SurfaceSegmentation::pcCallback, this);
        get_pc_srv = nh.advertiseService("surf_seg_capture_pc", &SurfaceSegmentation::captureCallback, this);
        get_table_srv = nh.advertiseService("segment_surfaces", &SurfaceSegmentation::surfSegCallback, this);
        pc_captured = false; do_capture = false;
        ros::Duration(1.0).sleep();
    }

    bool SurfaceSegmentation::captureCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
        pc_captured = false; do_capture = true;
        ros::Rate r(100);
        double start_time = ros::Time::now().toSec();
        while(ros::ok() && !pc_captured && ros::Time::now().toSec() - start_time < 1) {
            ros::spinOnce();
            r.sleep();
        }
        do_capture = false;
        return true;
    }

    void SurfaceSegmentation::pcCallback(sensor_msgs::PointCloud2::ConstPtr pc_msg) {
        if(!(do_capture && !pc_captured))
            return;
        pcl::PointCloud<PRGB>::Ptr pc_full_ptr(new pcl::PointCloud<PRGB>());
        pcl::fromROSMsg(*pc_msg, *pc_full_ptr);
        if(accum_pc.points.size() == 0)
            accum_pc = *pc_full_ptr;
        else
            accum_pc += *pc_full_ptr;
        pc_captured = true;
    }

    bool SurfaceSegmentation::surfSegCallback(
                     hrl_table_detect::SegmentSurfaces::Request& req, 
                     hrl_table_detect::SegmentSurfaces::Response& resp) {
        double min_z_val = 0.5, max_z_val = 1.5;
        double norm_ang_thresh = 0.7;
        //double surf_clust_dist = 0.03, surf_clust_min_size = 50;
        double surf_clust_dist = 0.05, surf_clust_min_size = 600;

        pcl::PointCloud<PRGB>::Ptr pc_full_frame_ptr(new pcl::PointCloud<PRGB>());
        string base_frame("/base_link");
        ros::Time now = ros::Time::now();
        accum_pc.header.stamp = now;

        // Transform PC to base frame
        tf_listener.waitForTransform(accum_pc.header.frame_id, base_frame, now, ros::Duration(3.0));
        pcl_ros::transformPointCloud(base_frame, accum_pc, *pc_full_frame_ptr, tf_listener);

        sensor_msgs::PointCloud2::Ptr pc2_full_frame_ptr(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2::Ptr pc2_downsampled_ptr(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*pc_full_frame_ptr, *pc2_full_frame_ptr);
        pcl::PointCloud<PRGB>::Ptr pc_downsampled_ptr(new pcl::PointCloud<PRGB>());
        pcl::VoxelGrid<sensor_msgs::PointCloud2> vox_grid;
        vox_grid.setInputCloud(pc2_full_frame_ptr);
        vox_grid.setLeafSize(0.01, 0.01, 0.01);
        vox_grid.filter(*pc2_downsampled_ptr);
        pcl::fromROSMsg(*pc2_downsampled_ptr, *pc_downsampled_ptr);
        pc_pub.publish(*pc2_downsampled_ptr);

        // Filter floor and ceiling
        pcl::PointCloud<PRGB>::Ptr pc_filtered_ptr(new pcl::PointCloud<PRGB>());
        pcl::PassThrough<PRGB> z_filt;
        z_filt.setFilterFieldName("z");
        z_filt.setFilterLimits(min_z_val, max_z_val);
        //z_filt.setInputCloud(pc_full_frame_ptr);
        z_filt.setInputCloud(pc_downsampled_ptr);
        z_filt.filter(*pc_filtered_ptr);

        if(pc_filtered_ptr->size() < 20)
            return false;

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
        mls.setSearchRadius(0.05);
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
            return false;

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
            cout << surf_clust_list[i].indices.size() << endl;

            for(uint32_t j=0;j<surf_coeffs.values.size();j++)
                cout << i << " " << j << " " << surf_coeffs.values[j] << endl;
        }
        pc_filtered_ptr->header.stamp = ros::Time::now();
        pc_filtered_ptr->header.frame_id = base_frame;
        pc_pub2.publish(pc_filtered_ptr);

        if(surf_models.size() == 0)
            return false;

        pcl::PointCloud<PRGB> flat_pc;
        BOOST_FOREACH(const PRGB& pt, pc_full_frame_ptr->points) {
            if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
                continue;
            PRGB n_pt;
            n_pt.x = pt.x; n_pt.y = pt.y; n_pt.z = pt.z; 
            double a = surf_models[0].values[0], b = surf_models[0].values[1];
            double c = surf_models[0].values[2], d = surf_models[0].values[3];
            double dist = (a*pt.x + b*pt.y + c*pt.z + d) / sqrt(a*a+b*b+c*c);
            
            if(fabs(dist) < 0.02) {
                uint32_t green = 0xFF00FF00;
                ((uint32_t*) &n_pt.rgb)[0] = green;
            } else if (dist > 0) {
                uint32_t blue = 0xFF0000FF;
                ((uint32_t*) &n_pt.rgb)[0] = blue;
            } else {
                uint32_t red = 0xFFFF0000;
                ((uint32_t*) &n_pt.rgb)[0] = red;
            }
            flat_pc.push_back(n_pt);
        }
        flat_pc.header.stamp = ros::Time::now();
        flat_pc.header.frame_id = pc_full_frame_ptr->header.frame_id;
        pc_pub3.publish(flat_pc);

        // project table inliers onto plane model found
        pcl::PointCloud<PRGB>::Ptr table_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ProjectInliers<PRGB> proj_ins;
        proj_ins.setInputCloud(pc_filtered_ptr);
        proj_ins.setIndices(boost::make_shared<pcl::PointIndices>(surf_clust_list[0]));
        proj_ins.setModelType(pcl::SACMODEL_PLANE);
        proj_ins.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(surf_models[0]));
        proj_ins.filter(*table_proj);

        // convex hull of largest surface
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
        convex_hull.setInputCloud(table_proj);
        //convex_hull.setIndices(boost::make_shared<pcl::PointIndices>(surf_clust_list[0]));
        convex_hull.reconstruct(*cloud_hull);

        // publish table hull polygon
        visualization_msgs::Marker hull_poly;
        hull_poly.type = visualization_msgs::Marker::LINE_STRIP;
        hull_poly.action = visualization_msgs::Marker::ADD;
        hull_poly.ns = "table_hull";
        hull_poly.header.frame_id = pc_filtered_ptr->header.frame_id;
        hull_poly.header.stamp = now;
        hull_poly.pose.orientation.w = 1;
        hull_poly.scale.x = 0.01; hull_poly.scale.y = 0.01; hull_poly.scale.z = 0.01; 
        hull_poly.color.g = 1; hull_poly.color.a = 1;
        for(uint32_t j=0;j<cloud_hull->points.size();j++) {
            geometry_msgs::Point n_pt;
            n_pt.x = cloud_hull->points[j].x; 
            n_pt.y = cloud_hull->points[j].y; 
            n_pt.z = cloud_hull->points[j].z; 
            hull_poly.points.push_back(n_pt);
        }
        hull_poly.points.push_back(hull_poly.points[0]);
        poly_pub.publish(hull_poly);
        accum_pc.points.clear();
        resp.surfaces.push_back(hull_poly);
        return true;
    }

};

using namespace hrl_table_detect;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_segmentation");
    SurfaceSegmentation ta;
    ta.onInit();
    ros::spin();
    return 0;
}



