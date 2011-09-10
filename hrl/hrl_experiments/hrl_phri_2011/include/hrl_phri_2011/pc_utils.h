#include <map>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/format.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/make_shared.hpp>
#include <boost/foreach.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include <hrl_phri_2011/PHRIPointCloudCapture.h>

using namespace std;
using namespace Eigen;

typedef hrl_phri_2011::PHRIPointCloudCapture PCC;
typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;
typedef pcl::PointCloud<pcl::Normal> PCNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFeatures;
typedef pcl::KdTree<PRGB> KDTree;
typedef map<string, Eigen::Affine3d, less<string>, 
            Eigen::aligned_allocator<std::pair<const string, Eigen::Affine3d> > > map_str_t3d;


void boxFilter(const PCRGB &in_pc, PCRGB &out_pc,
               double min_x, double max_x, double min_y, double max_y, double min_z, double max_z) {
    pcl::ConditionAnd<PRGB>::Ptr near_cond(new pcl::ConditionAnd<PRGB>());
    PCRGB::Ptr near_pts(new PCRGB());
    pcl::ConditionalRemoval<PRGB> near_extract;
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "x", pcl::ComparisonOps::GT, min_x)));
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "x", pcl::ComparisonOps::LT, max_x)));
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "y", pcl::ComparisonOps::GT, min_y)));
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "y", pcl::ComparisonOps::LT, max_y)));
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "z", pcl::ComparisonOps::GT, min_z)));
    near_cond->addComparison(pcl::FieldComparison<PRGB>::Ptr(new pcl::FieldComparison<PRGB>(
                             "z", pcl::ComparisonOps::LT, max_z)));
    near_extract.setCondition(near_cond);
    near_extract.setKeepOrganized(true);
    near_extract.setInputCloud(in_pc.makeShared());
    near_extract.filter(out_pc);
}

void transformPC(const PCRGB &in_pc, PCRGB &out_pc, 
                 const Eigen::Affine3d& transform) {
    MatrixXd pt_mat = MatrixXd::Constant(4, in_pc.points.size(), 1.0);
    uint32_t i = 0;
    BOOST_FOREACH(PRGB const pt, in_pc.points) {
        pt_mat(0, i) = pt.x; pt_mat(1, i) = pt.y; pt_mat(2, i) = pt.z; 
        i++;
    }
    MatrixXd trans_pt_mat = transform.matrix() * pt_mat;
    for(i=0;i<in_pc.points.size();i++) {
        PRGB pt;
        pt.x = trans_pt_mat(0, i); pt.y = trans_pt_mat(1, i); pt.z = trans_pt_mat(2, i); 
        pt.rgb = in_pc.points[i].rgb;
        out_pc.points.push_back(pt);
    }
}

void pubLoop(PCRGB &pc, const std::string& topic, double rate = 1.0) {
    ros::NodeHandle nh;
    ros::Publisher pub_pc = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
    ros::Rate r(rate);
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(pc, pc_msg);
    while(ros::ok()) {
        pc_msg.header.stamp = ros::Time::now();
        pub_pc.publish(pc_msg);
        r.sleep();
    }
}

void computeNormals(const PCRGB::Ptr& in_pc, const KDTree::Ptr& in_kd_tree, PCNormals::Ptr& out_normals) {
    pcl::MovingLeastSquares<PRGB, pcl::Normal> mls;
    PCRGB mls_points;
    mls.setInputCloud(in_pc);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(in_kd_tree);
    mls.setSearchRadius(0.02);
    mls.setOutputNormals(out_normals);
    mls.reconstruct(mls_points);
}

void computeFPFH(const PCRGB::Ptr& in_pc, FPFeatures::Ptr& out_features) {
    KDTree::Ptr kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    PCNormals::Ptr normals(new PCNormals());
    computeNormals(in_pc, kd_tree, normals);
    pcl::FPFHEstimation<PRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(in_pc);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(kd_tree);
    fpfh_est.setRadiusSearch(0.02);
    fpfh_est.compute(*out_features);
}

void computeSACRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc) {
    ROS_INFO("IN ALIGN");
    FPFeatures::Ptr target_features(new FPFeatures());
    FPFeatures::Ptr source_features(new FPFeatures());
    computeFPFH(target_pc, target_features);
    computeFPFH(source_pc, source_features);
    ROS_INFO("OUT FEATS");
    pcl::SampleConsensusInitialAlignment<PRGB, PRGB, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputTarget(target_pc);
    sac_ia.setTargetFeatures(target_features);
    sac_ia.setInputCloud(source_pc);
    sac_ia.setSourceFeatures(source_features);
    PCRGB reg_out;
    sac_ia.align(reg_out);
    ROS_INFO("OUT ALIGN1");
    Eigen::Affine3d sac_tf;
    sac_tf.matrix() = sac_ia.getFinalTransformation().cast<double>();
    PCRGB tf_pc;
    transformPC(*source_pc, tf_pc, sac_tf);
    tf_pc += *target_pc;
    ROS_INFO("OUT ALIGN, %f %f %f", sac_tf.matrix()(0, 3), sac_tf.matrix()(1, 3), sac_tf.matrix()(2, 3));
    reg_out += *target_pc;
    ROS_INFO("in pub");
    pubLoop(tf_pc, "/pub_pc");
}

void computeICPRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                            Eigen::Affine3d& tf_mat, int max_iters=300) {
    pcl::IterativeClosestPointNonLinear<PRGB, PRGB> icp;
    icp.setInputTarget(target_pc);
    icp.setInputCloud(source_pc);
    icp.setMaximumIterations(max_iters);
    PCRGB tmp_pc;
    icp.align(tmp_pc);
    tf_mat = icp.getFinalTransformation().cast<double>();
}

