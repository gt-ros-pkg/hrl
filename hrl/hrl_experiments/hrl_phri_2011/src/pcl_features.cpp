#include<hrl_phri_2011/pcl_features.h>

void computeNormals(const PCRGB::Ptr& in_pc, const KDTree::Ptr& in_kd_tree, PCNormals::Ptr& out_normals) 
{
    pcl::MovingLeastSquares<PRGB, pcl::Normal> mls;
    PCRGB mls_points;
    mls.setInputCloud(in_pc);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(in_kd_tree);
    mls.setSearchRadius(0.02);
    mls.setOutputNormals(out_normals);
    mls.reconstruct(mls_points);
}

void computeFPFH(const PCRGB::Ptr& in_pc, FPFeatures::Ptr& out_features) 
{
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

void computeSACRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc) 
{
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
                            Eigen::Affine3d& tf_mat, int max_iters, double color_weight) 
                            {
    pcl::IterativeClosestPointNonLinear<PRGB, PRGB> icp;
    boost::shared_ptr<pcl::DefaultPointRepresentation<PRGB> const> pt_rep(new pcl::DefaultPointRepresentation<PRGB>(color_weight, color_weight, color_weight));
    icp.setPointRepresentation(pt_rep);
    icp.setInputTarget(target_pc);
    icp.setInputCloud(source_pc);
    icp.setMaximumIterations(max_iters);
    PCRGB tmp_pc;
    icp.align(tmp_pc);
    tf_mat = icp.getFinalTransformation().cast<double>();
}
