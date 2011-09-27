#include <Eigen/Eigen>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <iostream>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>
//#include <unsupported/Eigen/CholmodSupport>
#include <pcl/kdtree/kdtree_flann.h>

#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/pcl_basic.h>

#define SE(r, l) ( std::exp(-(r) * (r) / (2 * (l) * (l))) )

void colorizeDataPC(const PCRGB& data_pc, PCRGB& color_pc, vector<int> color_inds, double saturation)
{
    vector<float> data;
    for(size_t i=0;i<color_inds.size();i++) 
        data.push_back(data_pc.points[color_inds[i]].rgb);
    vector<int> all_inds, null_inds(color_pc.points.size() - color_inds.size());
    for(size_t i=0;i<color_pc.points.size();i++)
        all_inds.push_back(i);
    set_difference(all_inds.begin(), all_inds.end(), color_inds.begin(), color_inds.end(), null_inds.begin());

    float max_val = *std::max_element(data.begin(), data.end());
    ROS_INFO("Max data value: %f", max_val);
    double h, s, l;
    for(size_t i=0;i<color_inds.size();i++) {
        extractHSL(color_pc.points[color_inds[i]].rgb, h, s, l);
        h = (double) 240.0 * data[i] / max_val;
        if(h < 0) h = 0; if(h > 240.0) h = 240.0;
        writeHSL(240.0 - h, saturation, l, color_pc.points[color_inds[i]].rgb);
    }

    for(size_t i=0;i<null_inds.size();i++) {
        extractHSL(color_pc.points[null_inds[i]].rgb, h, s, l);
        writeHSL(h, 0, l, color_pc.points[null_inds[i]].rgb);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gp_regression");
    double cov_sigma, noise_sigma;
    ros::param::param<double>("~cov_sigma", cov_sigma, 0.01);
    ros::param::param<double>("~noise_sigma", noise_sigma, 0.01);
    ROS_INFO("cov_sigma: %f, noise_sigma: %f", cov_sigma, noise_sigma);

    vector<int> inds;
    vector<float> dists;

    // Load PC bag
    vector<PCRGB::Ptr> pc_list;
    readBagTopic<PCRGB>(argv[1], pc_list, "/data_cloud");
    PCRGB::Ptr input_pc = pc_list[0];
    PCRGB::Ptr data_pc(new PCRGB());
    
    // prune data_pc
    pcl::KdTreeFLANN<PRGB> kd_tree_input(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree_input.setInputCloud(input_pc);
    for(size_t i=0;i<input_pc->points.size();i++) {
        kd_tree_input.radiusSearch(i, 3 * cov_sigma, inds, dists);
        if(dists.size() != 0)
            data_pc->points.push_back(input_pc->points[i]);
        inds.clear(); dists.clear();
    }

    pc_list.clear();
    readBagTopic<PCRGB>(argv[2], pc_list, "/stitched_head");
    PCRGB::Ptr head_pc = pc_list[0];
    pcl::KdTreeFLANN<PRGB> kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    Eigen::DynamicSparseMatrix<double> Kdyn(data_pc->size(), data_pc->size());
    kd_tree.setInputCloud(data_pc);
    Eigen::VectorXd y(data_pc->points.size());
    Eigen::VectorXd alpha(data_pc->points.size());
    int nnz = 0;
    inds.clear(); dists.clear();
    for(size_t i=0;i<data_pc->points.size();i++) {
        kd_tree.radiusSearch(i, 3 * cov_sigma, inds, dists);
        for(size_t j=0;j<dists.size();j++) {
            if(i > (size_t) inds[j])
                Kdyn.coeffRef(i, inds[j]) = SE(dists[j], cov_sigma);
            else
                Kdyn.coeffRef(inds[j], i) = SE(dists[j], cov_sigma);
            nnz++;
        }
        inds.clear(); dists.clear();
        Kdyn.coeffRef(i, i) = noise_sigma * noise_sigma;
        y(i) = data_pc->points[i].rgb;
        alpha(i) = data_pc->points[i].rgb;
        if(i%100==0)
            ROS_INFO("%f", y(i));
    }
    ROS_INFO("Sparsity: %f", nnz / (double) (data_pc->size() * data_pc->size()));
    //Eigen::SparseLLT<Eigen::DynamicSparseMatrix<float>, Eigen::Cholmod> llt_solver;
    Eigen::SparseMatrix<double> K(Kdyn);
    Eigen::SparseLDLT<Eigen::SparseMatrix<double> > llt_solver;
    ROS_INFO("Inverting Covariance Matrix K");
    llt_solver.compute(K);
    llt_solver.solveInPlace(alpha);
    ROS_INFO("Inversion Completed");

    ROS_INFO("Computing K(X, Xtest) = Ks");
    Eigen::DynamicSparseMatrix<double> Ksdyn(data_pc->size(), head_pc->size());
    vector<int> color_inds;
    inds.clear(); dists.clear();
    for(size_t i=0;i<head_pc->points.size();i++) {
        kd_tree.radiusSearch(*head_pc, i, 3 * cov_sigma, inds, dists);
        for(size_t j=0;j<dists.size();j++)
            Ksdyn.coeffRef(inds[j], i) = SE(dists[j], cov_sigma);
        if(dists.size() != 0)
            color_inds.push_back(i);
        inds.clear(); dists.clear();
    }
    Eigen::SparseMatrix<double> Ks(Ksdyn);
    ROS_INFO("Ks Computed");

    for(size_t i=0;i<data_pc->points.size();i++)
        if(i%30==0)
            ROS_INFO("%f %f", alpha(i), y(i));

    Eigen::VectorXd f_mean = Ks.transpose() * alpha;
    PCRGB fm_pc, fm_color_pc;
    for(size_t i=0;i<head_pc->points.size();i++) {
        if(i%30==0 && f_mean(i) != 0)
            ROS_INFO("%f", f_mean(i));
        PRGB npt;
        const PRGB pt = head_pc->points[i];
        npt.x = pt.x; npt.y = pt.y; npt.z = pt.z;
        npt.rgb = f_mean(i);
        fm_pc.points.push_back(npt);
        npt.rgb = pt.rgb;
        fm_color_pc.points.push_back(npt);
    }
    colorizeDataPC(fm_pc, fm_color_pc, color_inds, 100);
    ROS_INFO("Done processing, publishing colored PC to /function_mean");
    fm_color_pc.header.frame_id = "/base_link";
    data_pc->header.frame_id = "/base_link";
    //pubLoop(*data_pc, "/function_mean", 1);
    pubLoop(fm_color_pc, "/function_mean", 1);
}
