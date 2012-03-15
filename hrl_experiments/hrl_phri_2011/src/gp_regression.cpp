#include <Eigen/Eigen>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <iostream>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>
#include <unsupported/Eigen/CholmodSupport>
#include <pcl/kdtree/kdtree_flann.h>

#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/pcl_basic.h>

#define SE(r, l) ( std::exp(-(r) * (r) / (2 * (l) * (l))) )

using namespace Eigen;
using namespace boost;

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

class GPRegressor
{
public:
    PCRGB::Ptr input_pc, data_pc;
    pcl::KdTreeFLANN<PRGB> data_kd_tree;
    SparseLLT<SparseMatrix<double>, Cholmod > K_inverse;
    shared_ptr<SparseMatrix<double> > K_sparse, Ks_sparse;
    shared_ptr<DynamicSparseMatrix<double> > K_dyn, Ks_dyn;
    VectorXd y, alpha;
    double length_scale, signal_variance, noise_variance;
    vector<int> nonzero_inds;
    size_t training_size, testing_size;

    GPRegressor();
    void importTrainingData(const PCRGB::Ptr& input_pc);
    void computeK();
    void computeAlpha();
    void computeKs(const PCRGB::Ptr& test_pc);
    void findRegression(const PCRGB::Ptr& test_pc, VectorXd& f_mean);
    double logPseudoLikelihood();
};

GPRegressor::GPRegressor() :
    data_pc(new PCRGB()),
    data_kd_tree(new pcl::KdTreeFLANN<PRGB> ())
{
    ros::param::param<double>("~length_scale", length_scale, 0.01);
    ros::param::param<double>("~signal_variance", signal_variance, 0.01);
    ros::param::param<double>("~noise_variance", noise_variance, 0.01);
    ROS_INFO("length_scale: %f, signal_variance: %f, noise_variance: %f", 
              length_scale, signal_variance, noise_variance);
}

void GPRegressor::importTrainingData(const PCRGB::Ptr& input_pc)
{
    // prune data_pc
    pcl::KdTreeFLANN<PRGB> kd_tree_input(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree_input.setInputCloud(input_pc);
    vector<int> inds; vector<float> dists;
    for(size_t i=0;i<input_pc->size();i++) {
        kd_tree_input.radiusSearch(i, 3 * length_scale, inds, dists);
        if(dists.size() != 0)
            data_pc->points.push_back(input_pc->points[i]);
        inds.clear(); dists.clear();
    }
    training_size = data_pc->size();
    y.resize(training_size);
    for(size_t i=0;i<training_size;i++)
        y(i) = data_pc->points[i].rgb;
    data_kd_tree.setInputCloud(data_pc);
}

void GPRegressor::computeK() 
{
    ROS_INFO("Computing K(X, X) = K");
    K_dyn.reset(new DynamicSparseMatrix<double>(training_size, training_size));
    int nnz = 0;
    vector<int> inds; vector<float> dists;
    for(size_t i=0;i<training_size;i++) {
        data_kd_tree.radiusSearch(i, 3 * length_scale, inds, dists);
        for(size_t j=0;j<dists.size();j++) {
            if(i < (size_t) inds[j])
                K_dyn->coeffRef(i, inds[j]) = SQ(signal_variance) * SE(dists[j], length_scale);
            else
                K_dyn->coeffRef(inds[j], i) = SQ(signal_variance) * SE(dists[j], length_scale);
            nnz++;
        }
        inds.clear(); dists.clear();
        K_dyn->coeffRef(i, i) = SQ(noise_variance);
    }
    ROS_INFO("Sparsity: %f", nnz / (double) (training_size * training_size));
    K_sparse.reset(new SparseMatrix<double>(*K_dyn));
    ROS_INFO("Inverting Covariance Matrix K");
    K_inverse.compute(*K_sparse);
    ROS_INFO("Inversion Completed (success? %d)", K_inverse.succeeded());
}

void GPRegressor::computeAlpha() 
{
    ROS_INFO("Solving K alpha = y");
    //alpha = y;
    alpha = K_inverse.solve(y);
    ROS_INFO("Alpha computed.");
}

double GPRegressor::logPseudoLikelihood()
{
    double u_i_diff, s_i_sq, K_inv_i_i, pred_log_prob_i;
    double loo_log_pred_prob = 0;
    VectorXd e_i(training_size), e_sol;
    for(size_t i=0;i<training_size;i++) {
        e_i.setZero();
        e_i(i) = 1;
        e_sol = K_inverse.solve(e_i);
        K_inv_i_i = e_sol(i);
        u_i_diff = alpha(i) / K_inv_i_i;
        s_i_sq = 1.0 / K_inv_i_i;
        pred_log_prob_i = -0.5 * log(s_i_sq) - SQ(u_i_diff) / (2.0 * s_i_sq) - 0.5 * log(2.0 * PI);
        loo_log_pred_prob += pred_log_prob_i;
    }
    return loo_log_pred_prob;
}

void GPRegressor::computeKs(const PCRGB::Ptr& test_pc) 
{
    testing_size = test_pc->size();
    Ks_dyn.reset(new DynamicSparseMatrix<double>(training_size, testing_size));
    ROS_INFO("Computing K(X, Xtest) = Ks");
    vector<int> inds; vector<float> dists;
    for(size_t i=0;i<testing_size;i++) {
        data_kd_tree.radiusSearch(*test_pc, i, 3 * length_scale, inds, dists);
        for(size_t j=0;j<dists.size();j++)
            Ks_dyn->coeffRef(inds[j], i) = SE(dists[j], length_scale);
        if(dists.size() != 0)
            nonzero_inds.push_back(i);
        inds.clear(); dists.clear();
    }
    Ks_sparse.reset(new SparseMatrix<double>(*Ks_dyn));
    ROS_INFO("Ks Computed");
    VectorXd sol = K_sparse->transpose() * alpha;
    for(size_t i=0;i<training_size;i++) 
        if(i%30==0)
            ROS_INFO("y: %f, sol: %f, diff: %f", y(i), sol(i), y(i) - sol(i));
}

void GPRegressor::findRegression(const PCRGB::Ptr& test_pc, VectorXd& f_mean)
{
    computeK();
    computeAlpha();
    ROS_INFO("LOO log likelihood: %f", logPseudoLikelihood());
    computeKs(test_pc);
    f_mean = Ks_sparse->transpose() * alpha;
    f_mean = f_mean.array();
    ROS_INFO("Ridge regression found");
    /*
    for(size_t i=0;i<training_size;i++)
        if(i%100==0)
            printf("(%f, %f) ", alpha(i), y(i));
    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gp_regression");

    // Load PC bag
    PCRGB::Ptr input_pc, head_pc;
    vector<PCRGB::Ptr> pc_list;
    readBagTopic<PCRGB>(argv[1], pc_list, "/data_cloud");
    input_pc = pc_list[0];
    
    pc_list.clear();
    readBagTopic<PCRGB>(argv[2], pc_list, "/stitched_head");
    head_pc = pc_list[0];

    GPRegressor gp_reg;
    gp_reg.importTrainingData(input_pc);
    Eigen::VectorXd f_mean;
    gp_reg.findRegression(head_pc, f_mean);
    ROS_INFO("f_mean size: %d, nzs: %d", f_mean.size(), gp_reg.nonzero_inds.size());

    PCRGB fm_pc, fm_color_pc;
    for(size_t i=0;i<head_pc->points.size();i++) {
        /*
        if(i%30==0 && f_mean(i) != 0)
            ROS_INFO("%f", f_mean(i));
        */
        PRGB npt;
        const PRGB pt = head_pc->points[i];
        npt.x = pt.x; npt.y = pt.y; npt.z = pt.z;
        npt.rgb = f_mean(i);
        //npt.rgb = gp_reg.y[i];
        fm_pc.points.push_back(npt);
        npt.rgb = pt.rgb;
        fm_color_pc.points.push_back(npt);
    }
    colorizeDataPC(fm_pc, fm_color_pc, gp_reg.nonzero_inds, 100);
    ROS_INFO("Done processing, publishing colored PC to /function_mean");
    fm_color_pc.header.frame_id = "/base_link";
    pubLoop(fm_color_pc, "/function_mean", 1);
}
