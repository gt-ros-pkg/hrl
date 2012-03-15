#include <hrl_phri_2011/ellipsoid_space.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigen>
#include <tf_conversions/tf_eigen.h>
#include <hrl_phri_2011/EllipsoidParams.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/utils.h>

#define NORMAL(x, sig) ( std::exp( - (x) * (x) / (2.0 * (sig) * (sig))) / std::sqrt(2.0 * 3.14159 * (sig) * (sig)))

void projectEllipsoid(Ellipsoid& ell, double ell_height, const PCRGB& pc, PCRGB& pc_ell);
void createForceCloud(const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, PCRGB& fpc);
void colorPCHSL(const PCRGB& in_pc, const vector<double>& values, PCRGB& out_pc, double hue=0);
void createPriorCloud(const PCRGB& in_pc, const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, PCRGB& out_pc);
void marginalEllipsoid(const PCRGB& pc, const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, 
                       const PCRGB& fpc, PCRGB& pc_ell, double sigma = 0.01);
void projectEllipsoidDense(Ellipsoid& ell, double ell_height, const PCRGB& pc, 
                           int num_lat, int num_lon, double sigma = 0.01, int k = 10);

void projectEllipsoid(Ellipsoid& ell, double ell_height, const PCRGB& pc, PCRGB& pc_ell) 
{
    double lat, lon, height, x, y, z, h, s, l;
    BOOST_FOREACH(PRGB const pt, pc.points) {
        PRGB npt;
        extractHSL(pt.rgb, h, s, l);
        ell.cartToEllipsoidal(pt.x, pt.y, pt.z, lat, lon, height);
        ell.ellipsoidalToCart(lat, lon, ell_height, x, y, z);
        npt.x = x; npt.y = y; npt.z = z;
        writeHSL(0, 0, l, npt.rgb);
        pc_ell.points.push_back(npt);
    }
    pc_ell.header.frame_id = "base_link";
    //pubLoop(pc_ell, "/proj_head");
}

void createForceCloud(const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, PCRGB& fpc) 
{
    /*
    Affine3d a3d_tf(Quaternion<double>(0.994613, 0.0269077, -0.00270719, 0.100069));
    a3d_tf = Translation3d(0, 0.0284424, 0.0108643) * a3d_tf;
    */
    for(uint32_t i=0;i<fps.size();i++) {
        /*
        Affine3d a3d = a3d_tf * Translation3d(fps[i]->tool_frame.transform.translation.x,
                                              fps[i]->tool_frame.transform.translation.y,
                                              fps[i]->tool_frame.transform.translation.z);
        pt.x = a3d(0, 3);
        pt.y = a3d(1, 3);
        pt.z = a3d(2, 3);
        */
        PRGB pt;
        pt.x = fps[i]->tool_frame.transform.translation.x;
        pt.y = fps[i]->tool_frame.transform.translation.y;
        pt.z = fps[i]->tool_frame.transform.translation.z;
        ((uint32_t*) &pt.rgb)[0] = 0xffffffff;
        fpc.points.push_back(pt);
    }
}

void createPriorCloud(const PCRGB& in_pc, const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, PCRGB& out_pc) 
{
    double prior_sigma;
    ros::param::param<double>("~prior_sigma", prior_sigma, 0.01);

    pcl::KdTreeFLANN<PRGB> kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree.setInputCloud(in_pc.makeShared());
    vector<int> inds1(1), inds2;
    vector<float> dists1(1), dists2;
    vector<double> pdf(in_pc.size());
    double normal, Z = 0;
    for(uint32_t i=0;i<fps.size();i++) {
        PRGB pt;
        pt.x = fps[i]->tool_frame.transform.translation.x;
        pt.y = fps[i]->tool_frame.transform.translation.y;
        pt.z = fps[i]->tool_frame.transform.translation.z;
        inds1[0] = -1;
        kd_tree.nearestKSearch(pt, 1, inds1, dists1);
        if(inds1[0] == -1)
            continue;
        inds2.clear(); dists2.clear();
        kd_tree.radiusSearch(pt, 3 * prior_sigma, inds2, dists2);
        for(size_t j=0;j<inds2.size();j++) {
            normal = NORMAL(dists2[j], prior_sigma);
            pdf[inds2[j]] += normal;
            Z += normal;
        }
    }
    colorPCHSL(in_pc, pdf, out_pc, 0);
}

void colorPCHSL(const PCRGB& in_pc, const vector<double>& values, PCRGB& out_pc, double hue)
{
    double max_val = *max_element(values.begin(), values.end());
    double h, s, l;
    for(size_t i=0;i<in_pc.size();i++) {
        const PRGB pt = in_pc.points[i];
        PRGB npt;
        npt.x = pt.x; npt.y = pt.y; npt.z = pt.z;

        extractHSL(in_pc.points[i].rgb, h, s, l);
        s = 100.0 * values[i] / max_val;
        writeHSL(0, s, l, npt.rgb);
        out_pc.points.push_back(npt);
    }
}

void marginalEllipsoid(const PCRGB& pc, const vector<hrl_phri_2011::ForceProcessed::Ptr>& fps, 
                       const PCRGB& fpc, PCRGB& pc_ell, double sigma) 
{
    pcl::KdTreeFLANN<PRGB> kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree.setInputCloud(fpc.makeShared());
    double h, s, l, normal, normal_sum, val_sum, val;
    int k = 5;
    vector<int> finds;
    vector<float> fdists;
    for(uint32_t i=0;i<pc.points.size();i++) {
        extractHSL(pc_ell.points[i].rgb, h, s, l);
        finds.resize(k, 0); fdists.resize(k, 10000);
        kd_tree.nearestKSearch(pc, i, 1, finds, fdists);
        normal_sum = 0; val_sum = 0;
        for(uint32_t j=0;j<finds.size();j++) {
            if(sqrt(fdists[j]) > sigma * 3)
                continue;
            if(fps[finds[j]]->force_normal < 0.2)
                continue;
            normal = NORMAL(fdists[j], sigma);
            val_sum += normal * fps[finds[j]]->force_normal;
            normal_sum += normal;
        }
        if(normal_sum == 0)
            continue;
        val = val_sum / normal_sum;
        s = 20 * val;
        if(s < 0)
            s = 0;
        if(s > 100)
            s = 100;
        writeHSL(0, s, l, pc_ell.points[i].rgb);
    }
}

void projectEllipsoidDense(Ellipsoid& ell, double ell_height, const PCRGB& pc, 
                           int num_lat, int num_lon, double sigma, int k) 
{
    pcl::KdTreeFLANN<PRGB> kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree.setInputCloud(pc.makeShared());
    PCRGB pc_dense;
    vector<int> inds;
    vector<float> dists;
    double h, s, l, h_sum, s_sum, l_sum, normal, normal_sum;
    double x, y, z;
    double lat = 0, lon = 0;
    for(int i=0;i<num_lat;i++) {
        lat += PI / num_lat;
        lon = 0;
        for(int j=0;j<num_lon;j++) {
            lon += 2 * PI / num_lon;
            PRGB pt;
            ell.ellipsoidalToCart(lat, lon, ell_height, x, y, z);
            pt.x = x; pt.y = y; pt.z = z;
            inds.clear();
            dists.clear();
            kd_tree.nearestKSearch(pt, k, inds, dists);
            normal_sum = 0; h_sum = 0; s_sum = 0; l_sum = 0;
            for(uint32_t inds_i=0;inds_i<inds.size();inds_i++) {
                extractHSL(pc.points[inds[inds_i]].rgb, h, s, l);
                normal = NORMAL(dists[inds_i], sigma);
                normal_sum += normal;
                h_sum += normal * h; s_sum += normal * s; l_sum += normal * l;
            }
            writeHSL(h_sum / normal_sum, s_sum / normal_sum, l_sum / normal_sum, pt.rgb);
            pc_dense.points.push_back(pt);
        }
    }
    pc_dense.header.frame_id = "/base_link";
    pubLoop(pc_dense, "/proj_head");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_pc_ellipsoid");

    PCRGB pc_head;
    Ellipsoid ell;
    loadRegisteredHead(argv[1], argv[2], pc_head, ell);

    // load forces
    vector<hrl_phri_2011::ForceProcessed::Ptr> forces;
    readBagTopic<hrl_phri_2011::ForceProcessed>(argv[3], forces, "/force_processed");

    PCRGB fpc;
    createForceCloud(forces, fpc);
    fpc.header.frame_id = "/base_link";
    pubLoop(fpc, "force_cloud");
    return 0;
    PCRGB prior_cloud;
    createPriorCloud(pc_head, forces, prior_cloud);
    prior_cloud.header.frame_id = "/base_link";
    pubLoop(prior_cloud, "prior_cloud");

    PCRGB pc_ell;
    //PCRGB head_temp(*pc_head);
    //head_temp.header.frame_id = "/base_link";
    projectEllipsoid(ell, ell.height, pc_head, pc_ell);
    marginalEllipsoid(pc_head, forces, fpc, pc_ell);
    //marginalEllipsoid(*pc_head, forces, fpc, head_temp);
    //pubLoop(head_temp, "/proj_head");

    int num_lat, num_lon, num_nbrs;
    double sigma;
    ros::param::param<int>("~num_lat", num_lat, 300);
    ros::param::param<int>("~num_lon", num_lon, 300);
    ros::param::param<int>("~num_nbrs", num_nbrs, 10);
    ros::param::param<double>("~sigma", sigma, 0.01);
    projectEllipsoidDense(ell, ell.height, pc_ell, num_lat, num_lon, sigma, num_nbrs);
}
