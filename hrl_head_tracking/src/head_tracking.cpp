
#include <hrl_head_tracking/head_tracking.h>

void extractSkinPC(const PCRGB::Ptr& pc_in, PCRGB::Ptr& pc_out, double thresh) 
{
    uint8_t r, g, b; 
    double skin_like, h, s, l;
    for(uint32_t i=0;i<pc_in->size();i++) {
        if(PT_IS_NOT_NAN(pc_in, i)) {
            extractRGB(pc_in->points[i].rgb, r, g, b);
            skin_like = skin_likelihood(r, g, b);
            if(skin_like > thresh) 
                COPY_PT_INTO_CLOUD(pc_in, pc_out, i);
        }
    }
}

uint32_t findClosestPoint(const PCRGB::Ptr& pc, uint32_t u, uint32_t v)
{
    if(PT_IS_NOT_NAN(pc, u*640 + v))
        return u*640 + v;
    for(uint32_t i=1;i<5;i++) {
        for(uint32_t j=1;j<5;j++) {
            if(PT_IS_NOT_NAN(pc, u*640 + v + i))
                return u*640 + v + i;
            if(PT_IS_NOT_NAN(pc, u*640 + v - i))
                return u*640 + v + i;
            if(PT_IS_NOT_NAN(pc, u*640 + v + j*640))
                return u*640 + v + i;
            if(PT_IS_NOT_NAN(pc, u*640 + v - j*640))
                return u*640 + v + i;
        }
    }
    return -1;
}

void sphereTrim(const PCRGB::Ptr& pc_in, PCRGB::Ptr& pc_out, uint32_t ind, double radius)
{
    KDTree::Ptr kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree->setInputCloud(pc_in);
    vector<int> inds;
    vector<float> dists;
    kd_tree->radiusSearch(*pc_in, ind, radius, inds, dists);
    for(uint32_t j=0;j<inds.size();j++)
        COPY_PT_INTO_CLOUD(pc_in, pc_out, inds[j]);
}

namespace pcl {

class ColorPointRepresentation : public PointRepresentation<PRGB> {
public:
    ColorPointRepresentation(float h_mult = 0.0005, float s_mult = 0.0005, float l_mult = 0.0005) {
        nr_dimensions_ = 6;
        alpha_.resize(6);
        alpha_[0] = 1; alpha_[1] = 1; alpha_[2] = 1;
        alpha_[3] = h_mult; alpha_[4] = s_mult; alpha_[5] = l_mult;
    }
    virtual void copyToFloatArray(const PRGB& p, float* out) const {
        double h, s, l;
        extractHSL(p.rgb, h, s, l);
        out[0] = p.x; out[1] = p.y; out[2] = p.z;
        out[3] = h; out[4] = s; out[5] = l;
    }
    //bool isValid<PRGB>(const PRGB& p) const {
    //    if(p.x == p.x && p.y == p.y && p.z == p.z)
    //        return true;
    //    else
    //        return false;
    //}
};
}

void computeICPRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                            Eigen::Affine3d& tf_mat, int max_iters, double color_weight) 
                            {
    pcl::IterativeClosestPointNonLinear<PRGB, PRGB> icp;
    boost::shared_ptr<pcl::PointRepresentation<PRGB> const> pt_rep(new pcl::ColorPointRepresentation(color_weight, color_weight, color_weight));
    icp.setPointRepresentation(pt_rep);
    icp.setInputTarget(target_pc);
    icp.setInputCloud(source_pc);
    icp.setMaximumIterations(max_iters);
    PCRGB tmp_pc;
    icp.align(tmp_pc);
    tf_mat = icp.getFinalTransformation().cast<double>();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking");
    ros::NodeHandle nh;
    PCRGB::Ptr input_pc, template_pc;
    readPCBag(argv[1], input_pc);
    PCRGB::Ptr skin_pc(new PCRGB());
    PCRGB::Ptr expanded_pc(new PCRGB());
    PCRGB::Ptr trimmed_pc(new PCRGB());
    PCRGB::Ptr tf_pc(new PCRGB());

    uint32_t closest_ind = findClosestPoint(input_pc, atoi(argv[3]), atoi(argv[4]));
    sphereTrim(input_pc, trimmed_pc, closest_ind, atof(argv[5]));
    extractSkinPC(trimmed_pc, skin_pc, atof(argv[2]));

    Eigen::Affine3d tf_mat;
    readPCBag(argv[6], template_pc);
    computeICPRegistration(template_pc, skin_pc, tf_mat, 100, 0);

    transformPC(*skin_pc, *tf_pc, tf_mat);
    tf_pc->header.frame_id = "/base_link";
    pubLoop(tf_pc, "test", 5);
    return 0;
}
