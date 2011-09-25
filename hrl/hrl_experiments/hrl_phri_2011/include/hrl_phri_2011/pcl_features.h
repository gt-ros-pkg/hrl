#ifndef PCL_FEATURES_H
#define PCL_FEATURES_H
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>

typedef pcl::KdTree<PRGB> KDTree;
typedef pcl::PointCloud<pcl::Normal> PCNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFeatures;

void computeNormals(const PCRGB::Ptr& in_pc, const KDTree::Ptr& in_kd_tree, PCNormals::Ptr& out_normals);
void computeFPFH(const PCRGB::Ptr& in_pc, FPFeatures::Ptr& out_features);
void computeSACRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc);
void computeICPRegistration(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                            Eigen::Affine3d& tf_mat, int max_iters=300, double color_weight=0.0005);

namespace pcl {

template <>
class DefaultPointRepresentation<PRGB> : public PointRepresentation<PRGB> {
public:
    DefaultPointRepresentation(float h_mult = 0.0005, float s_mult = 0.0005, float l_mult = 0.0005) {
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
    /*bool isValid<PRGB>(const PRGB& p) const {
        if(p.x == p.x && p.y == p.y && p.z == p.z)
            return true;
        else
            return false;
    }*/
};
}

#endif // PCL_FEATURES_H
