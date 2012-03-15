#include<hrl_phri_2011/utils.h>


void applyRegistration(const PCRGB& in_pc, const hrl_phri_2011::EllipsoidParams& params, PCRGB& out_pc)
{
    PCRGB pc_tf;
    tf::Transform tf_tf;
    tf::transformMsgToTF(params.e_frame.transform, tf_tf);
    Eigen::Affine3d tf_eigen;
    tf::TransformTFToEigen(tf_tf, tf_eigen);
    tf_eigen = tf_eigen.inverse();
    transformPC(in_pc, out_pc, tf_eigen);
}

void loadRegisteredHead(const string& head_bag, const string& params_bag, PCRGB& out_pc, Ellipsoid& ell)
{
    // Load PC bag
    vector<PCRGB::Ptr> pc_list;
    readBagTopic<PCRGB>(head_bag, pc_list, "/stitched_head");

    // load params
    vector<hrl_phri_2011::EllipsoidParams::Ptr> params;
    readBagTopic<hrl_phri_2011::EllipsoidParams>(params_bag, params, "/ellipsoid_params");

    applyRegistration(*pc_list[0], *params[0], out_pc);
    ell.setParams(*params[0]);
}
