#include <hrl_phri_2011/pc_utils.h>

void findTransformFromOverlapICP(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                                 Eigen::eigen2_Transform3d& tf_mat) {
    KDTree::Ptr kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree->setInputCloud(target_pc);
    vector<bool> target_inds(target_pc->points.size(), false);
    vector<bool> source_inds(source_pc->points.size(), false);
    for(uint32_t i=0;i<source_pc->points.size();i++) {
        vector<int> inds;
        vector<float> dists;
        kd_tree->radiusSearch(*source_pc, i, 0.015, inds, dists);
        for(uint32_t j=0;j<inds.size();j++) 
            target_inds[inds[j]] = true;
        if(inds.size() > 0)
            source_inds[i] = true;
    }
    pcl::IndicesPtr target_inds_list(new vector<int>()), source_inds_list(new vector<int>());
    for(uint32_t i=0;i<target_pc->points.size();i++) 
        if(target_inds[i])
            target_inds_list->push_back(i);
    for(uint32_t i=0;i<source_pc->points.size();i++) 
        if(source_inds[i])
            source_inds_list->push_back(i);
    pcl::ExtractIndices<PRGB> extract_inds;
    PCRGB::Ptr target_pc_ol(new PCRGB());
    PCRGB::Ptr source_pc_ol(new PCRGB());
    extract_inds.setInputCloud(target_pc);
    extract_inds.setIndices(target_inds_list);
    extract_inds.filter(*target_pc_ol);
    extract_inds.setInputCloud(source_pc);
    extract_inds.setIndices(source_inds_list);
    extract_inds.filter(*source_pc_ol);

    computeICPRegistration(target_pc_ol, source_pc_ol, tf_mat);
}

void stitchPCs(std::vector<PCC::Ptr> &captures) {
    vector<PCRGB::Ptr> pc_list;
    BOOST_FOREACH(PCC::Ptr pcc, captures) {
        map_str_t3d cap_poses;
        for(uint32_t i=0;i<pcc->frame_names.size();i++) 
            tf::poseMsgToEigen(pcc->saved_frames[i].pose, cap_poses[pcc->frame_names[i]]);

        PCRGB pc_raw, pc_trans, pc_filtered;
        pcl::fromROSMsg(pcc->pc_capture, pc_raw);
        std::vector<int> inds;
        pcl::removeNaNFromPointCloud<PRGB>(pc_raw, pc_raw, inds);

        Eigen::eigen2_Transform3d pc_tf(cap_poses["/head_center"].inverse() * cap_poses["/openni_rgb_optical_frame"]);
        transformPC(pc_raw, pc_trans, pc_tf);

        boxFilter(pc_trans, pc_filtered, -0.22, 0.22, -0.15, 0.15, -0.2, 0.25);

        pc_list.push_back(PCRGB::Ptr(new PCRGB(pc_filtered)));
    }
    int num_pc = (int) pc_list.size();
    PCRGB pc_combo;
    pc_combo += *pc_list[num_pc/2];
    for(int i=num_pc/2 - 1;i>=0;i--) {
        PCRGB::Ptr source_tf_pc(new PCRGB());
        Eigen::eigen2_Transform3d tf_mat;
        findTransformFromOverlapICP(pc_combo.makeShared(), pc_list[i], tf_mat);
        transformPC(*pc_list[i], *source_tf_pc, tf_mat);
        pc_combo += *source_tf_pc;
        ROS_INFO("Stitched cloud %d", i);
    }
    for(int i=num_pc/2 + 1;i<num_pc;i++) {
        PCRGB::Ptr source_tf_pc(new PCRGB());
        Eigen::eigen2_Transform3d tf_mat;
        findTransformFromOverlapICP(pc_combo.makeShared(), pc_list[i], tf_mat);
        transformPC(*pc_list[i], *source_tf_pc, tf_mat);
        pc_combo += *source_tf_pc;
        ROS_INFO("Stitched cloud %d", i);
    }
    pubLoop(pc_combo, "/stitched_head");
    /*
    PCRGB pc_downsampled;
    pcl::VoxelGrid<PRGB> vox_grid;
    vox_grid.setInputCloud(pc_combo.makeShared());
    vox_grid.setLeafSize(0.0025, 0.0025, 0.0025);
    vox_grid.filter(pc_downsampled);
    pubLoop(pc_downsampled);
    */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_head_stitcher");

    // Load bag
    rosbag::Bag bag;
    bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/point_cloud_captures"));
    std::vector<PCC::Ptr> captures;
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        captures.push_back(m.instantiate<PCC>());
    }

    stitchPCs(captures);

    return 0;
}
