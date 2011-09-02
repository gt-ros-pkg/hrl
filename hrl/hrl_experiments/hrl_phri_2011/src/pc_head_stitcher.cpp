#include <limits>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <hrl_phri_2011/pc_utils.h>

void findOverlaps(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                  pcl::IndicesPtr& target_inds_list, pcl::IndicesPtr& source_inds_list, 
                  double icp_radius=0.03) {
    KDTree::Ptr kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree->setInputCloud(target_pc);
    vector<bool> target_inds(target_pc->points.size(), false);
    vector<bool> source_inds(source_pc->points.size(), false);
    for(uint32_t i=0;i<source_pc->points.size();i++) {
        vector<int> inds;
        vector<float> dists;
        kd_tree->radiusSearch(*source_pc, i, icp_radius, inds, dists);
        for(uint32_t j=0;j<inds.size();j++) 
            target_inds[inds[j]] = true;
        if(inds.size() > 0)
            source_inds[i] = true;
    }
    for(uint32_t i=0;i<target_pc->points.size();i++) 
        if(target_inds[i])
            target_inds_list->push_back(i);
    for(uint32_t i=0;i<source_pc->points.size();i++) 
        if(source_inds[i])
            source_inds_list->push_back(i);
}

void findTransformFromOverlapICP(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                                 Eigen::eigen2_Transform3d& tf_mat, double icp_radius) {
    pcl::IndicesPtr target_inds_list(new vector<int>()), source_inds_list(new vector<int>());
    findOverlaps(target_pc, source_pc, target_inds_list, source_inds_list, icp_radius);
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

void erodePC(const PCRGB::Ptr& target_pc, const vector<int>& inds, PCRGB::Ptr& eroded_pc) {
    cv::Mat pc_img = cv::Mat::zeros(640, 480, CV_32F);
    cv::Mat pc_img_eroded = cv::Mat::zeros(640, 480, CV_32F);
    for(uint32_t i=0;i<inds.size();i++) 
        if(target_pc->points[inds[i]].x == target_pc->points[inds[i]].x &&
           target_pc->points[inds[i]].y == target_pc->points[inds[i]].y &&
           target_pc->points[inds[i]].z == target_pc->points[inds[i]].z) {
            pc_img.at<uint8_t>(inds[i] % 640, inds[i] / 640) = 1.0;
        }

    IplImage pc_img_ipl = pc_img, pc_img_eroded_ipl = pc_img_eroded;
    int num_erode;
    ros::NodeHandle nh_priv("~");
    nh_priv.param<int>("num_erode", num_erode, 2);
    if(num_erode > 0)
        cvErode(&pc_img_ipl, &pc_img_eroded_ipl, NULL, num_erode);
    else
        pc_img_eroded_ipl = pc_img_ipl;
    pcl::IndicesPtr inds_erode(new vector<int>());
    for(uint32_t i=0;i<inds.size();i++) 
        if(pc_img_eroded.at<uint8_t>(inds[i] % 640, inds[i] / 640) == 1.0)
            inds_erode->push_back(i);

    pcl::ExtractIndices<PRGB> extract_inds;
    extract_inds.setInputCloud(target_pc);
    extract_inds.setIndices(inds_erode);
    extract_inds.filter(*eroded_pc);
}

void extractPCs(const vector<PCC::Ptr> & captures, vector<PCRGB::Ptr>& pc_list) {
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

        BOOST_FOREACH(PRGB const p, pc_trans.points) {
            PRGB new_p;
            if(p.x == p.x && p.y == p.y && p.z == p.z &&
               p.x > -0.22 && p.x < 0.22 && p.y > -0.15 && p.y < 0.15 && p.z > -0.2 && p.z < 0.25) {
                new_p.x = p.x; new_p.y = p.y; new_p.z = p.z; new_p.rgb = p.rgb;
            } else {
                new_p.x = numeric_limits<float>::quiet_NaN();
                new_p.y = numeric_limits<float>::quiet_NaN();
                new_p.z = numeric_limits<float>::quiet_NaN();
            }
            pc_filtered.points.push_back(new_p);
        }
        //boxFilter(pc_trans, pc_filtered, -0.22, 0.22, -0.15, 0.15, -0.2, 0.25);

        //pc_list.push_back(PCRGB::Ptr(new PCRGB(pc_filtered)));
        PCRGB::Ptr pc_eroded(new PCRGB());
        erodePC(pc_filtered.makeShared(), inds, pc_eroded);

        pc_list.push_back(pc_eroded);
    }
}

void concatRegisteredPC(PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc, 
                        double icp_radius, double keep_radius) {
    PCRGB::Ptr source_tf_pc(new PCRGB()), source_filtered_pc(new PCRGB());
    Eigen::eigen2_Transform3d tf_mat;
    findTransformFromOverlapICP(target_pc, source_pc, tf_mat, icp_radius);
    transformPC(*source_pc, *source_tf_pc, tf_mat);

    pcl::IndicesPtr target_inds_list(new vector<int>()), source_inds_list(new vector<int>());
    findOverlaps(target_pc, source_tf_pc, target_inds_list, source_inds_list, keep_radius);
    pcl::ExtractIndices<PRGB> extract_inds;
    extract_inds.setNegative(true);
    extract_inds.setInputCloud(source_tf_pc);
    extract_inds.setIndices(source_inds_list);
    extract_inds.filter(*source_filtered_pc);
    target_pc->operator+=(*source_filtered_pc);
}

void stitchPCs(std::vector<PCC::Ptr> &captures, double icp_radius, double keep_radius) {
    vector<PCRGB::Ptr> pc_list;
    extractPCs(captures, pc_list);
    int num_pc = (int) pc_list.size();
    PCRGB::Ptr pc_combo(new PCRGB());
    pc_combo->operator+=(*pc_list[0]);
    ROS_INFO("Begin Stitching");
    for(int i=1;i<=num_pc/2-1;i++) {
        concatRegisteredPC(pc_combo, pc_list[i], icp_radius, keep_radius);
        ROS_INFO("Stitched cloud %d", i);
    }
    for(int i=num_pc-1;i>=num_pc/2;i--) {
        concatRegisteredPC(pc_combo, pc_list[i], icp_radius, keep_radius);
        ROS_INFO("Stitched cloud %d", i);
    }
    pc_combo->header.frame_id = "/head_center";
    ROS_INFO("Publishing PC to topic /stitched_head");
    pubLoop(*pc_combo, "/stitched_head");
}

void displayRawPCs(std::vector<PCC::Ptr> &captures) {
    vector<PCRGB::Ptr> pc_list;
    extractPCs(captures, pc_list);
    PCRGB pc_combo;
    BOOST_FOREACH(PCRGB::Ptr const pc, pc_list)
        pc_combo += *pc;
    pc_combo.header.frame_id = "/head_center";
    pubLoop(pc_combo, "/stitched_head");
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
    double icp_radius, keep_radius;
    ros::NodeHandle nh_priv("~");
    nh_priv.param<double>("icp_radius", icp_radius, 0.03);
    nh_priv.param<double>("keep_radius", keep_radius, 0.01);

    if(argc == 2)
        stitchPCs(captures, icp_radius, keep_radius);
    if(argc == 3 && argv[2][0] == 'd') 
        displayRawPCs(captures);

    return 0;
}
