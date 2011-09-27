#include <limits>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <hrl_phri_2011/PHRIPointCloudCapture.h>
#include <hrl_phri_2011/pcl_features.h>
#include <hrl_phri_2011/utils.h>

typedef hrl_phri_2011::PHRIPointCloudCapture PCC;
typedef map<string, Eigen::Affine3d, less<string>, 
            Eigen::aligned_allocator<std::pair<const string, Eigen::Affine3d> > > map_str_t3d;

void findOverlaps(const PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc,
                  pcl::IndicesPtr& target_inds_list, pcl::IndicesPtr& source_inds_list, 
                  double icp_radius=0.03, bool use_rgb=false, double color_weight=0.001) 
                  {
    KDTree::Ptr kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    kd_tree->setInputCloud(target_pc);
    if(use_rgb){
        boost::shared_ptr<pcl::DefaultPointRepresentation<PRGB> const> 
            pt_rep(new pcl::DefaultPointRepresentation<PRGB>(color_weight, color_weight, color_weight));
        kd_tree->setPointRepresentation(pt_rep);
    }
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
                                 Eigen::Affine3d& tf_mat) 
                                 {
    int icp_iters, use_rgb;
    double color_weight, icp_radius;
    ros::param::param<int>("~icp_iters", icp_iters, 10);
    ros::param::param<double>("~color_weight", color_weight, 0.0005);
    ros::param::param<double>("~icp_radius", icp_radius, 0.05);
    ros::param::param<int>("~use_rgb", use_rgb, 1);
    ROS_INFO("PS %d %f %f", icp_iters, color_weight, icp_radius);

    pcl::IndicesPtr target_inds_list(new vector<int>()), source_inds_list(new vector<int>());
    findOverlaps(target_pc, source_pc, target_inds_list, source_inds_list, icp_radius, use_rgb, color_weight);
    ROS_INFO("INDS %d %d", target_inds_list->size(), source_inds_list->size());
    PCRGB::Ptr target_pc_ol(new PCRGB());
    PCRGB::Ptr source_pc_ol(new PCRGB());
    extractIndices(target_pc, target_inds_list, target_pc_ol);
    extractIndices(source_pc, source_inds_list, source_pc_ol);
    ROS_INFO("PC COUNT %d %d", target_pc_ol->points.size(), source_pc_ol->points.size());
    computeICPRegistration(target_pc_ol, source_pc_ol, tf_mat, icp_iters, color_weight);
}

void erodePC(const PCRGB::Ptr& target_pc, const vector<int>& inds, PCRGB::Ptr& eroded_pc) 
{
    cv::Mat pc_img = cv::Mat::zeros(640, 480, CV_32F);
    cv::Mat pc_img_eroded = cv::Mat::zeros(640, 480, CV_32F);
    for(uint32_t i=0;i<inds.size();i++) 
        if(target_pc->points[i].x == target_pc->points[i].x &&
           target_pc->points[i].y == target_pc->points[i].y &&
           target_pc->points[i].z == target_pc->points[i].z) {
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

void extractPCs(const vector<PCC::Ptr> & captures, vector<PCRGB::Ptr>& pc_list) 
{
    double z_cutoff;
    ros::param::param<double>("~z_cutoff", z_cutoff, -0.2);
    BOOST_FOREACH(PCC::Ptr pcc, captures) {
        map_str_t3d cap_poses;
        for(uint32_t i=0;i<pcc->frame_names.size();i++) 
            tf::poseMsgToEigen(pcc->saved_frames[i].pose, cap_poses[pcc->frame_names[i]]);

        PCRGB pc_raw, pc_trans, pc_filtered;
        pcl::fromROSMsg(pcc->pc_capture, pc_raw);
        std::vector<int> inds;

        Eigen::Affine3d pc_tf(cap_poses["/head_center"].inverse() * cap_poses["/openni_rgb_optical_frame"]);
        transformPC(pc_raw, pc_trans, pc_tf);

        for(size_t i=0;i<pc_trans.points.size();i++) {
            const PRGB p = pc_trans.points[i];
            PRGB new_p;
            if(p.x == p.x && p.y == p.y && p.z == p.z &&
               p.x > -0.22 && p.x < 0.22 && p.y > -0.15 && p.y < 0.15 && p.z > z_cutoff && p.z < 0.25) {
                new_p.x = p.x; new_p.y = p.y; new_p.z = p.z; new_p.rgb = p.rgb;
                inds.push_back(i);
                pc_filtered.points.push_back(new_p);
            } else {
                new_p.x = numeric_limits<float>::quiet_NaN();
                new_p.y = numeric_limits<float>::quiet_NaN();
                new_p.z = numeric_limits<float>::quiet_NaN();
            }
        }
        //boxFilter(pc_trans, pc_filtered, -0.22, 0.22, -0.15, 0.15, -0.2, 0.25);

        //pc_list.push_back(PCRGB::Ptr(new PCRGB(pc_filtered)));
        PCRGB::Ptr pc_eroded(new PCRGB());
        erodePC(pc_filtered.makeShared(), inds, pc_eroded);
        //pc_eroded->header.frame_id = "/head_center";
        //pubLoop(*pc_eroded, "/stitched_head");

        pc_list.push_back(pc_eroded);
    }
}

void concatRegisteredPC(PCRGB::Ptr& target_pc, const PCRGB::Ptr& source_pc, double keep_radius) 
{
    PCRGB::Ptr source_tf_pc(new PCRGB()), source_filtered_pc(new PCRGB());
    Eigen::Affine3d tf_mat;
    findTransformFromOverlapICP(target_pc, source_pc, tf_mat);
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

void stitchPCs(std::vector<PCC::Ptr> &captures, double keep_radius) 
{
    vector<PCRGB::Ptr> pc_list;
    extractPCs(captures, pc_list);
    int num_pc = (int) pc_list.size();
    PCRGB::Ptr pc_combo(new PCRGB());
    pc_combo->operator+=(*pc_list[0]);
    ROS_INFO("Begin Stitching");
    for(int i=1;i<=num_pc/2-1;i++) {
        concatRegisteredPC(pc_combo, pc_list[i], keep_radius);
        ROS_INFO("Stitched cloud %d", i);
    }
    for(int i=num_pc-1;i>=num_pc/2;i--) {
        concatRegisteredPC(pc_combo, pc_list[i], keep_radius);
        ROS_INFO("Stitched cloud %d", i);
    }
    pc_combo->header.frame_id = "/head_center";
    ROS_INFO("Publishing PC to topic /stitched_head");
    pubLoop(*pc_combo, "/stitched_head");
}

void displayRawPCs(std::vector<PCC::Ptr> &captures) 
{
    vector<PCRGB::Ptr> pc_list;
    extractPCs(captures, pc_list);
    PCRGB pc_combo;
    BOOST_FOREACH(PCRGB::Ptr const pc, pc_list) {
        pc_combo += *pc;
    }
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
    double keep_radius;
    ros::NodeHandle nh_priv("~");
    nh_priv.param<double>("keep_radius", keep_radius, 0.01);

    if(argc == 2)
        stitchPCs(captures, keep_radius);
    if(argc == 3 && argv[2][0] == 'd') 
        displayRawPCs(captures);

    return 0;
}
