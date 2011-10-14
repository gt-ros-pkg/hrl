#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>

#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/pcl_basic.h>

#define SE(x, sig) ( std::exp( - (x) / (2.0 * (sig) * (sig))) / (sig) * (sig))

void colorizeDataPC(const PCRGB& data_pc, PCRGB& color_pc, 
                    double saturation=100, double lightness=50, bool use_min=true)
{
    int use_min_;
    ros::param::param<int>("~use_min", use_min_, 1);
    use_min = use_min_;
    vector<float> data;
    for(size_t i=0;i<data_pc.size();i++) 
        data.push_back(data_pc.points[i].rgb);

    float max_val = *std::max_element(data.begin(), data.end());
    float min_val = *std::min_element(data.begin(), data.end());
    ROS_INFO("Max data value: %f, Min data_value: %f", max_val, min_val);
    double h;
    for(size_t i=0;i<data.size();i++) {
        PRGB pt;
        pt.x = data_pc.points[i].x;
        pt.y = data_pc.points[i].y;
        pt.z = data_pc.points[i].z;
        if(use_min)
            h = (double) 240.0 * (data[i] - min_val) / (max_val - min_val);
        else
            h = (double) 240.0 * data[i] / max_val;
        if(h < 0) h = 0; if(h > 240.0) h = 240.0;
        writeHSL(240.0 - h, saturation, lightness, pt.rgb);
        color_pc.push_back(pt);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "density_estimation");
    ros::NodeHandle nh;

    double target_force, pilot_ph, pilot_fh, percent_trim;
    ros::param::param<double>("~target_force", target_force, 2);
    ros::param::param<double>("~pilot_ph", pilot_ph, 0.02);
    ros::param::param<double>("~pilot_fh", pilot_fh, 0.5);
    ros::param::param<double>("~percent_trim", percent_trim, 0.2);

    // Load PC bag
    PCRGB::Ptr head_pc;
    vector<PCRGB::Ptr> data_pcs;
    pcl::KdTreeFLANN<PRGB> head_kd_tree(new pcl::KdTreeFLANN<PRGB> ());
    vector<pcl::KdTreeFLANN<PRGB>::Ptr > data_kd_trees;

    vector<PCRGB::Ptr> pc_list;
    //readBagTopic<PCRGB>(argv[1], pc_list, "/stitched_head");
    readBagTopic<PCRGB>(argv[1], pc_list, "/data_cloud");
    head_pc = pc_list[0];
    head_kd_tree.setInputCloud(head_pc);

    for(int i=2;i<argc-1;i++) {
        pc_list.clear();
        readBagTopic<PCRGB>(argv[i], pc_list, "/data_cloud");
        data_pcs.push_back(pc_list[0]);
        pcl::KdTreeFLANN<PRGB>::Ptr new_kd_tree(new pcl::KdTreeFLANN<PRGB> ());
        new_kd_tree->setInputCloud(pc_list[0]);
        data_kd_trees.push_back(new_kd_tree);
    }
    

    PCRGB joint_den, pos_den, marginal_den, expected_val, force_variance;
    vector<int> inds; vector<float> dists;
    double cur_force;
    double joint_cur_dist, joint_kern_val, joint_kern_sum = 0.0;
    double pos_cur_dist, pos_kern_val, pos_kern_sum = 0.0, max_pos_kern = 0, cur_pos_kern_val;
    double marginal_kern_sum = 0.0;
    double exp_val;
    double force_var, cur_force_err;
    PRGB pt;
    for(size_t i=0;i<head_pc->size();i++) {
        joint_kern_val = 0;
        pos_kern_val = 0;
        exp_val = 0;
        force_var = 0;
        pt.x = head_pc->points[i].x;
        pt.y = head_pc->points[i].y;
        pt.z = head_pc->points[i].z;
        for(size_t k=0;k<data_kd_trees.size();k++) {
            data_kd_trees[k]->radiusSearch(*head_pc, i, pilot_ph * 3, inds, dists);
            if(dists.size() != 0) {
                double user_joint_kern_val = 0, user_pos_kern_val = 0, user_exp_val = 0, user_force_var = 0;
                for(size_t j=0;j<dists.size();j++) {
                    cur_force = data_pcs[k]->points[inds[j]].rgb;
                    joint_cur_dist = dists[j] / SQ(pilot_ph) + 
                               SQ(target_force - cur_force) / SQ(pilot_fh);
                    pos_cur_dist = dists[j] / SQ(pilot_ph);
                    user_joint_kern_val += exp(- 0.5 * joint_cur_dist) / (SQ(pilot_ph) * pilot_ph * pilot_fh);
                    cur_pos_kern_val = exp(- 0.5 * pos_cur_dist) / (SQ(pilot_ph) * pilot_ph);
                    user_pos_kern_val += cur_pos_kern_val;
                    user_exp_val += cur_pos_kern_val * cur_force;
                }
                if(pos_kern_val > 0.01) {
                    for(size_t j=0;j<dists.size();j++) {
                        cur_force = data_pcs[k]->points[inds[j]].rgb;
                        cur_force_err = cur_force - exp_val / pos_kern_val;
                        user_force_var += pos_kern_val * SQ(cur_force_err);
                    }
                }
                joint_kern_val += user_joint_kern_val / dists.size();
                pos_kern_val += user_pos_kern_val / dists.size();
                exp_val += user_exp_val / dists.size();
                force_var += user_force_var / dists.size();
                inds.clear(); dists.clear();
            }
        }

        // joint density
        pt.rgb = joint_kern_val;
        joint_den.push_back(pt);
        joint_kern_sum += joint_kern_val;

        // position density
        if(pos_kern_val > 0.01) {
            pt.rgb = pos_kern_val;
            pos_kern_sum += pos_kern_val;
            max_pos_kern = max(max_pos_kern, pos_kern_val);
        }
        pos_den.push_back(pt);

        // marginal density
        if(pos_kern_val > 0.01) {
            pt.rgb = joint_kern_val / pos_kern_val;
            marginal_kern_sum += joint_kern_val / pos_kern_val;
        }
        else
            pt.rgb = 0;
        marginal_den.push_back(pt);

        // expected value
        if(pos_kern_val > 0.01) {
            pt.rgb = exp_val / pos_kern_val;
        }
        else
            pt.rgb = 0;
        expected_val.push_back(pt);

        // variance
        if(pos_kern_val > 0.01) {
            pt.rgb = sqrt(force_var / pos_kern_val);
        }
        else
            pt.rgb = 0;
        force_variance.push_back(pt);
    }

    for(size_t i=0;i<pos_den.size();i++) {
        // remove low position density artifacts
        if(pos_den.points[i].rgb < max_pos_kern * percent_trim) {
            pos_den.points[i].rgb = 0;
            marginal_den.points[i].rgb = 0;
            expected_val.points[i].rgb = 0;
            force_variance.points[i].rgb = 0;
        }
    }

    PCRGB trimmed_joint_den, trimmed_pos_den, trimmed_marginal_den, trimmed_expected_val, trimmed_force_variance;
    for(size_t i=0;i<head_pc->size();i++) {
        if(joint_den.points[i].rgb != 0)
            trimmed_joint_den.push_back(joint_den.points[i]);
        if(pos_den.points[i].rgb != 0)
            trimmed_pos_den.push_back(pos_den.points[i]);
        if(marginal_den.points[i].rgb != 0)
            trimmed_marginal_den.push_back(marginal_den.points[i]);
        if(expected_val.points[i].rgb != 0)
            trimmed_expected_val.push_back(expected_val.points[i]);
        if(force_variance.points[i].rgb != 0)
            trimmed_force_variance.push_back(force_variance.points[i]);
    }
    joint_den = trimmed_joint_den;
    pos_den = trimmed_pos_den;
    marginal_den = trimmed_marginal_den;
    expected_val = trimmed_expected_val;
    force_variance = trimmed_force_variance;

    vector<PCRGB::Ptr> pcs;
    vector<string> topics;
    PCRGB color_joint_den, color_pos_den, color_marginal_den, color_expected_val, color_force_variance;

    colorizeDataPC(joint_den, color_joint_den);
    color_joint_den.header.frame_id = "/base_link";
    pcs.push_back(color_joint_den.makeShared()); topics.push_back("/joint_density");

    colorizeDataPC(pos_den, color_pos_den);
    color_pos_den.header.frame_id = "/base_link";
    pcs.push_back(color_pos_den.makeShared()); topics.push_back("/position_density");

    colorizeDataPC(marginal_den, color_marginal_den);
    color_marginal_den.header.frame_id = "/base_link";
    pcs.push_back(color_marginal_den.makeShared()); topics.push_back("/marginal_density");

    colorizeDataPC(expected_val, color_expected_val);
    color_expected_val.header.frame_id = "/base_link";
    pcs.push_back(color_expected_val.makeShared()); topics.push_back("/expected_value");

    colorizeDataPC(force_variance, color_force_variance);
    color_force_variance.header.frame_id = "/base_link";
    pcs.push_back(color_force_variance.makeShared()); topics.push_back("/force_variance");

    pubLoop(pcs, topics, 1);
    //pubLoop(color_pilot_den, "/function_mean", 1);
    return 0;

    rosbag::Bag bag;
    bag.open(argv[argc-1], rosbag::bagmode::Write);
    bag.write("/data_cloud", ros::Time::now(), joint_den);
    bag.close();
}
