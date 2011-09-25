#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/pcl_features.h>

typedef Eigen::Matrix<double, 6, 1> CartVec;

void wrenchMsgToEigen(const geometry_msgs::Wrench& wrench_msg, CartVec& wrench_eig) 
{
    wrench_eig(0, 0) = wrench_msg.force.x;
    wrench_eig(1, 0) = wrench_msg.force.y;
    wrench_eig(2, 0) = wrench_msg.force.z;
    wrench_eig(3, 0) = wrench_msg.torque.x;
    wrench_eig(4, 0) = wrench_msg.torque.y;
    wrench_eig(5, 0) = wrench_msg.torque.z;
}

#define NORM(x, y, z) ( std::sqrt( (x) * (x) + (y) * (y) + (z) * (z) ) )
#define SQ(x) ( (x) * (x) )
#define NORMAL(x, sig) ( std::exp( - (x) * (x) / (2.0 * (sig) * (sig))) / std::sqrt(2.0 * 3.14159 * (sig) * (sig)))

class ForceVisualizer 
{
private:
    ros::Subscriber wrench_sub;
    tf::TransformListener tf_list;
    ros::NodeHandle nh;
    PCRGB::Ptr pc_head;
    KDTree::Ptr kd_tree;
    PCNormals::Ptr normals;
    int pub_ind;
    vector<double> force_densities;
    vector<double> force_high;
    double force_density_sum, force_high_sum, start_time, force_contact_thresh;
    double time_contact_thresh, time_start_contact;
    int last_start_ind, contact_period;
    string region, user;
    vector<hrl_phri_2011::ForceProcessed> fp_list;
    rosbag::Bag processed_bag;

    void wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_stamped) 
    {
        double cur_time = wrench_stamped->header.stamp.toSec();
        if(start_time == -1)
            start_time = cur_time;
        CartVec w;
        wrenchMsgToEigen(wrench_stamped->wrench, w);
        double force_mag = NORM(w(0, 0), w(1, 0), w(2, 0));
        if(force_mag < force_contact_thresh) {
            if(cur_time - time_start_contact < time_contact_thresh) {
                // erase prev entries
                for(int i=0;i<pub_ind - last_start_ind;i++) 
                    fp_list.pop_back();
            } else {
                // end this contact period
                contact_period++;
            }
            last_start_ind = -1;
            return;
        }
        // we are probably making contact
        if(last_start_ind == -1) {
            // just starting contact
            last_start_ind = pub_ind;
            time_start_contact = cur_time;
        }
        tf::StampedTransform tool_loc_tf;
        try {
            tf_list.waitForTransform("/head_center", "/wipe_finger", wrench_stamped->header.stamp, ros::Duration(3.0));
            tf_list.lookupTransform("/head_center", "/wipe_finger", wrench_stamped->header.stamp, tool_loc_tf);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        btVector3 tool_loc = tool_loc_tf.getOrigin();
        PRGB query_pt;
        query_pt.x = tool_loc.x(); query_pt.y = tool_loc.y(); query_pt.z = tool_loc.z(); 
        vector<int> nk_inds(1);
        vector<float> nk_dists(1);
        kd_tree->nearestKSearch(query_pt, 1, nk_inds, nk_dists);
        int closest_ind = nk_inds[0];

        hrl_phri_2011::ForceProcessed fp;
        fp.time_offset = cur_time - start_time;
        tf::transformStampedTFToMsg(tool_loc_tf, fp.tool_frame);
        fp.header = wrench_stamped->header;
        fp.header.frame_id = "/head_center";
        fp.wrench = wrench_stamped->wrench;
        fp.pc_ind = closest_ind;
        fp.pc_normal.x = normals->points[closest_ind].normal[0];
        fp.pc_normal.y = normals->points[closest_ind].normal[1];
        fp.pc_normal.z = normals->points[closest_ind].normal[2];
        fp.force_magnitude = force_mag;
        fp.force_normal = fp.pc_normal.x * fp.wrench.force.x + 
                          fp.pc_normal.y * fp.wrench.force.y + 
                          fp.pc_normal.z * fp.wrench.force.z;
        fp.force_tangental = sqrt(SQ(fp.force_magnitude) - SQ(fp.force_normal));
        fp.region = region;
        fp.user = user;
        fp.contact_period = contact_period;
        fp.time_from_contact_start = cur_time - time_start_contact;
        fp_list.push_back(fp);
        pub_ind++;
        if(pub_ind % 100 == 0)
            ROS_INFO("Recorded %d samples", pub_ind);
    }

public:
    void startVisualization() 
    {
        wrench_sub = nh.subscribe("/tool_netft_zeroer/wrench_zeroed", 100, &ForceVisualizer::wrenchCallback, this);
    }

    ForceVisualizer(PCRGB::Ptr& pch, const string& pbag_name) : 
        pc_head(pch),
        kd_tree(new pcl::KdTreeFLANN<PRGB> ()),
        normals(new PCNormals()),
        pub_ind(0),
        force_densities(pch->points.size()),
        force_high(pch->points.size()),
        force_density_sum(0.0),
        force_high_sum(0.0),
        start_time(-1),
        last_start_ind(-1),
        contact_period(0)
    {
        kd_tree->setInputCloud(pc_head);
        computeNormals(pc_head, kd_tree, normals);
        pc_head->header.frame_id = "/head_center";
        processed_bag.open(pbag_name, rosbag::bagmode::Write);
        ros::param::param<double>("/force_contact_thresh", force_contact_thresh, 0.2);
        ros::param::param<double>("/time_contact_thresh", force_contact_thresh, 0.2);
        ros::Duration(1.0).sleep();
    }

    void writeBag() 
    {
        BOOST_FOREACH(const hrl_phri_2011::ForceProcessed& fp, fp_list) {
            processed_bag.write("/force_processed", fp.header.stamp, fp);
        }
        processed_bag.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_extractor");

    rosbag::Bag pc_bag, force_bag;
    PCRGB::Ptr pc_head(new PCRGB);
    // Load PC bag
    pc_bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View pc_view(pc_bag, rosbag::TopicQuery("/stitched_head"));
    BOOST_FOREACH(rosbag::MessageInstance const m, pc_view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
    }

    ForceVisualizer fv(pc_head, argv[2]);
    fv.startVisualization();
    ROS_INFO("Ready for collection");
    ros::spin();
    fv.writeBag();
}
