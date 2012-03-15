#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/pcl_features.h>
#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ellipsoid_space.h>

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
#ifndef SQ
#define SQ(x) ( (x) * (x) )
#endif
#define NORMAL(x, sig) ( std::exp( - (x) * (x) / (2.0 * (sig) * (sig))) / std::sqrt(2.0 * 3.14159 * (sig) * (sig)))

class DataExtractor 
{
private:
    ros::Subscriber wrench_sub;
    tf::TransformListener tf_list;
    ros::NodeHandle nh;
    PCRGB::Ptr pc_head;
    KDTree::Ptr kd_tree;
    PCNormals::Ptr normals;
    int pub_ind;
    double start_time;
    vector<hrl_phri_2011::ForceProcessed> fp_list;
    bool compute_norms;
    tf::Transform registration_tf, ell_reg_tf;
    Ellipsoid ell;

    void wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_stamped) 
    {
        double cur_time = wrench_stamped->header.stamp.toSec();
        if(start_time == -1)
            start_time = cur_time;
        CartVec w;
        wrenchMsgToEigen(wrench_stamped->wrench, w);
        double force_mag = NORM(w(0, 0), w(1, 0), w(2, 0));
        tf::StampedTransform tool_loc_tf;
        try {
            tf_list.waitForTransform("/head_center", "/wipe_finger", wrench_stamped->header.stamp, ros::Duration(0.1));
            tf_list.lookupTransform("/head_center", "/wipe_finger", wrench_stamped->header.stamp, tool_loc_tf);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        tool_loc_tf.mult(registration_tf, tool_loc_tf);
        btVector3 tool_loc = tool_loc_tf.getOrigin();
        PRGB query_pt;
        query_pt.x = tool_loc.x(); query_pt.y = tool_loc.y(); query_pt.z = tool_loc.z(); 
        vector<int> nk_inds(1);
        vector<float> nk_dists(1);
        kd_tree->nearestKSearch(query_pt, 1, nk_inds, nk_dists);
        int closest_ind = nk_inds[0];
        float closest_dist = nk_dists[0];

        hrl_phri_2011::ForceProcessed fp;
        fp.time_offset = cur_time - start_time;
        tf::transformStampedTFToMsg(tool_loc_tf, fp.tool_frame);
        fp.header = wrench_stamped->header;
        fp.header.frame_id = "/head_center";
        fp.wrench = wrench_stamped->wrench;
        fp.pc_ind = closest_ind;
        fp.pc_dist = closest_dist;
        fp.pc_pt.x = pc_head->points[closest_ind].x;
        fp.pc_pt.y = pc_head->points[closest_ind].y;
        fp.pc_pt.z = pc_head->points[closest_ind].z;
        fp.force_magnitude = force_mag;
        if(compute_norms) {
            fp.pc_normal.x = normals->points[closest_ind].normal[0];
            fp.pc_normal.y = normals->points[closest_ind].normal[1];
            fp.pc_normal.z = normals->points[closest_ind].normal[2];
        }

        // do ellipsoidal processing
        tf::Transform tool_loc_ell = ell_reg_tf * tool_loc_tf;
        tf::transformTFToMsg(tool_loc_ell, fp.tool_ell_frame);
        btVector3 tloce_pos = tool_loc_ell.getOrigin();
        ell.cartToEllipsoidal(tloce_pos.x(), tloce_pos.y(), tloce_pos.z(), 
                              fp.ell_coords.x, fp.ell_coords.y, fp.ell_coords.z);

        fp_list.push_back(fp);
        pub_ind++;
        if(pub_ind % 100 == 0)
            ROS_INFO("Recorded %d samples", pub_ind);
    }

public:
    void startVisualization(bool use_raw = false) 
    {
        string topic;
        if(!use_raw)
            topic = "/tool_netft_zeroer/wrench_zeroed";
        else
            topic = "/tool_netft/wrench_raw";
        wrench_sub = nh.subscribe(topic, 100, &DataExtractor::wrenchCallback, this);
    }

    DataExtractor(PCRGB::Ptr& pch, bool compute_normals, const geometry_msgs::Transform& registration,
                  const hrl_phri_2011::EllipsoidParams& ep) : 
        pc_head(pch),
        kd_tree(new pcl::KdTreeFLANN<PRGB> ()),
        normals(new PCNormals()),
        pub_ind(0),
        start_time(-1),
        compute_norms(compute_normals),
        ell(ep)
    {
        kd_tree->setInputCloud(pc_head);
        if(compute_norms)
            computeNormals(pc_head, kd_tree, normals);
        pc_head->header.frame_id = "/head_center";
        tf::transformMsgToTF(registration, registration_tf);
        if(ep.e_frame.transform.rotation.w != 0) {
            tf::transformMsgToTF(ep.e_frame.transform, ell_reg_tf);
            ell_reg_tf = ell_reg_tf.inverse();
        }
    }

    void writeBag(const string& pbag_name) 
    {
        rosbag::Bag processed_bag;
        processed_bag.open(pbag_name, rosbag::bagmode::Write);
        BOOST_FOREACH(const hrl_phri_2011::ForceProcessed& fp, fp_list) {
            processed_bag.write("/force_processed", fp.header.stamp, fp);
        }
        processed_bag.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_extractor");

    if(argc < 3 || argc > 7) {
        printf("Usage: data_extractor head_pc forces_bag_output [compute_normals=1] [registration_bag] [use_raw=0] [ellipse_registration]\n");
        return 1;
    }

    rosbag::Bag pc_bag, force_bag;
    PCRGB::Ptr pc_head(new PCRGB);
    // Load PC bag
    pc_bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View pc_view(pc_bag, rosbag::TopicQuery("/stitched_head"));
    BOOST_FOREACH(rosbag::MessageInstance const m, pc_view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
    }

    bool compute_normals = true;
    if(argc >= 4)
        compute_normals = atoi(argv[3]);
    bool use_raw = false;
    if(argc >= 6)
        use_raw = atoi(argv[5]);
    geometry_msgs::Transform tf_msg;
    tf_msg.rotation.w = 1.0;
    if(argc >= 5) {
        vector<geometry_msgs::TransformStamped::Ptr> tf_msgs;
        readBagTopic<geometry_msgs::TransformStamped>(argv[4], tf_msgs, "/itf_transform");
        tf_msg = tf_msgs[0]->transform;
    }
    vector<hrl_phri_2011::EllipsoidParams::Ptr> e_params_msgs;
    e_params_msgs.push_back(boost::shared_ptr<hrl_phri_2011::EllipsoidParams>(new hrl_phri_2011::EllipsoidParams()));
    e_params_msgs[0]->e_frame.transform.rotation.w = 1.0;
    if(argc >= 7) {
        e_params_msgs.clear();
        readBagTopic<hrl_phri_2011::EllipsoidParams>(argv[6], e_params_msgs, "/ellipsoid_params");
    }
    

    DataExtractor de(pc_head, compute_normals, tf_msg, *e_params_msgs[0]);
    de.startVisualization(use_raw);
    ROS_INFO("Ready for collection");
    ros::spin();
    de.writeBag(argv[2]);
    ROS_INFO("Bag written to %s", argv[2]);
}
