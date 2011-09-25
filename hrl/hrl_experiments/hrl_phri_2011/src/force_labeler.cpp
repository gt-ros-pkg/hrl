#include <hrl_phri_2011/pc_utils.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>

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

#define NORMAL(x, sig) ( std::exp( - (x) * (x) / (2.0 * (sig) * (sig))) / std::sqrt(2.0 * 3.14159 * (sig) * (sig)))

class ForceVisualizer 
{
private:
    ros::Subscriber wrench_sub;
    ros::Publisher pc_pub;
    tf::TransformListener tf_list;
    ros::NodeHandle nh;
    PCRGB::Ptr pc_head;
    KDTree::Ptr kd_tree;
    int pub_ind;
    vector<double> force_densities;
    vector<double> force_high;
    double force_density_sum;
    double force_high_sum;

    void wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_stamped) 
    {
        double sigma = 0.002, force_zero_thresh = 1.0, force_high_thresh = 3.0;
        CartVec w;
        wrenchMsgToEigen(wrench_stamped->wrench, w);
        double force_mag = NORM(w(0, 0), w(1, 0), w(2, 0));
        if(force_mag < force_zero_thresh)
            return;
        tf::StampedTransform tool_loc_tf;
        try {
            tf_list.lookupTransform("/head_center", "/wipe_finger", ros::Time(0), tool_loc_tf);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
        btVector3 tool_loc = tool_loc_tf.getOrigin();
        PRGB query_pt;
        query_pt.x = tool_loc.x(); query_pt.y = tool_loc.y(); query_pt.z = tool_loc.z(); 
        vector<int> nk_inds(1), rs_inds;
        vector<float> nk_dists(1), rs_dists;
        kd_tree->nearestKSearch(query_pt, 1, nk_inds, nk_dists);
        int closest_ind = nk_inds[0];
        kd_tree->radiusSearch(closest_ind, sigma * 3, rs_inds, rs_dists);
        for(uint32_t i=0;i<rs_inds.size();i++) {
            double prob_dens = NORMAL(rs_dists[i], sigma);
            force_densities[rs_inds[i]] += prob_dens;
            force_density_sum += prob_dens;
            if(force_mag > force_high_thresh) {
                force_high[rs_inds[i]] += prob_dens;
                force_high_sum += prob_dens;
            }
            //((uint8_t*) &pc_head->points[rs_inds[i]].rgb)[2] = 0xFF;
        }
        if(++pub_ind % 10 == 0) 
            colorHead(wrench_stamped->header.stamp);
    }

    void colorHead(const ros::Time& stamp) 
    {
        PCRGB::Ptr cur_head(new PCRGB(*pc_head));
        for(uint32_t i=0;i<pc_head->points.size();i++) {
            // 0 = blue, 1 = green, 2 = red
            uint8_t r = ((uint8_t*) &cur_head->points[i].rgb)[2];
            uint8_t g = ((uint8_t*) &cur_head->points[i].rgb)[1];
            uint8_t b = ((uint8_t*) &cur_head->points[i].rgb)[0];
            double h, s, l;
            RGBToHSL(r, g, b, h, s, l);
            h = 0.0; s = 0.0;

            ///////////////////////////////////////////////////////////////
            if(force_densities[i] > 0.001) {
                //double post_prob = (force_high[i] * force_density_sum) / (force_densities[i] * force_high_sum);
                double post_prob = force_high[i] / force_densities[i];
                //if(i % 100 == 0)
                //    ROS_INFO("%f %f %f", post_prob, force_high[i] / force_high_sum, force_densities[i] / force_density_sum);
                s = post_prob * 100.0;
            }
            ///////////////////////////////////////////////////////////////

            HSLToRGB(h, s, l, r, g, b);
            ((uint8_t*) &cur_head->points[i].rgb)[2] = r;
            ((uint8_t*) &cur_head->points[i].rgb)[1] = g;
            ((uint8_t*) &cur_head->points[i].rgb)[0] = b;
        }
        cur_head->header.stamp = stamp;
        pc_pub.publish(*cur_head);
    }

public:
    void startVisualization() 
    {
        wrench_sub = nh.subscribe("/tool_netft_zeroer/wrench_zeroed", 1, &ForceVisualizer::wrenchCallback, this);
        pc_pub = nh.advertise<PCRGB >("/colored_pc", 1);
    }

    ForceVisualizer(PCRGB::Ptr& pch) : 
        pc_head(pch),
        kd_tree(new pcl::KdTreeFLANN<PRGB> ()),
        pub_ind(0),
        force_densities(pch->points.size()),
        force_high(pch->points.size()),
        force_density_sum(0.0),
        force_high_sum(0.0) {
        kd_tree->setInputCloud(pc_head);
        pc_head->header.frame_id = "/head_center";
        ros::Duration(1.0).sleep();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_labeler");

    rosbag::Bag pc_bag, force_bag;
    PCRGB::Ptr pc_head(new PCRGB);
    // Load PC bag
    pc_bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    //rosbag::View pc_view(pc_bag, rosbag::TopicQuery("/stitched_head"));
    rosbag::View pc_view(pc_bag, rosbag::TopicQuery("/pub_pc"));
    BOOST_FOREACH(rosbag::MessageInstance const m, pc_view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
    }

    ForceVisualizer fv(pc_head);
    fv.startVisualization();
    ros::spin();
}
