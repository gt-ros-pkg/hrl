#include <hrl_phri_2011/pc_utils.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>

typedef Eigen::Matrix<double, 6, 1> CartVec;

void wrenchMsgToEigen(const geometry_msgs::Wrench& wrench_msg, CartVec& wrench_eig) {
    wrench_eig(0, 0) = wrench_msg.force.x;
    wrench_eig(1, 0) = wrench_msg.force.y;
    wrench_eig(2, 0) = wrench_msg.force.z;
    wrench_eig(3, 0) = wrench_msg.torque.x;
    wrench_eig(4, 0) = wrench_msg.torque.y;
    wrench_eig(5, 0) = wrench_msg.torque.z;
}

void RGBToHSL(uint8_t r, uint8_t g, uint8_t b, double& h, double& s, double& l) {
    double rd = r / 255.0, gd = g / 255.0, bd = b / 255.0;
    double min_color = min(rd, min(gd, bd));
    double max_color = max(rd, max(gd, bd));
    l = (min_color + max_color) / 2.0;
    if(min_color == max_color) {
        s = 0.0; h = 0.0;
        l *= 100.0;
        return;
    }
    if(l < 0.5) 
        s = (max_color - min_color) / (max_color + min_color);
    else
        s = (max_color - min_color) / (2.0 - max_color - min_color);
    if(rd == max_color)
        h = (gd - bd) / (max_color - min_color);
    else if(gd == max_color)
        h = 2.0 + (bd - rd) / (max_color - min_color);
    else 
        h = 4.0 + (rd - bd) / (max_color - min_color);
    h *= 60.0;
    if(h < 0)
        h += 360.0;
    s *= 100.0;
    l *= 100.0;
}

void HSLToRGB(double h, double s, double l, uint8_t& r, uint8_t& g, uint8_t& b) {
    h /= 360.0;
    s /= 100.0;
    l /= 100.0;
    double rd, gd, bd;
    if(s == 0) {
        rd = l; gd = l; bd = l;
    } else {
        double temp2;
        if(l < 0.5)
            temp2 = l * (1.0 + s);
        else
            temp2 = l + s - l*s;
        double temp1 = 2.0 * l - temp2;
        double rtemp3 = h + 1.0 / 3.0;
        if(rtemp3 < 0) rtemp3 += 1.0;
        if(rtemp3 > 1) rtemp3 -= 1.0;
        double gtemp3 = h;
        if(gtemp3 < 0) gtemp3 += 1.0;
        if(gtemp3 > 1) gtemp3 -= 1.0;
        double btemp3 = h - 1.0 / 3.0;
        if(btemp3 < 0) btemp3 += 1.0;
        if(btemp3 > 1) btemp3 -= 1.0;
        if(6.0 * rtemp3 < 1.0) rd = temp1 + (temp2 - temp1) * 6.0 * rtemp3;
        else if(2.0 * rtemp3 < 1.0) rd = temp2;
        else if(3.0 * rtemp3 < 2.0) rd = temp1 + (temp2 - temp1) * (2.0/3.0 - rtemp3) * 6.0;
        else rd = temp1;
        if(6.0 * gtemp3 < 1.0) gd = temp1 + (temp2 - temp1) * 6.0 * gtemp3;
        else if(2.0 * gtemp3 < 1.0) gd = temp2;
        else if(3.0 * gtemp3 < 2.0) gd = temp1 + (temp2 - temp1) * (2.0/3.0 - gtemp3) * 6.0;
        else gd = temp1;
        if(6.0 * btemp3 < 1.0) bd = temp1 + (temp2 - temp1) * 6.0 * btemp3;
        else if(2.0 * btemp3 < 1.0) bd = temp2;
        else if(3.0 * btemp3 < 2.0) bd = temp1 + (temp2 - temp1) * (2.0/3.0 - btemp3) * 6.0;
        else bd = temp1;
    }
    r = rd * 255.0;
    g = gd * 255.0;
    b = bd * 255.0;
}

#define NORM(x, y, z) ( std::sqrt( (x) * (x) + (y) * (y) + (z) * (z) ) )

#define NORMAL(x, sig) ( std::exp( - (x) * (x) / (2.0 * (sig) * (sig))) / std::sqrt(2.0 * 3.14159 * (sig) * (sig)))

class ForceVisualizer {
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

    void wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_stamped) {
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

    void colorHead(const ros::Time& stamp) {
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
    void startVisualization() {
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
