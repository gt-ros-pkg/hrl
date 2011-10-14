#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ellipsoid_space.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/EllipsoidParams.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

float extractValue(const hrl_phri_2011::ForceProcessed& fp, const string& value)
{
    if(value == "force_magnitude")
        return fp.force_magnitude;
    return -9999;
}

void extractToolFrameCloud(const vector<hrl_phri_2011::ForceProcessed::Ptr>& fp_list, const string& value,
                           PCRGB& data_cloud)
{
    for(size_t i=0;i<fp_list.size();i++) {
        PRGB pt;
        pt.x = fp_list[i]->tool_frame.transform.translation.x;
        pt.y = fp_list[i]->tool_frame.transform.translation.y;
        pt.z = fp_list[i]->tool_frame.transform.translation.z;
        pt.rgb = extractValue(*fp_list[i], value);
        data_cloud.points.push_back(pt);
    }
}

void extractEllipsoidFrameCloud(const vector<hrl_phri_2011::ForceProcessed::Ptr>& fp_list, const string& value,
                                PCRGB& data_cloud, Ellipsoid& e, bool use_ell_height = false)
{
    double x, y, z, lat, lon, height;
    for(size_t i=0;i<fp_list.size();i++) {
        PRGB pt;
        lat = fp_list[i]->ell_coords.x;
        lon = fp_list[i]->ell_coords.y;
        if(!use_ell_height)
            height = fp_list[i]->ell_coords.z;
        else
            height = e.height;
        e.ellipsoidalToCart(lat, lon, height, x, y, z);
        pt.x = x; pt.y = y; pt.z = z;
        pt.rgb = extractValue(*fp_list[i], value);
        data_cloud.points.push_back(pt);
    }
}

void projectCloudEllipsoid(const PCRGB& in_pc, PCRGB& out_pc, Ellipsoid& e)
{
    double x, y, z, lat, lon, height;
    for(size_t i=0;i<in_pc.size();i++) {
        PRGB pt;
        x = in_pc.points[i].x; y = in_pc.points[i].y; z = in_pc.points[i].z;
        e.cartToEllipsoidal(x, y, z, lat, lon, height);
        e.ellipsoidalToCart(lat, lon, e.height, x, y, z);
        pt.x = x; pt.y = y; pt.z = z;
        pt.rgb = in_pc.points[i].rgb;
        out_pc.points.push_back(pt);
    }
}

void projectDataEllipsoidHead(const PCRGB& head_pc, const PCRGB& ell_head_pc, const PCRGB& data_pc, PCRGB& out_pc)
{
    boost::mt19937 rand_gen;
    boost::normal_distribution<> norm_dist(0.0, 0.000);
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > norm_gen(rand_gen, norm_dist);

    pcl::KdTreeFLANN<PRGB> head_kd(new pcl::KdTreeFLANN<PRGB> ());
    head_kd.setInputCloud(ell_head_pc.makeShared());
    vector<int> inds(1); vector<float> dists(1);
    for(size_t i=0;i<data_pc.size();i++) {
        inds[0] = -1;
        PRGB pt;
        head_kd.nearestKSearch(data_pc, i, 1, inds, dists);
        pt.x = head_pc.points[inds[0]].x + norm_gen();
        pt.y = head_pc.points[inds[0]].y + norm_gen();
        pt.z = head_pc.points[inds[0]].z + norm_gen();
        pt.rgb = data_pc.points[i].rgb;
        out_pc.push_back(pt);
    }
}

void multiplyCloud(PCRGB& pc, double multiplier)
{
    for(size_t i=0;i<pc.size();i++)
        pc.points[i].rgb = pc.points[i].rgb * multiplier;
}

void extractContacts(PCRGB& pc, double force_thresh, double time_thresh)
{
    PCRGB tmp_pc, new_pc;
    ROS_INFO("IN %d", pc.size());
    for(size_t i=0;i<pc.size();i++) {
        if(pc.points[i].rgb > force_thresh) {
            tmp_pc.push_back(pc.points[i]);
        } else {
            if(tmp_pc.size() > time_thresh * 100)
                new_pc += tmp_pc;
            tmp_pc.clear();
        }
    }
    pc = new_pc;
    ROS_INFO("OUT %d", pc.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "function_extractor");
    ros::NodeHandle nh;
    if(argc < 4 || argc > 6) {
        printf("Usage: function_extractor force_processed_bag value output_bag [ellipsoid_registration] [user_projection] [multiplier]\n");
        return 1;
    }
    vector<hrl_phri_2011::ForceProcessed::Ptr> fp_list;
    readBagTopic<hrl_phri_2011::ForceProcessed>(argv[1], fp_list, "/force_processed");
    PCRGB data_cloud;
    if(argc >= 5) {
        vector<hrl_phri_2011::EllipsoidParams::Ptr> ep_list;
        readBagTopic<hrl_phri_2011::EllipsoidParams>(argv[4], ep_list, "/ellipsoid_params");
        Ellipsoid e(*ep_list[0]);
        if(argc >= 6) {
            PCRGB head_pc, ell_head_pc, ell_data_pc;
            loadRegisteredHead(argv[5], argv[4], head_pc, e);
            projectCloudEllipsoid(head_pc, ell_head_pc, e);
            extractEllipsoidFrameCloud(fp_list, argv[2], ell_data_pc, e, true);
            projectDataEllipsoidHead(head_pc, ell_head_pc, ell_data_pc, data_cloud);
        } else
            extractEllipsoidFrameCloud(fp_list, argv[2], data_cloud, e);
    } else {
        extractToolFrameCloud(fp_list, argv[2], data_cloud);
    }
    double force_thresh, time_thresh, multiplier;
    ros::param::param<double>("~force_thresh", force_thresh, -1);
    ros::param::param<double>("~time_thresh", time_thresh, -1);
    ros::param::param<double>("~multiplier", multiplier, -1);
    if(force_thresh != -1 && time_thresh != -1)
        extractContacts(data_cloud, force_thresh, time_thresh);
    if(multiplier != -1)
        multiplyCloud(data_cloud, multiplier);
    data_cloud.header.frame_id = "/base_link";
    rosbag::Bag bag;
    bag.open(argv[3], rosbag::bagmode::Write);
    bag.write("/data_cloud", ros::Time::now(), data_cloud);
    bag.close();
}
