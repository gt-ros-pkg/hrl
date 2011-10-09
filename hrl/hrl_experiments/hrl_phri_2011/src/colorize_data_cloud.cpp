#include <ros/ros.h>

#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>

void colorizeDataPC(const PCRGB& data_pc, PCRGB& color_pc, double saturation=100, double lightness=50)
{
    vector<float> data;
    for(size_t i=0;i<data_pc.size();i++) 
        data.push_back(data_pc.points[i].rgb);

    float max_val = *std::max_element(data.begin(), data.end());
    float min_val = *std::min_element(data.begin(), data.end());
    ROS_INFO("Max data value: %f, Min data_value: %f", max_val, min_val);
    double h, s, l;
    for(size_t i=0;i<data.size();i++) {
        PRGB pt;
        pt.x = data_pc.points[i].x;
        pt.y = data_pc.points[i].y;
        pt.z = data_pc.points[i].z;
        h = (double) 240.0 * data[i] / max_val;
        if(h < 0) h = 0; if(h > 240.0) h = 240.0;
        writeHSL(240.0 - h, saturation, lightness, pt.rgb);
        color_pc.push_back(pt);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "colorize_data_cloud");
    ros::NodeHandle nh;

    if(argc < 2 || argc > 4) {
        printf("Usage: colorize_data_cloud data_bag output_bag\n");
        return 1;
    }

    vector<PCRGB::Ptr> cloud;
    readBagTopic<PCRGB>(argv[1], cloud, "/contact_cloud");
    PCRGB color_pc;
    colorizeDataPC(*cloud[0], color_pc, 100);
    color_pc.header.frame_id = cloud[0]->header.frame_id;
    
    rosbag::Bag bag;
    bag.open(argv[2], rosbag::bagmode::Write);
    bag.write("/contact_cloud", ros::Time::now(), color_pc);
    bag.close();

    return 0;
}
