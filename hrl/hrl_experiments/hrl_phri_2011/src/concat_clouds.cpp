#include <ros/ros.h>

#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "concat_clouds");
    ros::NodeHandle nh;

    if(argc == 1) {
        printf("Usage: concat_clouds cloud_bag1 ... output_bag\n");
        return 1;
    }

    PCRGB out_pc;
    for(int i=1;i<argc-1;i++) {
        vector<PCRGB::Ptr> cloud;
        readBagTopic<PCRGB>(argv[i], cloud, "/contact_cloud");
        if(out_pc.size() != 0)
            out_pc += *cloud[0];
        else
            out_pc = *cloud[0];
    }
    
    rosbag::Bag bag;
    bag.open(argv[argc-1], rosbag::bagmode::Write);
    bag.write("/contact_cloud", ros::Time::now(), out_pc);
    bag.close();

    return 0;
}
