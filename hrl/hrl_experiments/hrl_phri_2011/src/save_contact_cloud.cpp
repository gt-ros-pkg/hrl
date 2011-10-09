#include <ros/ros.h>

#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ForceProcessed.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_contact_cloud");
    ros::NodeHandle nh;

    if(argc < 2 || argc > 4) {
        printf("Usage: save_contact_cloud forces_bag output_bag [frame]\n");
        return 1;
    }

    // load forces
    vector<hrl_phri_2011::ForceProcessed::Ptr> forces;
    readBagTopic<hrl_phri_2011::ForceProcessed>(argv[1], forces, "/force_processed");

    PCRGB fpc;
    for(uint32_t i=0;i<forces.size();i++) {
        PRGB pt;
        pt.x = forces[i]->tool_frame.transform.translation.x;
        pt.y = forces[i]->tool_frame.transform.translation.y;
        pt.z = forces[i]->tool_frame.transform.translation.z;
        pt.rgb = forces[i]->force_magnitude;
        fpc.points.push_back(pt);
    }
    if(argc > 3)
        fpc.header.frame_id = argv[3];
    else
        fpc.header.frame_id = "/base_link";
    
    rosbag::Bag bag;
    bag.open(argv[2], rosbag::bagmode::Write);
    bag.write("/contact_cloud", ros::Time::now(), fpc);
    bag.close();

    return 0;
}
