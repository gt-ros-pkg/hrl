#include <ros/ros.h>

#include <hrl_phri_2011/pcl_basic.h>
#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ForceProcessed.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_contact_cloud");

    if(argc < 2 || argc > 4) {
        printf("Usage: show_contact_cloud forces_bag [frame] [hz]\n");
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
        ((uint32_t*) &pt.rgb)[0] = 0xffffffff;
        fpc.points.push_back(pt);
    }
    if(argc > 2)
        fpc.header.frame_id = argv[2];
    else
        fpc.header.frame_id = "/base_link";
    if(argc == 3)
        pubLoop(fpc, "contact_cloud");
    else
        pubLoop(fpc, "contact_cloud", atof(argv[3]));

    return 0;
}
