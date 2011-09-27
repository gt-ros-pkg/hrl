#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ForceProcessed.h>

void extractToolFrameCloud(const vector<hrl_phri_2011::ForceProcessed::Ptr>& fp_list, const string& value,
                           PCRGB& data_cloud)
{
    for(size_t i=0;i<fp_list.size();i++) {
        PRGB pt;
        pt.x = fp_list[i]->tool_frame.transform.translation.x;
        pt.y = fp_list[i]->tool_frame.transform.translation.y;
        pt.z = fp_list[i]->tool_frame.transform.translation.z;
        if(value == "force_magnitude")
            pt.rgb = fp_list[i]->force_magnitude;
        data_cloud.points.push_back(pt);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "function_extractor");
    ros::NodeHandle nh;
    vector<hrl_phri_2011::ForceProcessed::Ptr> fp_list;
    readBagTopic<hrl_phri_2011::ForceProcessed>(argv[1], fp_list, "/force_processed");
    PCRGB data_cloud;
    extractToolFrameCloud(fp_list, argv[2], data_cloud);
    data_cloud.header.frame_id = "/base_link";
    rosbag::Bag bag;
    bag.open(argv[3], rosbag::bagmode::Write);
    bag.write("/data_cloud", ros::Time::now(), data_cloud);
    bag.close();
}
