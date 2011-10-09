#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>

#include <hrl_phri_2011/pcl_basic.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_head", ros::init_options::AnonymousName);
    if(argc < 2 || argc > 5) {
        printf("Usage pub_head bag_file [topic] [frame] [rate]\n");
        return 1;
    }

    // Load bag
    rosbag::Bag bag;
    bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View view(bag);

    PCRGB::Ptr pc_head(new PCRGB());
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
        break;
    }
    if(argc == 2)
        pubLoop(*pc_head, "/stitched_head");
    else if(argc == 3)
        pubLoop(*pc_head, std::string(argv[2]));
    else if(argc == 4) {
        pc_head->header.frame_id = std::string(argv[3]);
        pubLoop(*pc_head, std::string(argv[2]));
    }
    else if(argc == 5) {
        pc_head->header.frame_id = std::string(argv[3]);
        pubLoop(*pc_head, std::string(argv[2]), atof(argv[4]));
    }

    return 0;
}
