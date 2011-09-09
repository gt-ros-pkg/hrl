#include <hrl_phri_2011/pc_utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_head");

    // Load bag
    rosbag::Bag bag;
    bag.open(std::string(argv[1]), rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/stitched_head"));

    PCRGB::Ptr pc_head(new PCRGB());
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        sensor_msgs::PointCloud2::Ptr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
        pcl::fromROSMsg(*pc2, *pc_head);
        break;
    }
    pubLoop(*pc_head, "/stitched_head");
    return 0;
}
