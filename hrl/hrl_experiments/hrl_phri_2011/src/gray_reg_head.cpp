#include <hrl_phri_2011/utils.h>
#include <hrl_phri_2011/ellipsoid_space.h>
#include <hrl_phri_2011/hsl_rgb_conversions.h>
#include <hrl_phri_2011/EllipsoidParams.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "function_extractor");
    ros::NodeHandle nh;

    if(argc < 4 || argc > 4) {
        printf("Usage: gray_reg_head head_pc ellipsoid_registration output_bag \n");
        return 1;
    }

    PCRGB head_pc;
    Ellipsoid e;
    loadRegisteredHead(argv[1], argv[2], head_pc, e);
    double h, s, l;
    for(size_t i=0;i<head_pc.size();i++) {
        extractHSL(head_pc.points[i].rgb, h, s, l);
        writeHSL(0, 0, l, head_pc.points[i].rgb);
    }

    head_pc.header.frame_id = "/base_link";
    rosbag::Bag bag;
    bag.open(argv[3], rosbag::bagmode::Write);
    bag.write("/data_cloud", ros::Time::now(), head_pc);
    bag.close();
}
