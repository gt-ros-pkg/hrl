

#include <hrl_head_tracking/head_tracking.h>
#include <hrl_head_tracking/HeadRegistration.h>
#include <geometry_msgs/TransformStamped.h>

typedef hrl_head_tracking::HeadRegistration HeadRegSrv;

PCRGB::Ptr cur_pc, template_pc;

ros::Subscriber pc_sub;
ros::ServiceServer reg_srv;
        
void subPCCallback(const PCRGB::Ptr& cur_pc_);
bool regCallback(HeadRegSrv::Request& req, HeadRegSrv::Response& resp);

void subPCCallback(const PCRGB::Ptr& cur_pc_)
{
    printf("here\n");
    cur_pc = cur_pc_;
}

void readTFBag(const string& filename, geometry_msgs::TransformStamped::Ptr& tf_msg) 
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/itf_transform"));
    BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
        tf_msg = msg.instantiate< geometry_msgs::TransformStamped >();
        break;
    }
    bag.close();
}

bool regCallback(HeadRegSrv::Request& req, HeadRegSrv::Response& resp)
{
    if(!cur_pc) {
        ROS_ERROR("No point cloud received.");
        return false;
    }
    Eigen::Affine3d tf_mat;
    geometry_msgs::PoseStamped tf_pose;
    findFaceRegistration(template_pc, cur_pc, req.u, req.v, tf_mat);
    tf::poseEigenToMsg(tf_mat, tf_pose.pose);

#if 0
    PCRGB::Ptr tf_pc(new PCRGB());
    transformPC(*cur_pc, *tf_pc, tf_mat);
    tf_pc->header.frame_id = "/openni_rgb_optical_frame";
    pubLoop(tf_pc, "test3", 5);
#endif
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_tracking");
    ros::NodeHandle nh;

    readPCBag(argv[1], template_pc);

    pc_sub = nh.subscribe("/kinect_head/rgb/points", 1, &subPCCallback);
    reg_srv = nh.advertiseService("/head_registration", &regCallback);
    ros::spin();

    return 0;
}
