#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <rosbag/bag.h>

using namespace std;

class InteractiveTF 
{
private:
    interactive_markers::InteractiveMarkerServer im_server_;
    tf::TransformBroadcaster tf_broad_;
    std::string parent_frame_, child_frame_;
    double rate_;
    ros::Timer tf_timer_;
    geometry_msgs::Pose marker_pose_;
    geometry_msgs::TransformStamped cur_tf_msg;
public:
    InteractiveTF(const std::string& parent_frame, const std::string& child_frame, double rate = 100);
    void processTFControl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void addTFMarker();
    void publishTF(const ros::TimerEvent& event);
    void bagTF(const string& bag_name, const string& topic_name);
};

InteractiveTF::InteractiveTF(const std::string& parent_frame, 
                             const std::string& child_frame, double rate) :
    im_server_("transform_marker"),
    parent_frame_(parent_frame),
    child_frame_(child_frame),
    rate_(rate) 
{
    marker_pose_.orientation.w = 1;
}

void InteractiveTF::addTFMarker() 
{
    ros::NodeHandle nh;
    visualization_msgs::InteractiveMarker tf_marker;
    tf_marker.header.frame_id = parent_frame_;
    tf_marker.name = "tf_marker";
    tf_marker.scale = 0.2;
    visualization_msgs::InteractiveMarkerControl tf_control;
    tf_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    // x
    tf_control.orientation.x = 1; tf_control.orientation.y = 0;
    tf_control.orientation.z = 0; tf_control.orientation.w = 1;
    tf_control.name = "rotate_x";
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    tf_marker.controls.push_back(tf_control);
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    tf_marker.controls.push_back(tf_control);
    // y
    tf_control.orientation.x = 0; tf_control.orientation.y = 1;
    tf_control.orientation.z = 0; tf_control.orientation.w = 1;
    tf_control.name = "rotate_y";
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    tf_marker.controls.push_back(tf_control);
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    tf_marker.controls.push_back(tf_control);
    // z
    tf_control.orientation.x = 0; tf_control.orientation.y = 0;
    tf_control.orientation.z = 1; tf_control.orientation.w = 1;
    tf_control.name = "rotate_z";
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    tf_marker.controls.push_back(tf_control);
    tf_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    tf_marker.controls.push_back(tf_control);
    im_server_.insert(tf_marker, boost::bind(&InteractiveTF::processTFControl, this, _1));
    im_server_.applyChanges();
    tf_timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &InteractiveTF::publishTF, this);
}

void InteractiveTF::processTFControl(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) 
{
    /*
    ROS_INFO_STREAM(feedback->pose.position.x << " " << 
                    feedback->pose.position.y << " " << 
                    feedback->pose.position.z << " " << 
                    feedback->pose.orientation.x << " " << 
                    feedback->pose.orientation.y << " " << 
                    feedback->pose.orientation.z << " " << 
                    feedback->pose.orientation.w << " ");
    */
    marker_pose_ = feedback->pose;
}

void InteractiveTF::publishTF(const ros::TimerEvent& event) 
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = marker_pose_.position.x;
    tf_msg.transform.translation.y = marker_pose_.position.y;
    tf_msg.transform.translation.z = marker_pose_.position.z;
    tf_msg.transform.rotation.x = marker_pose_.orientation.x;
    tf_msg.transform.rotation.y = marker_pose_.orientation.y;
    tf_msg.transform.rotation.z = marker_pose_.orientation.z;
    tf_msg.transform.rotation.w = marker_pose_.orientation.w;
    tf_broad_.sendTransform(tf_msg);
    cur_tf_msg = tf_msg;
}

void InteractiveTF::bagTF(const string& bag_name, const string& topic_name) 
{
    rosbag::Bag bag;
    bag.open(bag_name, rosbag::bagmode::Write);
    bag.write(topic_name, cur_tf_msg.header.stamp, cur_tf_msg);
    bag.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interactive_tf", ros::init_options::AnonymousName);
    if(argc < 3 || argc > 6) {
        printf("Usage: interative_tf parent_frame child_frame [rate] [bag_file] [bag_topic]\n");
        return 1;
    }
    if(argc >= 4) {
        InteractiveTF itf(argv[1], argv[2], atof(argv[3]));
        itf.addTFMarker();
        ros::spin();
        if(argc >= 5) {
            string topic_name = "/itf_transform";
            if(argc >= 6)
                topic_name = argv[5];
            itf.bagTF(argv[4], topic_name);
        }
    } else {
        InteractiveTF itf(argv[1], argv[2]);
        itf.addTFMarker();
        ros::spin();
    }

    return 0;
}
