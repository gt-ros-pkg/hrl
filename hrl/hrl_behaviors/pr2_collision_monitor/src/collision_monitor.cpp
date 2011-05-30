#include <numeric>
#include <math.h>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <pr2_collision_monitor/FingerState.h>
#include <pr2_collision_monitor/FingerStateSrv.h>
#include <pr2_collision_monitor/JointDetectionStart.h>

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {
    class CollisionMonitor {
        public:
            ros::NodeHandle nh;
            ros::ServiceServer start_srv, stop_srv;
            ros::ServiceClient joint_start_cli, joint_stop_cli;
            ros::Subscriber finger_coll_sub, joint_coll_sub;
            ros::Publisher arm_stop_pub, collision_pub;
            std::string arm;
            bool detection_on;
            double finger_last_time, joint_last_time;

            CollisionMonitor();
            void onInit();
            void fingerCollCallback(const FingerState& msg);
            void jointCollCallback(const std_msgs::Bool& msg);
            void stateCallback(const std_msgs::Bool& msg);
            bool srvStartDetection(JointDetectionStart::Request&, 
                                   JointDetectionStart::Response&);
            bool srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            void collisionProcess();
            void spinMonitor();
    };

    CollisionMonitor::CollisionMonitor() : nh("~") {
    }

    void CollisionMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));

        start_srv = nh.advertiseService("start_monitoring", 
                                             &CollisionMonitor::srvStartDetection, this);
        ROS_INFO("[collision_monitor] Service advertised at start_monitoring");
        stop_srv = nh.advertiseService("stop_monitoring", 
                                            &CollisionMonitor::srvStopDetection, this);
        ROS_INFO("[collision_monitor] Service advertised at stop_monitoring");
        joint_start_cli = nh.serviceClient<JointDetectionStart>(
                             "/" + arm + "_joint_coll_detect/start_detection", false);
        joint_stop_cli = nh.serviceClient<std_srvs::Empty>(
                             "/" + arm + "_joint_coll_detect/stop_detection", false);

        finger_coll_sub = nh.subscribe("/" + arm + "_fingertip_monitor/collision_state", 1, 
                                       &CollisionMonitor::fingerCollCallback, this);
        joint_coll_sub = nh.subscribe(
                            "/" + arm + "_joint_coll_detect/arm_collision_detected", 1, 
                            &CollisionMonitor::jointCollCallback, this);
        arm_stop_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
                                                 "/" + arm + "_arm_controller/command", 1);
        collision_pub = nh.advertise<std_msgs::Bool>("collision_detected", 1);

        ros::Duration(0.3).sleep();
        ROS_INFO("[collision_monitor] CollisionMonitor loaded.");
    }

    void CollisionMonitor::fingerCollCallback(const FingerState& msg) {
        if(detection_on) {
            if(msg.any_collision) {
                ROS_INFO("[collision_monitor] Finger collision detected, stopping arm...");
                collisionProcess();
            }
            finger_last_time = ros::Time::now().toSec();
        }
    }

    void CollisionMonitor::jointCollCallback(const std_msgs::Bool& msg) {
        if(detection_on) {
            if(msg.data) {
                ROS_INFO("[collision_monitor] Joint collision detected, stopping arm...");
                collisionProcess();
            }
            joint_last_time = ros::Time::now().toSec();
        }
    }

    bool CollisionMonitor::srvStartDetection(JointDetectionStart::Request& req,
                                             JointDetectionStart::Response& resp) {
        finger_last_time = ros::Time::now().toSec();
        joint_last_time = ros::Time::now().toSec();
        joint_start_cli.call(req, resp);
        detection_on = true;
        return true;
    }

    bool CollisionMonitor::srvStopDetection(std_srvs::Empty::Request& req, 
                                            std_srvs::Empty::Response& resp) {
        joint_stop_cli.call(req, resp);
        detection_on = false;
        return true;
    }

    void CollisionMonitor::collisionProcess() {
        if(detection_on) {
            trajectory_msgs::JointTrajectory stop_arm_msg;
            stop_arm_msg.header.stamp = ros::Time::now();
            stop_arm_msg.points.resize(1);
            
            arm_stop_pub.publish(stop_arm_msg);
            std_msgs::Bool coll_msg; collision_pub.publish(coll_msg);
            detection_on = false;
            std_srvs::Empty::Request req; std_srvs::Empty::Response resp;
            joint_stop_cli.call(req, resp);
        }
    }

    void CollisionMonitor::stateCallback(const std_msgs::Bool& msg) {
        detection_on = msg.data;
    }

    void CollisionMonitor::spinMonitor() {
        ros::Rate r(100);
        while(ros::ok()) {
            ros::spinOnce();
            if(detection_on) {
                if(ros::Time::now().toSec() - joint_last_time > 0.3) {
                    ROS_INFO("[collision_monitor] Joint detection timeout, stopping...");
                    collisionProcess();
                }
                if(ros::Time::now().toSec() - finger_last_time > 0.3) {
                    ROS_INFO("[collision_monitor] Finger detection timeout, stopping...");
                    collisionProcess();
                }
            }
            r.sleep();
        }
    }

};

using namespace pr2_collision_monitor;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_monitor", ros::init_options::AnonymousName);
    CollisionMonitor cm;
    cm.onInit();
    cm.spinMonitor();
    return 0;
}

