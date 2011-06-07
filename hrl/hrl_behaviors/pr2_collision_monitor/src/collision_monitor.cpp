#include <numeric>
#include <math.h>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <pr2_collision_monitor/FingerState.h>
#include <pr2_collision_monitor/FingerStateSrv.h>
#include <pr2_collision_monitor/StartMonitor.h>
#include <pr2_collision_monitor/JointDetectionStart.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    int JOINTSTATE_INDS_R[] = {17, 18, 16, 20, 19, 21, 22};
    int JOINTSTATE_INDS_L[] = {29, 30, 28, 32, 31, 33, 34};
    string JOINTNAMES[] = { "_shoulder_pan_joint",
                            "_shoulder_lift_joint",
                            "_upper_arm_roll_joint",
                            "_elbow_flex_joint",
                            "_forearm_roll_joint",
                            "_wrist_flex_joint",
                            "_wrist_roll_joint" };

    class CollisionMonitor {
        public:
            ros::NodeHandle nh;
            ros::ServiceServer start_srv, stop_srv;
            ros::ServiceClient joint_start_cli, joint_stop_cli;
            ros::Subscriber finger_coll_sub, joint_coll_sub, force_coll_sub, state_sub;
            ros::Publisher arm_stop_pub, collision_pub, heartbeat_pub;
            std::string arm;
            bool detection_on, joint_on, finger_on, force_on;
            double finger_last_time, joint_last_time, force_last_time;
            vector<double> cur_joint_pos;

            CollisionMonitor();
            void onInit();
            void fingerCollCallback(const std_msgs::Bool& msg);
            void jointCollCallback(const std_msgs::Bool& msg);
            void forceCollCallback(const std_msgs::Bool& msg);
            void stateCallback(sensor_msgs::JointState::ConstPtr);
            bool srvStartDetection(StartMonitor::Request&, 
                                   StartMonitor::Response&);
            bool srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            void collisionProcess();
            void spinMonitor();
    };

    CollisionMonitor::CollisionMonitor() : nh("~"), cur_joint_pos(7) {
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

        finger_coll_sub = nh.subscribe("/" + arm + "_fingertip_monitor/collision_detected", 1, 
                                       &CollisionMonitor::fingerCollCallback, this);
        joint_coll_sub = nh.subscribe(
                            "/" + arm + "_joint_coll_detect/arm_collision_detected", 1, 
                            &CollisionMonitor::jointCollCallback, this);
        force_coll_sub = nh.subscribe("/force_torque_monitor/collision_detected", 1, 
                                      &CollisionMonitor::forceCollCallback, this);
        state_sub = nh.subscribe("/joint_states", 2, 
                                 &CollisionMonitor::stateCallback, this);
        arm_stop_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
                                                 "/" + arm + "_arm_controller/command", 1);
        // silent topic until a collision is detected
        collision_pub = nh.advertise<std_msgs::Bool>("collision_detected", 1);
        // publishes a boolean at 100hz which is false while detection is running and
        // true otherwise
        heartbeat_pub = nh.advertise<std_msgs::Bool>("collision_heartbeat", 1);

        ros::Duration(0.3).sleep();
        ROS_INFO("[collision_monitor] CollisionMonitor loaded.");
    }

    void CollisionMonitor::jointCollCallback(const std_msgs::Bool& msg) {
        if(detection_on && joint_on) {
            if(msg.data) {
                ROS_INFO("[collision_monitor] Joint collision detected, stopping arm...");
                collisionProcess();
            }
            joint_last_time = ros::Time::now().toSec();
        }
    }

    void CollisionMonitor::fingerCollCallback(const std_msgs::Bool& msg) {
        if(detection_on && finger_on) {
            if(msg.data) {
                ROS_INFO("[collision_monitor] Finger collision detected, stopping arm...");
                collisionProcess();
            }
            finger_last_time = ros::Time::now().toSec();
        }
    }

    void CollisionMonitor::forceCollCallback(const std_msgs::Bool& msg) {
        if(detection_on && force_on) {
            if(msg.data) {
                ROS_INFO("[collision_monitor] Force collision detected, stopping arm...");
                collisionProcess();
            }
            force_last_time = ros::Time::now().toSec();
        }
    }

    void CollisionMonitor::stateCallback(
            sensor_msgs::JointState::ConstPtr msg) {
        for(uint32_t i=0;i<7;i++) {
            int msg_ind;
            if(arm == "r")
                msg_ind = JOINTSTATE_INDS_R[i];
            else
                msg_ind = JOINTSTATE_INDS_L[i];
            cur_joint_pos[i] = msg->position[msg_ind];
        }
    }

    bool CollisionMonitor::srvStartDetection(StartMonitor::Request& req,
                                             StartMonitor::Response& resp) {
        joint_on = req.joint_detect;
        finger_on = req.finger_detect;
        force_on = req.force_detect;

        joint_last_time = ros::Time::now().toSec();
        finger_last_time = ros::Time::now().toSec();
        force_last_time = ros::Time::now().toSec();
        JointDetectionStart::Request jds_req; JointDetectionStart::Response jds_resp;
        joint_start_cli.call(jds_req, jds_resp);
        detection_on = true;
        return true;
    }

    bool CollisionMonitor::srvStopDetection(std_srvs::Empty::Request& req, 
                                            std_srvs::Empty::Response& resp) {
        ROS_INFO("[collision_monitor] Stop detection service called.");
        joint_stop_cli.call(req, resp);
        detection_on = false;
        return true;
    }

    void CollisionMonitor::collisionProcess() {
        if(detection_on) {
            detection_on = false;
            // publish collision message
            std_msgs::Bool coll_msg; coll_msg.data = true; collision_pub.publish(coll_msg);

            // turn off joint collision detection
            std_srvs::Empty::Request req; std_srvs::Empty::Response resp;
            joint_stop_cli.call(req, resp);

            // stop arm at current joint angles
            /* TODO LEAVE OR KEEP? TODO
            trajectory_msgs::JointTrajectory stop_arm_msg;
            stop_arm_msg.header.stamp = ros::Time::now() + ros::Duration(0.03);
            stop_arm_msg.points.resize(1);
            stop_arm_msg.joint_names.resize(7);
            for(uint32_t i=0;i<7;i++)
                stop_arm_msg.joint_names[i] = arm + JOINTNAMES[i];
            stop_arm_msg.points[0].positions.resize(7);
            copy(cur_joint_pos.begin(), cur_joint_pos.end(), 
                 stop_arm_msg.points[0].positions.begin());
            stop_arm_msg.points[0].velocities.resize(7);
            stop_arm_msg.points[0].accelerations.resize(7);
            arm_stop_pub.publish(stop_arm_msg);
            */
        }
    }

    /**
     * Heartbeat monitor which makes certain that contact is always established with
     * the collision detection nodes throughout the detection time.
     */
    void CollisionMonitor::spinMonitor() {
        ros::Rate r(100);
        while(ros::ok()) {
            ros::spinOnce();
            if(detection_on) {
                if(joint_on && ros::Time::now().toSec() - joint_last_time > 0.15) {
                    ROS_INFO("[collision_monitor] Joint detection timeout, stopping...");
                    collisionProcess();
                }
                if(finger_on && ros::Time::now().toSec() - finger_last_time > 0.15) {
                    ROS_INFO("[collision_monitor] Finger detection timeout, stopping...");
                    collisionProcess();
                }
                if(force_on && ros::Time::now().toSec() - force_last_time > 0.15) {
                    ROS_INFO("[collision_monitor] Force detection timeout, stopping...");
                    collisionProcess();
                }
            }
            // publish a simple boolean at 100hz which is only false when detection is
            // running and no collision is detected
            std_msgs::Bool heartbeat_msg; heartbeat_msg.data = !detection_on;
            heartbeat_pub.publish(heartbeat_msg);
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

