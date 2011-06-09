#include <numeric>
#include <math.h>
#include <boost/lambda/lambda.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "geometry_msgs/Vector3Stamped.h"
#include <pr2_collision_monitor/fir_diff_filter.h>

#define bl boost::lambda

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    class ForceTorqueMonitor {
        public:
            ros::NodeHandle nh;
            std::string arm;
            ros::Subscriber force_sub;
            ros::Publisher coll_state_pub, signal_pub;

            shared_ptr<FIRDiffFilter> force_x_filter, force_y_filter, force_z_filter;
            double threshold, time_constant;
            int history;
            
            ForceTorqueMonitor();
            void onInit();
            void forceCallback(const geometry_msgs::Vector3Stamped& msg); 
    };

    ForceTorqueMonitor::ForceTorqueMonitor() : nh("~") {
    }

    void ForceTorqueMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));
        nh.param<double>("force_threshold", threshold, 4.0);
        nh.param<double>("time_constant", time_constant, 0.01);
        nh.param<int>("history", history, 30);

        force_x_filter = shared_ptr<FIRDiffFilter>(new FIRDiffFilter(history, time_constant));
        force_y_filter = shared_ptr<FIRDiffFilter>(new FIRDiffFilter(history, time_constant));
        force_z_filter = shared_ptr<FIRDiffFilter>(new FIRDiffFilter(history, time_constant));

        coll_state_pub = nh.advertise<std_msgs::Bool>("collision_detected", 2);
        force_sub = nh.subscribe("/force_topic", 1, 
                              &ForceTorqueMonitor::forceCallback, this);
        signal_pub = nh.advertise<std_msgs::Float32>("force_signal", 2);
    }

    void ForceTorqueMonitor::forceCallback(const geometry_msgs::Vector3Stamped& msg) {
        double signal_x = force_x_filter->updateState(msg.vector.x);
        double signal_y = force_y_filter->updateState(msg.vector.y);
        double signal_z = force_z_filter->updateState(msg.vector.z);
        double sig_mag = sqrt(signal_x * signal_x + 
                              signal_y * signal_y + 
                              signal_z * signal_z); 

        bool in_collision = sig_mag > threshold;
        std_msgs::Bool state_msg;
        state_msg.data = in_collision;
        coll_state_pub.publish(state_msg);
        std_msgs::Float32 signal_msg;
        signal_msg.data = sig_mag;
        signal_pub.publish(signal_msg);
    }

};

using namespace pr2_collision_monitor;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_torque_monitor", ros::init_options::AnonymousName);
    ForceTorqueMonitor fm;
    fm.onInit();
    ros::spin();
    return 0;
}


