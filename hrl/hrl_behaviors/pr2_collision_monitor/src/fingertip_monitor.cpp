#include <numeric>
#include <math.h>
#include <boost/lambda/lambda.hpp>

#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <pr2_collision_monitor/FingerState.h>
#include <pr2_collision_monitor/GetFingerState.h>
#include <pr2_collision_monitor/fir_diff_filter.h>

#define bl boost::lambda

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    class FingertipMonitor {
        public:
            ros::NodeHandle nh;
            std::vector<boost::shared_ptr<FIRDiffFilter> > r_finger_filters;
            std::vector<boost::shared_ptr<FIRDiffFilter> > l_finger_filters;
            std::string arm;
            ros::Subscriber pressure_sub;
            ros::Publisher coll_state_pub, coll_bool_pub, mag_sig_pub;
            ros::ServiceServer coll_state_srv;
            std::vector<std::vector<double> > r_fing_vals, l_fing_vals;
            FingerState::Ptr cur_state;
            bool training_mode;
            double threshold;
            
            FingertipMonitor();
            void onInit();
            void pressureCallback(const pr2_msgs::PressureState& msg); 
            void computeStats();
            bool srvCallback(GetFingerState::Request& req, GetFingerState::Response& resp);
    };

    FingertipMonitor::FingertipMonitor() : nh("~"), r_fing_vals(21), l_fing_vals(21) {
    }

    void FingertipMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));
        nh.param<double>("sensor_threshold", threshold, 2.0);
        double time_constant;
        int history;
        nh.param<double>("time_constant", time_constant, 0.01);
        nh.param<int>("history", history, 30);

        // initialize the filters
        for(uint32_t i=0;i<21;i++) {
            r_finger_filters.push_back(
                    shared_ptr<FIRDiffFilter>(new FIRDiffFilter(history, time_constant)));
            l_finger_filters.push_back(
                    shared_ptr<FIRDiffFilter>(new FIRDiffFilter(history, time_constant)));
        }

        // subscriber to the pressure topic
        pressure_sub = nh.subscribe("/pressure/" + arm + "_gripper_motor", 2, 
                                    &FingertipMonitor::pressureCallback, this);
        // publish a FingerState message which details the collision state of all sensors
        // in that arm
        // TODO KEEP/REMOVE FINGERSTATE?
        //coll_state_pub = nh.advertise<FingerState>("collision_state", 2);
        // publish a simple bool summarizing whether any sensor has collided
        coll_bool_pub = nh.advertise<std_msgs::Bool>("collision_detected", 2);
        mag_sig_pub = nh.advertise<std_msgs::Float32>("finger_signal", 2);
        // accessor for the current collision state of the sensor
        //coll_state_srv = nh.advertiseService("state_request",
                                             //&FingertipMonitor::srvCallback, this);
    }

    /**
     * Update the filters and set the state of the collision detection
     */
    void FingertipMonitor::pressureCallback(const pr2_msgs::PressureState& msg) {
        // update the filters and set the state of the collision detection
        //FingerState::Ptr state(new FingerState()); 
        //state->r_finger_tip.resize(21); state->l_finger_tip.resize(21);
        double max_sig = 0;
        for(uint32_t i=0;i<21;i++) {
            double r_sig = r_finger_filters[i]->updateState(msg.r_finger_tip[i]);
            max_sig = max(r_sig*r_sig, max_sig);
            double l_sig = l_finger_filters[i]->updateState(msg.l_finger_tip[i]);
            max_sig = max(l_sig*l_sig, max_sig);
        }
        //coll_state_pub.publish(state);
        //cur_state = state;
        double signal = max_sig;
        std_msgs::Float32 signal_msg;
        signal_msg.data = signal;
        mag_sig_pub.publish(signal_msg);

        // publish a simple bool if the signal has exceeded the set threshold
        std_msgs::Bool coll_bool_msg;
        coll_bool_msg.data = signal > threshold;
        coll_bool_pub.publish(coll_bool_msg);
    }

    bool FingertipMonitor::srvCallback(GetFingerState::Request& req, 
                                       GetFingerState::Response& resp) {
        resp.state = *cur_state;
        return true;
    }

};

using namespace pr2_collision_monitor;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fingertip_monitor", ros::init_options::AnonymousName);
    FingertipMonitor fm;
    fm.onInit();
    ros::spin();
    return 0;
}


