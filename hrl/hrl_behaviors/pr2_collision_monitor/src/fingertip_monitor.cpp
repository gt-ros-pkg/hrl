#include <numeric>
#include <math.h>
#include <boost/lambda/lambda.hpp>

#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>
#include <std_msgs/Float32.h>
#include <pr2_collision_monitor/FingerState.h>
#include <pr2_collision_monitor/FingerStateSrv.h>

#define bl boost::lambda

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    class FIRDiffFilter {
        public:
            std::vector<double> history;
            int step;
            double coll_sig;
            FIRDiffFilter() : history(100), step(0), coll_sig(0.0) {
            }
            bool updateState(double z_obs) {
                double avg = std::accumulate(history.begin(), history.end(), 0.0) / 100.0;
                coll_sig = 0.01 * (coll_sig + z_obs - avg);
                history[step%100] = z_obs;
                step++;
                return std::fabs(coll_sig) > 0.4;
            }
    };

    class FingertipMonitor {
        public:
            ros::NodeHandle nh;
            std::vector<boost::shared_ptr<FIRDiffFilter> > r_finger_filters;
            std::vector<boost::shared_ptr<FIRDiffFilter> > l_finger_filters;
            std::string arm;
            ros::Subscriber pressure_sub;
            ros::Publisher coll_state_pub;
            ros::ServiceServer coll_state_srv;
            std::vector<std::vector<double> > r_fing_vals, l_fing_vals;
            FingerState::Ptr cur_state;
            bool training_mode;
            
            FingertipMonitor();
            void onInit();
            void pressureCallback(const pr2_msgs::PressureState& msg); 
            void computeStats();
            bool srvCallback(FingerStateSrv::Request& req, FingerStateSrv::Response& resp);
    };

    FingertipMonitor::FingertipMonitor() : nh("~"), r_fing_vals(21), l_fing_vals(21) {
    }

    void FingertipMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));

        for(uint32_t i=0;i<21;i++) {
            r_finger_filters.push_back(shared_ptr<FIRDiffFilter>(new FIRDiffFilter()));
            l_finger_filters.push_back(shared_ptr<FIRDiffFilter>(new FIRDiffFilter()));
        }

        pressure_sub = nh.subscribe("/pressure/" + arm + "_gripper_motor", 2, 
                                    &FingertipMonitor::pressureCallback, this);
        coll_state_pub = nh.advertise<FingerState>(arm + "_collision_state", 1);
        coll_state_srv = nh.advertiseService(arm + "_state_request",
                                             &FingertipMonitor::srvCallback, this);
    }

    void FingertipMonitor::pressureCallback(const pr2_msgs::PressureState& msg) {
        FingerState::Ptr state(new FingerState()); 
        state->r_finger_tip.resize(21); state->l_finger_tip.resize(21);
        for(uint32_t i=0;i<21;i++) {
            bool r_in_collision = r_finger_filters[i]->updateState(msg.r_finger_tip[i]);
            if(r_in_collision) {
                state->r_finger_tip[i] = true;
                state->any_collision = true;
            }
            bool l_in_collision = l_finger_filters[i]->updateState(msg.l_finger_tip[i]);
            if(l_in_collision) {
                state->l_finger_tip[i] = true;
                state->any_collision = true;
            }
        }
        coll_state_pub.publish(state);
        cur_state = state;
    }

    bool FingertipMonitor::srvCallback(FingerStateSrv::Request& req, 
                                     FingerStateSrv::Response& resp) {
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


