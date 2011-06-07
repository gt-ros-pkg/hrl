#include <numeric>
#include <math.h>
#include <boost/lambda/lambda.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Vector3Stamped.h"

#define bl boost::lambda

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    /*
        TODO ADD OR REMOVE LATER IF NEEDED/UNECCESSARY TODO
    class FIRDiffFilter {
        public:
            std::vector<double> history;
            int step;
            double coll_sig, threshold;
            FIRDiffFilter() : history(15), step(0), coll_sig(0.0) {
            }
            bool updateState(double z_obs) {
                double avg = std::accumulate(history.begin(), history.end(), 0.0) / 15.0;
                coll_sig = 0.01 * (coll_sig + z_obs - avg);
                history[step%15] = z_obs;
                step++;
                return std::fabs(coll_sig) > threshold;
            }
    };
    */

    class ForceTorqueMonitor {
        public:
            ros::NodeHandle nh;
            std::string arm;
            ros::Subscriber force_sub;
            ros::Publisher coll_state_pub;

            double threshold;
            //FIRDiffFilter force_filter;
            
            ForceTorqueMonitor();
            void onInit();
            void forceCallback(const geometry_msgs::Vector3Stamped& msg); 
    };

    ForceTorqueMonitor::ForceTorqueMonitor() : nh("~") {
    }

    void ForceTorqueMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));
        nh.param<double>("threshold", threshold, 4.0);
        //force_filter.threshold = threshold;

        coll_state_pub = nh.advertise<std_msgs::Bool>("collision_state", 2);
        force_sub = nh.subscribe("/force_torque_ft1_Vec3", 1, 
                              &ForceTorqueMonitor::forceCallback, this);
    }

    void ForceTorqueMonitor::forceCallback(const geometry_msgs::Vector3Stamped& msg) {
        double mag = sqrt(msg.vector.x * msg.vector.x + 
                          msg.vector.y * msg.vector.y + 
                          msg.vector.z * msg.vector.z);
        //bool in_collision = force_filter.updateState(mag);
        bool in_collision = mag > threshold;
        std_msgs::Bool state_msg;
        state_msg.data = in_collision;
        coll_state_pub.publish(state_msg);
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


