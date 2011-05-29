#include <numeric>
#include <math.h>
#include <boost/lambda/lambda.hpp>

#include <ros/ros.h>
#include <pr2_msgs/PressureState.h>
#include <std_msgs/Float32.h>

#define bl boost::lambda

using namespace std;
using namespace boost;

namespace pr2_collision_monitor {

    class SimpleKalmanFilter {
        public:
            double Q_var, P_var, R_var, x_state;
            bool is_first;
            SimpleKalmanFilter(double Q, double R) {
                Q_var = Q; R_var = R; is_first = true;
            }
            void updateState(double z_obs, double& cur_x, double& cur_P) {
                if(is_first) {
                    x_state = z_obs;
                    P_var = R_var;
                    is_first = false;
                } else {
                    P_var = P_var + Q_var;
                    double K = P_var / (P_var + R_var);
                    x_state += K * (z_obs - x_state);
                    P_var += -K * P_var;
                }
                cur_x = x_state; cur_P = P_var;
            }
    };

    class FingertipMonitor {
        public:
            ros::NodeHandle nh;
            std::vector<boost::shared_ptr<SimpleKalmanFilter> > r_finger_filters;
            std::vector<boost::shared_ptr<SimpleKalmanFilter> > l_finger_filters;
            std::string arm;
            ros::Subscriber pressure_sub;
            ros::Publisher x_pub, P_pub;
            std::vector<std::vector<double> > r_fing_vals, l_fing_vals;
            bool training_mode;
            
            FingertipMonitor();
            void onInit();
            void pressureCallback(const pr2_msgs::PressureState& msg); 
            void computeStats();
    };

    FingertipMonitor::FingertipMonitor() : nh("~"), r_fing_vals(22), l_fing_vals(22) {
    }

    void FingertipMonitor::onInit() {
        nh.param<std::string>("arm", arm, std::string("r"));
        nh.param<bool>("training_mode", training_mode, false);
        std::vector<double> r_vars, l_vars;
        XmlRpc::XmlRpcValue xml_r_vars, xml_l_vars;
        if(nh.hasParam("r_fing_variances")) {
            nh.getParam("r_fing_variances", xml_r_vars);
            for(int i=0;i<xml_r_vars.size();i++)
                r_vars.push_back(static_cast<double>(xml_r_vars[i]));
        } else {
            ROS_ERROR("[fingertip_monitor] Missing variance values.");
            ros::shutdown();
            return;
        }
        if(nh.hasParam("l_fing_variances")) {
            nh.getParam("l_fing_variances", xml_l_vars);
            for(int i=0;i<xml_l_vars.size();i++)
                l_vars.push_back(static_cast<double>(xml_l_vars[i]));
        } else {
            ROS_ERROR("[fingertip_monitor] Missing variance values.");
            ros::shutdown();
            return;
        }

        for(uint32_t i=0;i<22;i++) {
            r_finger_filters.push_back(shared_ptr<SimpleKalmanFilter>(
                                         new SimpleKalmanFilter(r_vars[i], r_vars[i])));
        }

        pressure_sub = nh.subscribe("/pressure/" + arm + "_gripper_motor", 2, 
                                    &FingertipMonitor::pressureCallback, this);
        x_pub = nh.advertise<std_msgs::Float32>("x_state", 1);
        P_pub = nh.advertise<std_msgs::Float32>("P_state", 1);
    }

    void FingertipMonitor::pressureCallback(const pr2_msgs::PressureState& msg) {
        for(uint32_t i=0;i<1;i++) {
            double cur_x, cur_P;
            r_finger_filters[i]->updateState(msg.r_finger_tip[i], cur_x, cur_P);
            std_msgs::Float32 x_msg, P_msg; x_msg.data = cur_x; P_msg.data = cur_P;
            x_pub.publish(x_msg);
            P_pub.publish(P_msg);

            if(training_mode) {
                r_fing_vals[i].push_back(msg.r_finger_tip[i]);
                l_fing_vals[i].push_back(msg.l_finger_tip[i]);
            }
        }
    }

    void FingertipMonitor::computeStats() {

        printf("\nRight finger variances:\n[");
        for(uint32_t i=0;i<22;i++) {
            double r_avg = accumulate(r_fing_vals[i].begin(), 
                                      r_fing_vals[i].end(), 0.0) / r_fing_vals[i].size();

            double r_var = accumulate(r_fing_vals[i].begin(),
                                      r_fing_vals[i].end(), 0.0, 
                                      bl::_1 += (bl::_2-r_avg)*(bl::_2-r_avg)) /
                                  r_fing_vals[i].size();
            printf("%f, ", r_var);
        }
        printf("]\nLeft finger variances:\n[");
        for(uint32_t i=0;i<22;i++) {
            double l_avg = accumulate(l_fing_vals[i].begin(), 
                                      l_fing_vals[i].end(), 0.0) / l_fing_vals[i].size();

            double l_var = accumulate(l_fing_vals[i].begin(),
                                      l_fing_vals[i].end(), 0.0, 
                                      bl::_1 += (bl::_2-l_avg)*(bl::_2-l_avg)) /
                                  l_fing_vals[i].size();
            printf("%f, ", l_var);
        }
        printf("]\n");
    }

};

using namespace pr2_collision_monitor;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fingertip_monitor", ros::init_options::AnonymousName);
    FingertipMonitor fm;
    fm.onInit();
    ros::spin();
    if(fm.training_mode)
        fm.computeStats();
    return 0;
}


