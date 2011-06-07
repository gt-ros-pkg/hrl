#include <numeric>
#include <math.h>
#include <boost/foreach.hpp>
#include <boost/math/distributions/students_t.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pr2_collision_monitor/JointErrorData.h>
#include <pr2_collision_monitor/JointDetectionStart.h>

using namespace std;

namespace pr2_collision_monitor {

    class JointCollDetect {
        public:
            JointCollDetect();
            void onInit();
            void writeErrorData();
            bool isTraining() { return training_mode; }
            ~JointCollDetect(); 

        protected:
            ros::NodeHandle nh;
            ros::NodeHandle nh_priv;
            std::string arm, behavior_name, data_filename;

            bool monitoring_collisions, training_mode, significance_mode, collision_detected;
            double start_time, end_time;
            vector<double> min_errors, max_errors;
            vector<float> cur_min_data, cur_max_data;
            JointErrorData error_data;
            vector<std::string> behavior_name_list;
            vector<vector<vector<float> > > total_min_data, total_max_data;

            bool startDetection(std::string&, float specificity);
            void stopDetection();
            bool triggerCollision();
            void errorCallback(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr message);
            bool srvStartDetection(JointDetectionStart::Request&, 
                                   JointDetectionStart::Response&);
            bool srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            bool srvTriggerCollision(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
            void loadErrorBag(const std::string& load_filename, 
                              JointErrorData::Ptr& err_data_ptr);
            void loadAllErrorData(vector<std::string>& filename_list);

            ros::Publisher detect_pub;
            ros::Subscriber error_sub;
            ros::ServiceServer start_srv, stop_srv, trig_srv;
    };

    JointCollDetect::JointCollDetect() : nh_priv("~"),
                                           min_errors(7),
                                           max_errors(7),
                                           cur_min_data(7),
                                           cur_max_data(7) {
        onInit();
    }

    void JointCollDetect::onInit() {
        nh_priv.param<std::string>("arm", arm, std::string("r"));

        // training_mode does not detect collisions but instead monitors
        // normal operation and collects statistics on joint errors for each behavior
        // execution
        nh_priv.param<bool>("training_mode", training_mode, false);
        // significance_mode employs the statistics used in training
        // if this mode is false, manual threshold must be provided through
        // the parameter server
        nh_priv.param<bool>("significance_mode", significance_mode, false);
        if(training_mode) {
            // training to find suitable thresholds
            nh_priv.param<std::string>("behavior_name", behavior_name, 
                                                        std::string("collision_behavior"));
            nh_priv.param<std::string>("data_filename", data_filename, 
                                                        std::string("joint_error_data"));
            error_data.arm = arm;
            error_data.behavior = behavior_name;
        } else {
            // loading thresholds
            if(significance_mode) {
                // load training data and can use specificity values
                // to determine thresholds
                std::string filename_prefix;
                nh_priv.getParam("filename_prefix", filename_prefix);
                vector<std::string> filename_list;
                XmlRpc::XmlRpcValue xml_filenames;
                if(nh_priv.hasParam("filename_list")) {
                    nh_priv.getParam("filename_list", xml_filenames);
                    for(int i=0;i<xml_filenames.size();i++) {
                        filename_list.push_back(filename_prefix + 
                                                static_cast<std::string>(xml_filenames[i]));
                    }
                } else {
                    ROS_ERROR("[joint_coll_detect] MUST PROVIDE FILENAMES IN significance_mode (filename_list)");
                    ros::shutdown();
                    return;
                }
                loadAllErrorData(filename_list);
            } else {
                // load from manually set thresholds
                XmlRpc::XmlRpcValue xml_min_errors, xml_max_errors;
                if(nh_priv.hasParam("min_errors")) {
                    nh_priv.getParam("min_errors", xml_min_errors);
                    for(int i=0;i<xml_min_errors.size();i++)
                        min_errors[i] = static_cast<double>(xml_min_errors[i]);
                } else {
                    ROS_ERROR("[joint_coll_detect] MUST PROVIDE THRESHOLDS (min_errors)");
                    ros::shutdown();
                    return;
                }
                if(nh_priv.hasParam("max_errors")) {
                    nh_priv.getParam("max_errors", xml_max_errors);
                    for(int i=0;i<xml_max_errors.size();i++)
                        max_errors[i] = static_cast<double>(xml_max_errors[i]);
                } else {
                    ROS_ERROR("[joint_coll_detect] MUST PROVIDE THRESHOLDS (max_errors)");
                    ros::shutdown();
                    return;
                }
            }
        }

        monitoring_collisions = false;

        detect_pub = nh_priv.advertise<std_msgs::Bool>("arm_collision_detected", 1);
        ROS_INFO("[joint_coll_detect] Publishing on arm_collision_detected");
        start_srv = nh_priv.advertiseService("start_detection", 
                                             &JointCollDetect::srvStartDetection, this);
        ROS_INFO("[joint_coll_detect] Service advertised at start_detection");
        stop_srv = nh_priv.advertiseService("stop_detection", 
                                            &JointCollDetect::srvStopDetection, this);
        ROS_INFO("[joint_coll_detect] Service advertised at stop_detection");
        trig_srv = nh_priv.advertiseService("trigger_collision", 
                                            &JointCollDetect::srvTriggerCollision, this);
        ROS_INFO("[joint_coll_detect] Service advertised at trigger_collision");

        error_sub = nh.subscribe(arm + "_arm_controller/state", 2, 
                &JointCollDetect::errorCallback, this);
        ROS_INFO("[joint_coll_detect] JointCollDetect loaded.");
    }

    float minus_squared(float a, float b, float c) { return a + (b-c)*(b-c); }

    bool JointCollDetect::startDetection(std::string& behavior, float sig_level) {
        if(!monitoring_collisions) {
            collision_detected = false;

            if(significance_mode) {
                // load the behavior requested
                uint32_t behavior_ind = std::find(behavior_name_list.begin(), 
                                                  behavior_name_list.end(),
                                                  behavior) - behavior_name_list.begin();
                if(behavior_ind == behavior_name_list.size() || 
                                             sig_level > 1 || sig_level < 0) {
                    ROS_WARN("[joint_coll_detect] Behavior %s not loaded (bad parameters)!", 
                                                           behavior.c_str());
                    return false;
                }

                // set the thresholds using the statistical properties of the 
                // training data
                for(int i=0;i<7;i++) {
                    // Perform a prediction interval on the training data
                    // using a Student's t-test
                    int Sn = total_min_data[behavior_ind][i].size();
                    float Sm_min = std::accumulate(total_min_data[behavior_ind][i].begin(),
                                                   total_min_data[behavior_ind][i].end(), 0.0) / Sn;
                    float Sm_max = std::accumulate(total_max_data[behavior_ind][i].begin(),
                                                   total_max_data[behavior_ind][i].end(), 0.0) / Sn;
                    boost::function<float(float, float)> minus_squared_bind;
                    minus_squared_bind = boost::bind(&minus_squared, _1, _2, Sm_min);
                    float Sd_min = std::sqrt(std::accumulate(
                                                   total_min_data[behavior_ind][i].begin(),
                                                   total_min_data[behavior_ind][i].end(),
                                                   0.0, minus_squared_bind) / Sn);
                    minus_squared_bind = boost::bind(&minus_squared, _1, _2, Sm_max);
                    float Sd_max = std::sqrt(std::accumulate(
                                                   total_max_data[behavior_ind][i].begin(),
                                                   total_max_data[behavior_ind][i].end(),
                                                   0.0, minus_squared_bind) / Sn);
                    boost::math::students_t st_dist(Sn-1);
                    float T = boost::math::quantile(st_dist, sig_level);
                    float thresh_min = Sm_min - T * Sd_min * std::sqrt(1 + 1.0/Sn);
                    float thresh_max = Sm_max + T * Sd_max * std::sqrt(1 + 1.0/Sn);
                    min_errors[i] = thresh_min;
                    max_errors[i] = thresh_max;
                    //ROS_INFO("Sm_min: %f, Sm_max: %f, Sn: %d", Sm_min, Sm_max, Sn);
                    //ROS_INFO("Sd_min: %f, Sd_max: %f, Sn: %d", Sd_min, Sd_max, Sn);
                }
            }
            printf("Min thresh: [");
            for(int i=0;i<7;i++)
                printf("%1.3f, ", min_errors[i]);
            printf("]\nMax thresh: [");
            for(int i=0;i<7;i++)
                printf("%1.3f, ", max_errors[i]);
            printf("]\n");
            start_time = ros::Time::now().toSec();
            monitoring_collisions = true;
            ROS_INFO("[joint_coll_detect] Monitoring for collisions.");
            if(training_mode) {
                std::fill(cur_min_data.begin(), cur_min_data.end(), 10000);
                std::fill(cur_max_data.begin(), cur_max_data.end(), -10000);
            }
            return true;
        }
        return false;
    }

    void JointCollDetect::stopDetection() {
        if(monitoring_collisions) {
            end_time = ros::Time::now().toSec();
            monitoring_collisions = false;
            ROS_INFO("[joint_coll_detect] Stopping monitoring (time passed: %2.1f).", end_time-start_time);
            if(training_mode) {
                error_data.min_errors.insert(error_data.min_errors.end(), 
                                             cur_min_data.begin(), cur_min_data.end());
                error_data.max_errors.insert(error_data.max_errors.end(), 
                                             cur_max_data.begin(), cur_max_data.end());
                printf("Min data: [");
                for(int i=0;i<7;i++)
                    printf("%1.3f, ", cur_min_data[i]);
                printf("]\nMax data: [");
                for(int i=0;i<7;i++)
                    printf("%1.3f, ", cur_max_data[i]);
                printf("]\n");
            }
        }
    }

    bool JointCollDetect::srvStartDetection(JointDetectionStart::Request& req, 
                                             JointDetectionStart::Response& resp) {
        return startDetection(req.behavior, req.sig_level);
    }

    bool JointCollDetect::srvStopDetection(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
        stopDetection();
        return true;
    }

    bool JointCollDetect::triggerCollision() {
        if(!training_mode) {
            if(monitoring_collisions)
                stopDetection();
            std_msgs::Bool bool_true;
            bool_true.data = true;
            detect_pub.publish(bool_true);
            collision_detected = true;
            return true;
        }
        return false;
    }

    bool JointCollDetect::srvTriggerCollision(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        triggerCollision();
        return true;
    }

    void JointCollDetect::errorCallback(pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr message) {
        if(!monitoring_collisions || message->error.positions.size() < 7)
            return;

        for(int i=0;i<7;i++) {
            if(!training_mode) {
                if(message->error.positions[i] < min_errors[i] ||
                   message->error.positions[i] > max_errors[i]) {
                    if(triggerCollision())
                        ROS_INFO("[joint_coll_detect] Collision detected on joint %d. Min: %1.3f, Max: %1.3f, Cur: %1.3f", i, min_errors[i], max_errors[i], message->error.positions[i]);
                }
            } else {
                if(message->error.positions[i] < cur_min_data[i])
                    cur_min_data[i] = message->error.positions[i];
                if(message->error.positions[i] > cur_max_data[i])
                    cur_max_data[i] = message->error.positions[i];
            }
        }
        if(!collision_detected) {
            std_msgs::Bool bool_false;
            bool_false.data = false;
            detect_pub.publish(bool_false);
        }
    }

    void JointCollDetect::writeErrorData() {
        ROS_INFO("[joint_coll_detect] Writing error data to file.");
        rosbag::Bag data_bag;
        data_bag.open(data_filename, rosbag::bagmode::Write);
        data_bag.write("/error_data", ros::Time::now(), error_data);
        data_bag.close();
        ROS_INFO("[joint_coll_detect] Bag file written.");
    }

    void JointCollDetect::loadErrorBag(const std::string& load_filename, 
                                         JointErrorData::Ptr& err_data_ptr) {
        rosbag::Bag data_bag;
        data_bag.open(load_filename, rosbag::bagmode::Read);
        rosbag::View view(data_bag, rosbag::TopicQuery("/error_data"));
        if(view.size() == 0 || view.size() > 1) {
            ROS_ERROR("[joint_coll_detect] Badly formed error_data file (%s)", load_filename.c_str());
            ros::shutdown();
            return;
        }
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            err_data_ptr = m.instantiate<JointErrorData>();
        }
    }

    void JointCollDetect::loadAllErrorData(vector<std::string>& filename_list) {
        total_min_data.resize(filename_list.size()); total_max_data.resize(filename_list.size());
        behavior_name_list.resize(filename_list.size());
        int beh_ind = 0;
        BOOST_FOREACH(std::string const filename, filename_list) {
            JointErrorData::Ptr err_data_ptr;
            loadErrorBag(filename, err_data_ptr);
            total_min_data[beh_ind].resize(7); total_max_data[beh_ind].resize(7);
            uint32_t data_pt_ind = 0;
            while(data_pt_ind < err_data_ptr->min_errors.size()) {
                for(int i=0;i<7;i++) {
                    total_min_data[beh_ind][i].push_back(err_data_ptr->min_errors[data_pt_ind]);
                    total_max_data[beh_ind][i].push_back(err_data_ptr->max_errors[data_pt_ind]);
                    data_pt_ind++;
                }
            }
            behavior_name_list[beh_ind] = err_data_ptr->behavior;
            beh_ind++;
        }
    }

    JointCollDetect::~JointCollDetect() {
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_coll_detect", ros::init_options::AnonymousName);
    pr2_collision_monitor::JointCollDetect cm;
    ros::spin();
    if(cm.isTraining()) 
        cm.writeErrorData();
    return 0;
}

