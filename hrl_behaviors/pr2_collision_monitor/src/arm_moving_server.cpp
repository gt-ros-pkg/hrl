#include <numeric>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <pr2_collision_monitor/ArmMovingWait.h>

using namespace std;

namespace pr2_collision_monitor {

    class ArmMovingWaitServer {
        public:
            ArmMovingWaitServer();
            void onInit();
            ~ArmMovingWaitServer(); 

        protected:
            ros::NodeHandle nh;
            ros::NodeHandle nh_priv;
            ros::Publisher moving_pub;
            std::string arm;
            bool arm_is_moving;
            vector<vector<float> > joint_pos_past;
            int step;
            bool srvCheckArmMoving(ArmMovingWait::Request&, ArmMovingWait::Response&);
            void stateCallback(sensor_msgs::JointState::ConstPtr);
            ros::Subscriber state_sub;
            ros::ServiceServer moving_srv;
    };

    ArmMovingWaitServer::ArmMovingWaitServer() : nh_priv("~"),
                                                 arm_is_moving(true),
                                                 joint_pos_past(7),
                                                 step(0) {
        for(int i=0;i<7;i++)
            joint_pos_past[i].resize(10);
        onInit();
    }

    void ArmMovingWaitServer::onInit() {
        nh_priv.param<std::string>("arm", arm, std::string("r"));
        moving_srv = nh_priv.advertiseService("arm_moving_wait", 
                                              &ArmMovingWaitServer::srvCheckArmMoving, this);
        ROS_INFO("[arm_moving_server] Service advertised at arm_moving_wait");
        state_sub = nh.subscribe("/joint_states", 2, 
                &ArmMovingWaitServer::stateCallback, this);
        moving_pub = nh_priv.advertise<std_msgs::Bool>("arm_moving", 1);
    }

    bool ArmMovingWaitServer::srvCheckArmMoving(ArmMovingWait::Request& req, ArmMovingWait::Response& resp) {
        if(!req.block) {
            resp.is_moving = arm_is_moving;
            return true;
        }
        ros::Rate r(100);
        double start_time = ros::Time::now().toSec();
        while(ros::ok() && arm_is_moving && 
              ros::Time::now().toSec() - start_time < req.timeout) {
            ros::spinOnce();
            r.sleep();
        }
        resp.is_moving = arm_is_moving;
        return true;
    }

    int JOINTSTATE_INDS_R[] = {17, 18, 16, 20, 19, 21, 22};
    int JOINTSTATE_INDS_L[] = {29, 30, 28, 32, 31, 33, 34};

    void ArmMovingWaitServer::stateCallback(
            sensor_msgs::JointState::ConstPtr msg) {

        bool arm_still_moving = false;
        for(int i=0;i<7;i++) {
            int msg_ind;
            if(arm == "r")
                msg_ind = JOINTSTATE_INDS_R[i];
            else
                msg_ind = JOINTSTATE_INDS_L[i];
            joint_pos_past[i][step%10] = msg->position[msg_ind];
            if(step < 10)
                continue;
            for(int j=0;j<10;j++) 
                if(std::fabs(msg->position[msg_ind] - joint_pos_past[i][j]) > 0.001) {
                    arm_still_moving = true;
            }
        }
        arm_is_moving = arm_still_moving;
        std_msgs::Bool moving_msg;
        moving_msg.data = arm_is_moving;
        moving_pub.publish(moving_msg);
        step++;
    }

    ArmMovingWaitServer::~ArmMovingWaitServer() {
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_moving_server", ros::init_options::AnonymousName);
    pr2_collision_monitor::ArmMovingWaitServer amws;
    ros::spin();
    return 0;
}

