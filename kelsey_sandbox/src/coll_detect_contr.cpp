#include "kelsey_sandbox/coll_detect_contr.h"
#include <pluginlib/class_list_macros.h>

namespace cd_controller {

    /// Controller initialization in non-realtime
    bool CollisionDetection::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {
        rosrt::init();
        node_ = n;
        robot_state_ = robot;

        std_msgs::Float64 float_temp;
        pub_val_.initialize(n, "/blah", 10, 10, float_temp);

        if(!chain_.init(robot, "torso_lift_link", "r_gripper_palm_link"))
            return false;

        chain_.toKDL(kdl_chain_);
        kin_.reset(new Kin<Joints>(kdl_chain_));
        loop_count = 0;

        joint_vel_filter_ = 0.2;
        
        return true;
    }


    /// Controller startup in realtime
    void CollisionDetection::starting() {
        qdot_filtered_.setZero();
        cd_filter_ = 0.0;
        cd_integral_ = 0.0;
        last_cycle_time_ = robot_state_->getTime();
    }


    /// Controller update loop in realtime
    void CollisionDetection::update() {
        JointVec q;
        chain_.getPositions(q);

        JointVec qdot_raw;
        chain_.getVelocities(qdot_raw);
        for (int i = 0; i < Joints; ++i)
            qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
        JointVec qdot = qdot_filtered_;
        qdot = qdot_raw;

        KDL::JntArray kdl_tau;
        kdl_tau.resize(Joints);
        chain_.getEfforts(kdl_tau);
        JointVec tau;
        for(int i=0;i<Joints;i++)
            tau(i) = kdl_tau(i);


        InertiaMat M;
        kin_->inertia(q, M);

        ros::Duration dt = robot_state_->getTime() - last_cycle_time_;
        last_cycle_time_ = robot_state_->getTime();
        cd_integral_ += dt.toSec() * (qdot.transpose() * tau + cd_filter_);
        double E = 0.5 * qdot.transpose() * M * qdot + 0.1 * qdot.norm();
        cd_filter_ = 50.0 * (E - cd_integral_);

        if(loop_count % 10 == 0) {
            std_msgs::Float64::Ptr val_msg;
            if (val_msg = pub_val_.allocate()) {  
                val_msg->data = cd_filter_;
                pub_val_.publish(val_msg);
            }
        }
        loop_count++;
    }


    /// Controller stopping in realtime
    void CollisionDetection::stopping() {}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(kelsey_sandbox,CollDetect, 
                        cd_controller::CollisionDetection, 
                        pr2_controller_interface::Controller)
