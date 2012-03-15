#include "kelsey_sandbox/impulse_filter.h"
#include <pluginlib/class_list_macros.h>

namespace kelsey_sandbox {

    ImpulseFilter::ImpulseFilter() : pr2_controller_interface::Controller(), 
                                     cur_torque(Joints),
                                     cur_force(6),
                                     integ_force(6),
                                     delay_integ(6) {
    }

    /// Controller initialization in non-realtime
    bool ImpulseFilter::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n) {
        rosrt::init();
        node_ = n;
        robot_state_ = robot;

        std_msgs::Float64 float_temp;
        pub_val_.initialize(n, "/blah", 10, 10, float_temp);

        geometry_msgs::Wrench wrench_temp;
        pub_impulse_.initialize(n, "/impulse", 10, 10, wrench_temp);

        kelsey_sandbox::ImpulseState state_temp;
        state_temp.q.resize(Joints);
        state_temp.q_dot.resize(Joints);
        state_temp.effort.resize(Joints);
        state_temp.jacobian.data.resize(6*Joints);
        state_temp.jacobian.layout.dim.resize(2);
        state_temp.inertia.data.resize(Joints*Joints);
        state_temp.inertia.layout.dim.resize(2);
        state_temp.cart_inertia.data.resize(6*6);
        state_temp.cart_inertia.layout.dim.resize(2);
        pub_state_.initialize(n, "/impulse_state", 10, 10, state_temp);

        mean_force_filter_.reset(new filters::MultiChannelFilterChain<double>("double"));
        delay_force_filter_.reset(new filters::MultiChannelFilterChain<double>("double"));
        if(!mean_force_filter_->configure(6, "/impulse/mean_force_filter"))
            return false;
        if(!delay_force_filter_->configure(6, "/impulse/delay_force_filter"))
            return false;
            

        if(!chain_.init(robot, "torso_lift_link", "r_gripper_tool_frame"))
            return false;

        chain_.toKDL(kdl_chain_);
        kin_.reset(new Kin<Joints>(kdl_chain_));
        loop_count = 0;

        joint_vel_filter_ = 0.2;
        
        return true;
    }


    /// Controller startup in realtime
    void ImpulseFilter::starting() {
        //qdot_filtered_.setZero();
        cd_filter_ = 0.0;
        cd_integral_ = 0.0;
        //last_cycle_time_ = robot_state_->getTime();
    }


    /// Controller update loop in realtime
    void ImpulseFilter::update() {
        JointVec q;
        chain_.getPositions(q);

        JointVec qdot_raw;
        chain_.getVelocities(qdot_raw);
        for (int i = 0; i < Joints; ++i)
            qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
        JointVec qdot = qdot_filtered_;
        qdot = qdot_raw;

        Eigen::eigen2_Transform3d x;
        kin_->fk(q, x);

        Jacobian J;
        kin_->jac(q, J);

        CartVec x_dot = J * qdot;

        KDL::JntArray kdl_tau;
        kdl_tau.resize(Joints);
        chain_.getEfforts(kdl_tau);
        Eigen::MatrixXd tau = Eigen::MatrixXd::Zero(Joints, 1);
        for(int i=0;i<Joints;i++) {
            tau(i) = kdl_tau(i);
            cur_torque[i] = kdl_tau(i);
        }
        Eigen::MatrixXd cur_force_eig, JT;
        JT = J.transpose();
        cur_force_eig = JT.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tau);
        for(int i=0;i<6;i++) 
            cur_force[i] = cur_force_eig(i);
        
        mean_force_filter_->update(cur_force, integ_force);
        delay_force_filter_->update(integ_force, delay_integ);
        CartVec impulse;
        for(int i=0;i<6;i++) 
            impulse(i) = integ_force[i] - delay_integ[i];

        InertiaMat M;
        kin_->inertia(q, M);

        Eigen::MatrixXd JMinvJT = J * M.inverse() * J.transpose();
        Eigen::MatrixXd M_cart = JMinvJT.inverse();

        ros::Duration dt = robot_state_->getTime() - last_cycle_time_;
        last_cycle_time_ = robot_state_->getTime();
        //cd_integral_ += dt.toSec() * (qdot.transpose() * tau + cd_filter_);
        double E = 0.5 * qdot.transpose() * M * qdot + 0.1 * qdot.norm();
        cd_filter_ = 50.0 * (E - cd_integral_);

        ros::Time now_time = robot_state_->getTime();
        kelsey_sandbox::ImpulseState::Ptr state_msg;
        if (state_msg = pub_state_.allocate()) {  
            state_msg->header.stamp = now_time;
            state_msg->header.frame_id = "torso_lift_link";
            for (int i = 0; i < Joints; ++i) {
                state_msg->q[i] = q(i);
                state_msg->q_dot[i] = qdot(i);
                state_msg->effort[i] = tau(i);
            }
            state_msg->x.header.stamp = now_time;
            state_msg->x.header.frame_id = "torso_lift_link";
            tf::poseEigenToMsg(x, state_msg->x.pose);
            tf::twistEigenToMsg(x_dot, state_msg->x_dot);
            tf::wrenchEigenToMsg(cur_force_eig, state_msg->f_effort);
            tf::matrixEigenToMsg(J, state_msg->jacobian);
            tf::matrixEigenToMsg(M, state_msg->inertia);
            tf::matrixEigenToMsg(M_cart, state_msg->cart_inertia);
            pub_state_.publish(state_msg);
        }

        if(loop_count % 10 == 0) {
            std_msgs::Float64::Ptr val_msg;
            if (val_msg = pub_val_.allocate()) {  
                val_msg->data = impulse(0);
                pub_val_.publish(val_msg);
            }
            geometry_msgs::Wrench::Ptr impulse_msg;
            if (impulse_msg = pub_impulse_.allocate()) {  
                impulse_msg->force.x = impulse(0);
                impulse_msg->force.y = impulse(1);
                impulse_msg->force.z = impulse(2);
                impulse_msg->torque.x = impulse(3);
                impulse_msg->torque.y = impulse(4);
                impulse_msg->torque.z = impulse(5);
                pub_impulse_.publish(impulse_msg);
            }
        }
        loop_count++;
    }


    /// Controller stopping in realtime
    void ImpulseFilter::stopping() {}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(kelsey_sandbox,ImpulseFilter, 
                        kelsey_sandbox::ImpulseFilter, 
                        pr2_controller_interface::Controller)
