
#include <boost/scoped_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Dense>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <rosrt/rosrt.h>
#include <hrl_netft/HybridForceState.h>

#define Joints 7

namespace hrl_netft
{

typedef Eigen::Matrix<double, Joints, 1> JointVec;
typedef Eigen::Matrix<double, 6, 1> CartVec;
typedef Eigen::Matrix<double, 6, Joints> Jacobian;
typedef Eigen::Matrix<double, Joints, Joints> JointMat;
typedef Eigen::Matrix<double, 6, 6> CartMat;

struct Kin {

    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jac_solver_;
    //KDL::ChainDynParam dyn_param_solver_;
    Eigen::LLT<JointMat> llt_solver_;

    KDL::JntArray kdl_q;
    KDL::Jacobian kdl_J;
    //KDL::JntSpaceJointMatrix kdl_M;

    Kin(const KDL::Chain &kdl_chain) :
        fk_solver_(kdl_chain), jac_solver_(kdl_chain),
        //dyn_param_solver_(kdl_chain, KDL::Vector::Zero()),
        llt_solver_(Joints),
        kdl_q(Joints), kdl_J(Joints) { //, kdl_M(Joints) {
    }

    ~Kin() {
    }

    void fk(const JointVec &q, Eigen::Affine3d &x) {
        kdl_q.data = q;
        KDL::Frame kdl_x;
        fk_solver_.JntToCart(kdl_q, kdl_x);
        for(int i=0;i<3;i++)
            x(i, 2) = kdl_x.p[i];
        for(int i=0;i<3;i++)
            for(int j=0;j<3;j++)
                x(i, j) = kdl_x.M(i, j);
        for(int i=0;i<3;i++)
            x(3, i) = 0.0;
        x(3, 3) = 1.0;
    }

    void jac(const JointVec &q, Jacobian &J) {
        kdl_q.data = q;
        jac_solver_.JntToJac(kdl_q, kdl_J);
        J = kdl_J.data;
    }
/*
    void inertia(const JointVec &q, JointMat &M) {
        kdl_q.data = q;
        dyn_param_solver_.JntToMass(kdl_q, kdl_M);
        for(int i=0;i<7;i++)
            for(int j=0;j<7;j++)
                M(i,j) = kdl_M(i,j);
    }

    void cartInertia(const JointVec &q, CartMat &C) {
        JointMat H;
        inertia(q, H);

        llt_solver_.compute(H);
        Jacobian J;
        jac(q, J);
        Jacobian JL = J * llt_solver_.matrixL().inverse().transpose();
        CartMat JLLJT = JL * JL.transpose();
        C = JLLJT.inverse();
    }
*/
};

class HybridForceController : public pr2_controller_interface::Controller {
    public:
        HybridForceController();
        ~HybridForceController();
        bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh);
        void starting();
        void update();

    private:
        //! Keep track of max force magnitude seen by sensor
        double max_force_;
        //! Keep track of max torque magnitude seen by sensor
        double max_torque_;

        //! Point to AnalogIn struct that provide force-torque data
        pr2_hardware_interface::AnalogIn *analog_in_;

        //! publish max_force values every X realtime cycles
        int pub_cycle_count_;
        bool should_publish_;

        /////////////////////////////////////////////////////////////////
        std::string base_link, netft_link, tool_link;

        pr2_mechanism_model::Chain netft_chain_, tool_chain_;
        KDL::Chain netft_kdl_chain_, tool_kdl_chain_;
        boost::scoped_ptr<Kin> netft_kin_, tool_kin_;

        CartVec ft_wrench_;
        CartMat shifting_mat_;

        realtime_tools::RealtimePublisher<hrl_netft::HybridForceState> pub_state_;
};

void eigenToWrench(const CartVec& eig_w, geometry_msgs::Wrench& ros_w);

}//namespace hrl_netft
