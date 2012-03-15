
#include <stdio.h>
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_inertia");
    KDL::Tree my_tree;
    ros::NodeHandle node;
    string robot_desc_string;
    node.param("robot_description", robot_desc_string, string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    KDL::Chain r_arm_chain;
    my_tree.getChain("torso_lift_link", "r_gripper_palm_link", r_arm_chain);
    ROS_INFO("%d\n", r_arm_chain.getNrOfJoints());
    KDL::Vector g(0, 0, 1);
    KDL::ChainDynParam dyn_params(r_arm_chain, g);
    KDL::JntArray q(7);
    ROS_INFO("%f\n", q.data[0]);
    KDL::JntSpaceInertiaMatrix H(7);
    dyn_params.JntToMass(q, H);
    ROS_INFO("%d %d\n", H.columns(), H.rows());
    for(int i=0;i<7;i++) {
        for(int j=0;j<7;j++)
            std::printf("%f, ", H(i,j));
        std::printf("\n");
    }
}
