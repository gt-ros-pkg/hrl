#! /usr/bin/python

import numpy as np
import sys

import roslib
roslib.load_manifest('hrl_pr2_arms')
import rospy
import actionlib
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory

joint_deltas = [0.01, 0.01, 0.01, 0.012, 0.01, 0.01, 0.01]

def main():
    rospy.init_node("mirror_setup")

    if sys.argv[1] in ['both', 'point']:
        print "Pointing head"
        head_point_sac = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',
                                                      PointHeadAction)
        head_point_sac.wait_for_server()
        head_goal = PointHeadGoal()
        head_goal.target = PoseConverter.to_point_stamped_msg('/torso_lift_link',
                                                              np.mat([1., 0.4, 0.]).T,
                                                              np.mat(np.eye(3)))
        head_goal.target.header.stamp = rospy.Time()
        head_goal.min_duration = rospy.Duration(3.)
        head_goal.max_velocity = 0.2
        head_point_sac.send_goal_and_wait(head_goal)

    if sys.argv[1] in ['both', 'mirror']:
        ctrl_switcher = ControllerSwitcher()
        ctrl_switcher.carefree_switch('r', 'r_joint_controller_mirror', 
                                      "$(find hrl_ellipsoidal_control)/params/mirror_params.yaml")
        arm = create_pr2_arm('r', PR2ArmJointTrajectory, controller_name="r_joint_controller_mirror")
        arm.set_ep([-0.26880036055585677, 0.71881299774143248, -0.010187938126536471, -1.43747589322259, -12.531293698878677, -0.92339874393497123, 3.3566322715405432], 5)
        rospy.sleep(6)

        gripper_sac = actionlib.SimpleActionClient(
                                     '/r_gripper_controller/gripper_action',
                                     Pr2GripperCommandAction)
        gripper_sac.wait_for_server()
        ggoal = Pr2GripperCommandGoal()
        ggoal.command.position = 0.03
        ggoal.command.max_effort = 30.0
        print "Gripper opening"
        rospy.sleep(1)
        gripper_sac.send_goal_and_wait(ggoal)
        print "Gripper closing"
        rospy.sleep(3)
        ggoal2 = Pr2GripperCommandGoal()
        ggoal2.command.position = 0.0
        ggoal2.command.max_effort = -1.0
        gripper_sac.send_goal_and_wait(ggoal2)

        r = rospy.Rate(10)
        q_act_last = arm.get_joint_angles()
        while not rospy.is_shutdown():
            q_act = arm.get_joint_angles()
            q_ep = arm.get_ep()
            new_ep = q_ep.copy()
            for i in range(7):
                if np.fabs(q_act[i] - q_act_last[i]) > joint_deltas[i]:
                    new_ep[i] = q_act[i]
            arm.set_ep(new_ep, 0.1)
            q_act_last = q_act
            r.sleep()


if __name__ == "__main__":
    main()
