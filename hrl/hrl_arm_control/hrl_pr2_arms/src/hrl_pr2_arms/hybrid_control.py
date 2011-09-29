#! /usr/bin/python

import copy
import numpy as np
import roslib
roslib.load_manifest('hrl_pr2_arms')
import rospy

from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

class HybridSurfaceControl(object):
    def __init__(self, arm):
        self.arm = arm

    def command_force_velocity(self, f, v, frame):
        arm = self.arm
        Kd = arm.trans_d_motion_gains[0]
        arm.set_tip_frame(frame)
        arm.set_force_directions(['x', 'z'])
        arm.update_gains()
        arm.set_force([Kd * v, 0, -f, 0, 0, 0])

    def command_stop(self):
        arm.set_force(6 * [0])
        rospy.sleep(0.3)
        arm.set_force_directions([])
        gains = copy.copy(arm.trans_p_motion_gains)
        gains[0] *= 0.1
        gains[2] *= 0.1
        arm.set_motion_gains(p_trans=gains)
        arm.set_ep(arm.get_end_effector_pose(), 1)
        arm.update_gains()
        rospy.sleep(0.3)
        gains[0] *= 10
        gains[2] *= 10
        arm.set_motion_gains(p_trans=gains)
        arm.update_gains()


if __name__ == "__main__":
    rospy.init_node("hybrid_control")
    arm = create_pr2_arm('l', PR2ArmHybridForce)
    kpt, kpr, kdt, kdr = 100, 10, 10, 1 
    kfpt, kfpr = 1, 0.3 # force gains 
    arm.set_motion_gains(kpt, kpr, kdt, kdr)
    arm.set_force_gains(kfpt, kfpr)
    arm.update_gains()

    arm.zero_sensor()
    hsc = HybridSurfaceControl(arm)
    rospy.sleep(1)

    arm.set_motion_gains(d_trans=[10, 10, 10])
    arm.update_gains()
    hsc.command_force_velocity(1.5, 0, "torso_lift_link")
    rospy.sleep(5)

    arm.set_motion_gains(d_trans=10)
    arm.update_gains()
    hsc.command_force_velocity(2, -0.5, "torso_lift_link")
    rospy.sleep(5)

    if False:
        arm.set_motion_gains(d_trans=[10, 10, 10])
        arm.update_gains()
        hsc.command_force_velocity(-2, 0, "torso_lift_link")
        rospy.sleep(5)

    hsc.command_stop()
    rospy.sleep(0.3)
    t, R = arm.get_end_effector_pose()
    arm.set_ep((t + np.mat([0, 0, 0.05]).T, R), 1)
