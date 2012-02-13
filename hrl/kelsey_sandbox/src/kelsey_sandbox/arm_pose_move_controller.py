#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("hrl_pr2_arms")
import rospy
from std_msgs.msg import String
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory, PR2ArmCartesianPostureBase
from arm_cart_vel_control import PR2ArmCartVelocityController
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

RATE = 20

class ArmPoseMoveController(object):
    def __init__(self, arm, ctrl_name, param_file):
        self.arm = arm
        self.ctrl_name = ctrl_name
        self.param_file = param_file
        self.cur_joint_traj = None

    def execute_trajectory(self, joint_trajectory, rate, blocking=True):
        if self.running:
            rospy.logerror("[arm_pose_move_controller] Trajectory already in motion.")
            return
        if self.cur_joint_traj is None:
            self.cur_joint_traj = joint_trajectory
            self.cur_idx = 0
        ctrl_switcher.carefree_switch(self.arm.arm, self.ctrl_name, self.param_file, reset=False)
        rospy.sleep(0.2)
        r = rospy.Rate(rate)
        self.stop_traj = False
        def exec_traj(te):
            self.running = True
            while not rospy.is_shutdown() and not self.stop_traj:
                self.arm.set_ep(self.cur_joint_traj[self.cur_idx], 1./rate)
                self.cur_idx += 1
                r.sleep()
            self.arm.set_ep(self.arm.get_ep(), 0.3)
            self.running = False
        if blocking:
            exec_traj(None)
        else:
            self.exec_traj_timer = rospy.Timer(rospy.Duration(0.1), exec_traj, oneshot=True)

    def pause_moving(self):
        self.stop_traj = True

    def stop_moving(self):
        self.stop_traj = True
        while not rospy.is_shutdown() and self.running:
            rospy.sleep(0.01)
        self.cur_joint_traj = None

class TrajectorySaver(object):
    def __init__(self, arm, rate):
        self.arm = arm
        self.rate = rate

    def record_trajectory(self):
        r = rospy.Rate(self.rate)
        self.traj = []
        self.stop_recording = False
        self.is_recording = True
        def rec_traj(te):
            rospy.loginfo("[arm_pose_move_controller] Recording trajectory.")
            while not rospy.is_shutdown() and not self.stop_recording:
                q = arm.get_ep().copy()
                self.traj.append(q)
            rospy.loginfo("[arm_pose_move_controller] Stopped recording trajectory.")
            self.is_recording = False
        self.rec_traj_timer = rospy.Timer(rospy.Duration(0.1), rec_traj, oneshot=True)

    def stop_record(self, save_file):
        self.stop_recording = True
        while not rospy.is_shutdown() and self.is_recording:
            rospy.sleep(0.1)
        print self.traj

def main():
    r_arm = create_pr2_arm('r', PR2ArmJointTrajectory)
    traj_saver = TrajectorySaver(r_arm, RATE)
    traj_saver.record_trajectory()
    rospy.sleep(4)
    traj_saver.stop_record("")
    return
    if False:
        r_arm = create_pr2_arm('r', PR2ArmJointTrajectory)
        r_arm.set_posture()
        rospy.sleep(0.2)
        arm_pm_ctrl = ArmPoseMoveController(r_arm,
                                            '%s_joint_controller_low', 
                                            '$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml')
        arm_pm_ctrl.execute_trajectory(jt, RATE)

if __name__ == "__main__":
    main()
