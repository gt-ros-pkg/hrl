#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("hrl_pr2_arms")
import rospy
import rosparam
from std_msgs.msg import Bool

from arm_pose_move_controller import ArmPoseMoveController
from arm_pose_move_controller import CTRL_NAME_LOW, CTRL_NAME_NONE, PARAMS_FILE_LOW, PARAMS_FILE_NONE
from msg import ArmPoseMoveCmd

HEARTBEAT_TOPIC = '/arm_pose_move_gui/heartbeat'
COMMAND_TOPIC = '/arm_pose_move_gui/command'
MONITOR_RATE = 20
POSE_TRAJ_PARAM_FILE = 'pose_traj_dir.yaml'
POSE_TRAJ_PARAM_PREFIX = '$(find kelsey_sandbox)/params/'

class ArmPoseMoveGuiBackend(object):
    def __init__(self):
        params = rosparam.get_param("/arm_pose_move")
        self.pose_dict = params['poses']
        self.traj_dict = params['trajectories']
        self.apm_ctrl = ArmPoseMoveController(CTRL_NAME_LOW, PARAMS_FILE_LOW)
        rospy.Subscriber(HEARTBEAT_TOPIC, Bool, self.heartbeat_cb)
        rospy.Subscriber(COMMAND_TOPIC, ArmPoseMoveCmd, self.cmd_cb)

    def heartbeat_cb(self, msg):
        pass

    def cmd_cb(self, msg):
        if msg.type == msg.START:
            if not self.apm_ctrl.is_moving():
                if msg.is_trajectory:
                    filename = self.traj_dict[msg.traj_name]['file']
                    filepath = roslib.substitution_args.resolve_args(POSE_TRAJ_PARAM_PREFIX + filename)
                    if msg.is_setup:
                        result = self.apm_ctrl.move_to_setup_from_file(filepath, 
                                                     reverse=not msg.is_forward, blocking=False)
                    else:
                        result = self.apm_ctrl.exec_traj_from_file(filepath, 
                                                     reverse=not msg.is_forward, blocking=False)
                else:
                    filename = self.pose_dict[msg.traj_name]['file']
                    filepath = roslib.substitution_args.resolve_args(POSE_TRAJ_PARAM_PREFIX + filename)
        elif msg.type == msg.STOP:
            self.apm_ctrl.pause_moving()
        elif msg.type == msg.RESET:
            self.apm_ctrl.stop_moving()
        else:
            rospy.logerror("[arm_pose_move_backend] Bad command.")

def main():
    rospy.init_node("arm_pose_move_backend")
    apm_backend = ArmPoseMoveGuiBackend()
    rospy.spin()
    

if __name__ == "__main__":
    main()
