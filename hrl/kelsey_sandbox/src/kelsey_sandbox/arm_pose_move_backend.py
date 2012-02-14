#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("hrl_pr2_arms")
import rospy
from std_msgs.msg import Bool

from arm_pose_move_controller import ArmPoseMoveController
from arm_pose_move_controller import CTRL_NAME_LOW, CTRL_NAME_NONE, PARAMS_FILE_LOW, PARAMS_FILE_NONE
from msg import ArmPoseMoveCmd

HEARTBEAT_TOPIC = '/arm_pose_move_gui/heartbeat'
COMMAND_TOPIC = '/arm_pose_move_gui/command'
MONITOR_RATE = 20

class ArmPoseMoveGuiBackend(object):
    def __init__(self):
        self.apm_ctrl = ArmPoseMoveController(CTRL_NAME_LOW, PARAMS_FILE_LOW)
        rospy.Subscriber(HEARTBEAT_TOPIC, Bool, self.heartbeat_cb)
        rospy.Subscriber(COMMAND_TOPIC, ArmPoseMoveCmd, self.cmd_cb)

    def heartbeat_cb(self, msg):
        pass

    def cmd_cb(self, msg):
        if msg.type == msg.START:
            if not self.apm_ctrl.is_moving():
                print "start"
                return
                result = self.apm_ctrl.exec_traj_from_file(filename, reverse=not msg.is_forward, blocking=False)
        elif msg.type == msg.STOP:
            print "stop"
        elif msg.type == msg.RESET:
            print "reset"
        else:
            rospy.logerror("[arm_pose_move_backend] Bad command.")

def main():
    apm_backend = ArmPoseMoveGuiBackend()
    rospy.spin()
    

if __name__ == "__main__":
    main()
