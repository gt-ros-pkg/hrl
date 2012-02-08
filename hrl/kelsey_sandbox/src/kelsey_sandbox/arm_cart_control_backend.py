#! /usr/bin/python

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
import rospy
from std_msgs.msg import String

MOVE_BUTTONS = {'translate_up' : [0, 1], 'translate_down' : [0, -1],
                'translate_left' : [1, 1], 'translate_right' : [1, -1],
                'translate_in' : [2, 1], 'translate_out' : [2, -1],
                'rotate_x_pos' : [3, 1], 'rotate_x_neg' : [3, -1],
                'rotate_y_pos' : [4, 1], 'rotate_y_neg' : [4, -1],
                'rotate_z_pos' : [5, 1], 'rotate_z_neg' : [5, -1]}

POSE_PARAMS = ['position.x', 'position.y', 'position.z', 
               'orientation.x', 'orientation.y', 'orientation.z']

MONITOR_RATE = 20

class ArmCartController(object):
    def __init__(self):
        rospy.Subscriber("/arm_control_gui/move_state", String, self.move_state_cb)
        rospy.Subscriber("/arm_control_gui/load_arm", String, self.load_arm_cb)

    def move_state_cb(self, msg):
        pass

    def load_arm_cb(self, msg):
        pass

def main():
    rospy.init_node("arm_cart_control_backend")

if __name__ == "__main__":
    main()
