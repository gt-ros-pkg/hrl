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

MONITOR_RATE = 20.
MOVE_STATE_TOPIC = "/arm_control_gui/move_state"
LOAD_ARM_TOPIC = "/arm_control_gui/load_arm"

class ArmCartCtrlBackend(object):
    def __init__(self, monitor_rate, misses_allowed=3):
        self.misses_allowed = misses_allowed
        rospy.Subscriber(MOVE_STATE_TOPIC, String, self.move_state_cb)
        rospy.Subscriber(LOAD_ARM_TOPIC, String, self.load_arm_cb)
        self.is_move_connected = False
        self.last_move_time = 0.
        self.misses = 0

    def move_state_cb(self, msg):
        self.is_move_connected = True
        self.last_move_time = rospy.get_time()
        self.misses = 0

    def load_arm_cb(self, msg):
        pass

    def check_state(self):
        if rospy.get_time() - self.last_move_time > 1. / MONITOR_RATE:
            self.misses += 1
            if self.misses > self.misses_allowed:
                self.is_move_connected = False
        return self.is_move_connected

def main():
    rospy.init_node("arm_cart_control_backend")
    r = rospy.Rate(MONITOR_RATE)
    cart_ctrl = ArmCartCtrlBackend(MONITOR_RATE)
    while not rospy.is_shutdown():
        if cart_ctrl.check_state():
            print "Connected"
        r.sleep()

if __name__ == "__main__":
    main()
