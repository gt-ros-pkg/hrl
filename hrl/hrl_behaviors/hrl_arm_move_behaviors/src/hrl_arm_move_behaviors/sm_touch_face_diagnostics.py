#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import String

STATUS_DICT = {"MOVE_RETURN_SETUP" : "[Touch Face] Moving to setup position.",
               "WAIT_RETURN_SETUP_CLICK" : "[Touch Face] Setup move halted, click anywhere to resume",
               "WAIT_TOUCH_CLICK" : "[Touch Face] Click the desired touch location.",
               "COARSE_POSE" : "[Touch Face] Moving to the approach position using standard coarse IK.",
               "FINE_POSITION_SETUP" : "[Touch Face] Moving to approach position using fine EPC control.",
               "FINE_APPROACH" : "[Touch Face] Approaching touch location using fine EPC control.",
               "WAIT_RETREAT_CLICK" : "[Touch Face] Click anywhere to retreat from face.",
               "COARSE_RETREAT" : "[Touch Face] Retreating from face back to approach position."}

TALK_DICT = {  "MOVE_RETURN_SETUP" : "Moving to setup position.",
               "WAIT_RETURN_SETUP_CLICK" : "Setup move halted.",
               "WAIT_TOUCH_CLICK" : "Click touch location.",
               "COARSE_POSE" : "Coarse approach.",
               "FINE_POSITION_SETUP" : "Fine setup.",
               "FINE_APPROACH" : "Fine approach.",
               "WAIT_RETREAT_CLICK" : "Click to retreat.",
               "COARSE_RETREAT" : "Retreating from face."}

class TouchFaceDiagnostics(object):
    def __init__(self):
        rospy.Subscriber("/touch_face/smach/container_status", self.status_cb)
        self.html_status_pub = rospy.Publisher("/wt_log_out", String)
        self.talk_pub = rospy.Publisher("/text_to_say", String)
        self.cur_state = ""

    def status_cb(self, msg):
        for state in msg.active_states:
            if state == self.cur_state:
                continue
            if state in STATUS_DICT:
                self.html_status_pub.publish(STATUS_DICT[state])
                sel.cur_state = state
            if state in TALK_DICT:
                self.talk_pub.publish(TALK_DICT[state])
                sel.cur_state = state

def main():
    rospy.init_node("touch_face_diags")
    tfd = TouchFaceDiagnostics()
    rospy.spin()


if __name__ == "__main__":
    main()
