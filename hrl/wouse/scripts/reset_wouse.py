#!/usr/bin/python

import sys

import roslib; roslib.load_manifest('wouse')
import rospy
from wouse.srv import WouseRunStop, WouseRunStopRequest

rospy.init_node('wouse_reset')
try:
    rospy.wait_for_service('/wouse_run_stop', 10)
    reset_client = rospy.ServiceProxy('wouse_run_stop', WouseRunStop)
    rospy.loginfo("[Wouse Reset]: Found wouse run stop service.")
except: 
    rospy.logwarn("[Wouse Reset]: Could not find wouse run stop service.")
    sys.exit()

req = WouseRunStopRequest()
req.start = True
success = reset_client(req)

if success:
    rospy.loginfo("[Wouse Reset]: Reset returned successfully.")
else:
    rospy.logwarn("[Wouse Reset]: Reset reported failure.")
