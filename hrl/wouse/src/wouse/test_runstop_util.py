#!/usr/bin/env python

from sys import exit

import roslib; roslib.load_manifest('wouse')
import rospy
from run_stop_util import RunStopUtil

run_stop = RunStopUtil()
if run_stop.init_successful:
    rospy.loginfo("Run-stop successfully initialized")
else:
    rospy.logwarn("Run-stop initialization failed!")
    exit()

rospy.sleep(2.0)

print "Run-stopping"
run_stop.run_stop()

print "Run-stopped"
rospy.sleep(5.0)

print "Run-starting"
run_stop.run_start()

print "Run-started"




