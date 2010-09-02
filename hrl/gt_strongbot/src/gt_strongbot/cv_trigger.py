import roslib
roslib.load_manifest('trigger_msgs')
import sys
import rospy
import cv

import trigger_msgs.msg

rospy.init_node("trigger", anonymous=True)
arm_trigger = rospy.Publisher("arm_trigger", trigger_msgs.msg.Trigger, latch = True)
head_trigger = rospy.Publisher("head_trigger", trigger_msgs.msg.Trigger, latch = True)
cv.NamedWindow('keyboard', 1)
img = cv.CreateImage((30, 30), cv.IPL_DEPTH_8U, 1)
#r = rospy.Rate(132/60.)
r = rospy.Rate(10.)

i = 0
while not rospy.is_shutdown():
    cv.ShowImage('keyboard', img)
    k = cv.WaitKey(10)
    #print (k & 0xff), k
    if chr(k & 0xff) == 'h':
        print 'head!'
        head_trigger.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))

    if chr(k & 0xff) == 'a':
        print 'arm!'
        arm_trigger.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))

    #if i % 4 == 0:

    #if i % 8 == 0:

    #i = i+1
    #r.sleep()
