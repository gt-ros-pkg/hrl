import roslib
roslib.load_manifest('trigger_msgs')
import sys
import rospy

import trigger_msgs.msg

rospy.init_node("trigger", anonymous=True)
head_up = rospy.Publisher("head_up", trigger_msgs.msg.Trigger, latch = True)
head_down = rospy.Publisher("head_down", trigger_msgs.msg.Trigger, latch = True)
arm_on = rospy.Publisher("arm_on", trigger_msgs.msg.Trigger, latch = True)
arm_off = rospy.Publisher("arm_off", trigger_msgs.msg.Trigger, latch = True)
r = rospy.Rate(60/60.)

i = 1
while not rospy.is_shutdown():
    print '------------', i, '-------------'

    if i > 4:
        if i % 2 == 1:
            #Down
            print 'down'
            head_down.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))

        if i % 2 == 0:
            #Up
            print 'up'
            head_up.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))

    if i % 4 == 1:
        arm_on.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))

    if i % 4 == 3:
        arm_off.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 0.0)), rospy.get_param("~event", ""))


    i = i+1
    r.sleep()
