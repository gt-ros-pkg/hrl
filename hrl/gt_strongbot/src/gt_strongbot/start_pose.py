import roslib
roslib.load_manifest('trigger_msgs')
import sys
import rospy

import trigger_msgs.msg

rospy.init_node("trigger", anonymous=True)

left_initial_pose00 = rospy.Publisher("left_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
right_initial_pose00 = rospy.Publisher("right_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
head_initial_pose00 = rospy.Publisher("head_initial_pose00", trigger_msgs.msg.Trigger, latch = True)
#                                      head_initial_pose00

r = rospy.Rate(60/60.)
head_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 1)), rospy.get_param("~event", ""))
left_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 10)), rospy.get_param("~event", ""))
right_initial_pose00.publish(rospy.get_rostime() + rospy.Duration(rospy.get_param("~delay", 10)), rospy.get_param("~event", ""))
r.sleep()
r.sleep()
r.sleep()

