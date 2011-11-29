#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_playpen')
import rospy
import math
import tf
import numpy as np

if __name__ == '__main__':
    rospy.init_node('get_gripper_position')

    listener = tf.TransformListener()

    rate = rospy.Rate(100.0)
    
    right = False
    left = False

    for item in rospy.get_published_topics():
        for thing in item:
            if '/r_overhead_grasp/feedback' in thing:
                right = True
            elif '/l_overhead_grasp/feedback' in thing:
                left = True

    if left == True:
        prefix = 'l_'
    elif right == True:
        prefix = 'r_'

    frame1 = prefix+'gripper_l_finger_tip_link'
    frame2 = prefix+'gripper_r_finger_tip_link'

    #######next step is to put into loop to run through each bag file and then to put in pickle file####
    #f_hand = open(bag_path,  
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            dist = math.sqrt((np.matrix(trans)*np.matrix(trans).T)[0,0])
            time = rospy.get_time()
            print "distance is :", dist, "time is :", time

        except (tf.LookupException, tf.ConnectivityException):
            print "didn't get anything yet"
            continue
        

        rate.sleep()
