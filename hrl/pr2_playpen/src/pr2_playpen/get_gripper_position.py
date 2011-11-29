#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_playpen')
import rospy
import math
import tf
import numpy as np
import os
import sys
import cPickle as pkl

def is_topic_pub(topic):
    flag = False
    for item in rospy.get_published_topics():
        for thing in item:
            if topic in thing:
                flag = True
            else:
                pass
    return flag

def get_data(listener, rate):
    right = False
    left = False

    dist_ls = []
    time_ls = []

    while is_topic_pub('/r_overhead_grasp/feedback') == False and is_topic_pub('/l_overhead_grasp/feedback') == False:
        print "waiting for bag file"
        rate.sleep()

    if is_topic_pub('/r_overhead_grasp/feedback'):
        right = True
    elif is_topic_pub('/l_overhead_grasp/feedback'):
        left = True

    if left == True:
        prefix = 'l_'
    elif right == True:
        prefix = 'r_'

    frame1 = prefix+'gripper_l_finger_tip_link'
    frame2 = prefix+'gripper_r_finger_tip_link'

    #run = True
    while is_topic_pub('/clock') == True: #run == True:#not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            dist = math.sqrt((np.matrix(trans)*np.matrix(trans).T)[0,0])
            time = rospy.get_time()
            dist_ls.append(dist)
            time_ls.append(time)

        except (tf.LookupException, tf.ConnectivityException):
            #run = False
            continue
        

        rate.sleep()

    return dist_ls, time_ls

if __name__ == '__main__':
    rospy.init_node('get_gripper_position')
    listener = tf.TransformListener()            
    rate = rospy.Rate(100.0)
    
    path = sys.argv[1]
    print "path is :", path

    for i in xrange(9):
        j = 0
        dist_dict = {}
        f_hand = open(path+'/object'+str(i).zfill(3)+'_gripper_dist.pkl', 'w')
        while os.path.isfile(path + '/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'.bag') == True: #j < 999:
            f_path = path + '/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'.bag'
            os.system('rosbag play -r 2 '+ f_path + ' &')
            dist_ls, time_ls = get_data(listener, rate)
            dist_dict['try'+str(j).zfill(3)] = {'dist':dist_ls, 'time':time_ls}                
            j = j+1
        pkl.dump(dist_dict, f_hand)
        f_hand.close()
