#!/usr/bin/env python  
import os
import roslib
roslib.load_manifest('pr2_playpen')
roslib.load_manifest('tf_conversions')
import rospy
import math
import tf
import sys
import tf_conversions.posemath as pm
import numpy as np
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('playpen_calibration')

    listener = tf.TransformListener()
    trans_list = []
    rot_list = []

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(sys.argv[1], sys.argv[2], rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform(sys.argv[3], sys.argv[4], rospy.Time(0))
            msg1 = Pose()
            msg2 = Pose()
            msg1.position.x, msg1.position.y, msg1.position.z = trans[0], trans[1], trans[2]
            msg2.position.x, msg2.position.y, msg2.position.z = trans2[0], trans2[1], trans2[2]
            msg1.orientation.x, msg1.orientation.y, msg1.orientation.z, msg1.orientation.w = rot[0], rot[1], rot[2], rot[3]
            msg2.orientation.x, msg2.orientation.y, msg2.orientation.z, msg2.orientation.w = rot2[0], rot2[1], rot2[2], rot2[3]
            (trans_tot, rot_tot) = pm.toTf(pm.fromMsg(msg1)*pm.fromMsg(msg2))
            print "translation: ", trans_tot, ", rotation :", rot_tot
            trans_list.append(trans_tot)
            rot_list.append(rot_tot)
            
        except (tf.LookupException, tf.ConnectivityException):
            continue
        rate.sleep()
    trans_str = str(np.median(np.array(trans_list), axis = 0))
    rot_str = str(np.median(np.array(rot_list), axis = 0))
    print "median of translation :", trans_str
    print "median of rotation :", rot_str
    
    try:
        os.remove('../../launch/kinect_playpen_to_torso_lift_link.launch')
    except:
        print 'no file to be removed, creating new file'
    f = open('../../launch/kinect_playpen_to_torso_lift_link.launch', 'w')
    f.write('<launch>\n')
    ############################################ I really need to verify the order of sys.arg[4] and sys.arg[1], this could be wrong!!! best way to check 
    ############################################ is to transform something from argv[4] frame to argv[1] frame and check
    f.write('<node pkg="tf" type="static_transform_publisher" name="kinect_playpen_to_pr2_lift_link" args=" '+trans_str[1:-1]+' '+rot_str[1:-1]+' '+sys.argv[4]+' '+sys.argv[1]+' 100" />\n')
    f.write('</launch>')
    f.close()

#run this to calibrate playpen, I should include it in a launch file with arguments and launching pr2_launch/ar_kinect and launch/ar_kinect
#./calibrate.py  /torso_lift_link /calibration_pattern /calibration_pattern2 /openni_camera2
