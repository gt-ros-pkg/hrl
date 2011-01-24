
import sys
import numpy as np, math

import add_cylinder as ac
import online_collision_detection as ocd

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import rospy
from mapping_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker

import hrl_lib.viz as hv

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa


if __name__ == '__main__':
    rospy.init_node('force_visualize_test')
    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    while not rospy.is_shutdown():
        f = pr2_arms.get_wrist_force(arm)
        print 'force:', f.A1
        rospy.sleep(0.1)









