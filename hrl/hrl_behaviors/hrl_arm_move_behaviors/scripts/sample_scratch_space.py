#! /usr/bin/python
import random
import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy
import actionlib
import tf
import tf.transformations as tf_trans
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray

from hrl_arm_move_behaviors.touch_face import TouchFace
import hrl_arm_move_behaviors.util as util

def main():
    rospy.init_node('test_linear_move')
    touch_pub = rospy.Publisher('touch_pose', PoseStamped)
    touch_arr_pub = rospy.Publisher('touch_poses', PoseArray)
    touch_face = TouchFace('r')
    rospy.sleep(5)
    param_bounds = [(0.76, 1.03),
                    (-0.05, -0.41),
                    (-0.30, 0.32),
                    (-60.0, 30.0),
                    (-20.0, 45.0)]
    touch_arr = PoseArray()
    while not rospy.is_shutdown():
        params = []
        for pmin, pmax in param_bounds:
            params.append(np.random.uniform(pmin, pmax))
        pos = params[:3]
        quat = tf_trans.quaternion_from_euler(np.radians(params[3]), np.radians(params[4]), 
                                              0, 'rzyx')
        touch_pose = PoseStamped()
        touch_pose.header.frame_id = 'torso_lift_link'
        touch_pose.header.stamp = rospy.Time.now()
        touch_pose.pose = Pose(Point(*pos), Quaternion(*quat))
        touch_pub.publish(touch_pose)
        touch_arr.header = touch_pose.header
        touch_arr.poses.append(touch_pose.pose)
        touch_arr_pub.publish(touch_arr)
        print touch_pose

        touch_face.touch_face_at_pose(touch_pose)
        rospy.sleep(1)

if __name__ == '__main__':
    main()
