#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_omni_teleop')

import rospy
import tf
import numpy as np
import math
import actionlib
from pr2_controllers_msgs.msg import *
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
from geometry_msgs.msg import Point

class TrackGrippers:

    def __init__(self):
        self.head_client = actionlib.SimpleActionClient(\
                'head_traj_controller/point_head_action',\
                PointHeadAction)
        self.tflistener = tf.TransformListener()

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                common_link = '/base_link'
                c_T_rgrip = tfu.transform(common_link, '/r_gripper_tool_frame', self.tflistener)
                c_T_lgrip = tfu.transform(common_link, '/l_gripper_tool_frame', self.tflistener)
                gripper_right_c = np.matrix(tr.translation_from_matrix(c_T_rgrip * tr.translation_matrix([0, 0, 0.])))
                gripper_left_c  = np.matrix(tr.translation_from_matrix(c_T_lgrip * tr.translation_matrix([0, 0, 0.])))
                look_at_point_c = ((gripper_right_c + gripper_left_c) / 2.0).A1.tolist()

                g = PointHeadGoal()
                g.target.header.frame_id = '/base_link'
                g.target.point = Point(*look_at_point_c)
                g.min_duration = rospy.Duration(1.0)
                g.max_velocity = 10.
                self.head_client.send_goal(g)
                self.head_client.wait_for_result(rospy.Duration(1.))
                r.sleep()
            except tf.LookupException, e:
                print e


if __name__ == '__main__':
    rospy.init_node('teleop_track_gripper')
    tg = TrackGrippers()
    tg.run()
