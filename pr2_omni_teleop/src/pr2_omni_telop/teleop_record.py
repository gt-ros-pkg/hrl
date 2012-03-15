#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_omni_teleop')

import math
import numpy as np
import hrl_lib.tf_utils as tfu
import pdb

import rospy
import tf
import tf.transformations as tr


class TeleopRecord:

    def __init__(self):
        self.tflistener = tf.TransformListener()
        tfu.wait('map', 'base_footprint', self.tflistener, 4.)

    def print_pose(self):
        try:
            tfu.wait('map', 'base_footprint', self.tflistener, 1.)
            m_T_b = tfu.transform('map', 'base_footprint', self.tflistener)
            t, q = tfu.matrix_as_tf(m_T_b)

            print m_T_b
            print 't', t, 'q', q

        except Exception, e:
            print e

if __name__ == '__main__':
    rospy.init_node('teleop_record')
    t = TeleopRecord()
    t.print_pose()
