#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
import rospy

import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

rospy.init_node("test_controller")
arm = create_pr2_arm('l', PR2ArmHybridForce)
arm.use_auto_update(True)
rospy.sleep(0.1)
arm.zero_sensor()
arm.reset_ep()
t, R = arm.get_ep()
arm.set_ep((t, R), 1)
q = arm.get_joint_angles()
arm.set_posture(q.tolist()[0:3] + 4 * [9999])


