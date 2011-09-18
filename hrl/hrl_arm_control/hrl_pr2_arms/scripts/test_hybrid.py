
import numpy as np

import roslib 
roslib.load_manifest("hrl_pr2_arms")
import rospy
import rosparam
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJointTrajectory, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

rospy.init_node("ipython_node")
arm = create_pr2_arm('l', PR2ArmHybridForce)
# motion gains p - proportional, d - derivative
#              t - translational, r - rotational
kpt, kpr, kdt, kdr = 400, 100, 10, 1 
kfpt, kfpr, kfdt, kfdr = 2, 1, 10, 0.5 # force gains (derivative damps velocity)
force_selector = [1, 0, 0, 0, 0, 0] # control force in x direction, motion in others
arm.set_motion_gains(kpt, kpr, kdt, kdr)
arm.set_force_gains(kfpt, kfpr, kfdt, kfdr)
arm.update_gains()
# force desired is currently at 0

if False:
    arm.set_force([1, 0, 0, 0, 0, 0]) 
    # apply 1N in x direction (can only command values in x currently)
