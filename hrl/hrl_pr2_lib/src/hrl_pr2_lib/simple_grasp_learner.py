#! /usr/bin/python
import numpy as np, math
import sys
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib

from hrl_lib.util import save_pickle, load_pickle
from hrl_lib.transforms import rotX, rotY, rotZ
from hrl_pr2_lib.pr2_arms import PR2Arms

node_name = "simple_grasp_learner" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

SETUP_POS_ANGS = []
ARM = 0 # right arm
NUM_X = 4
NUM_Y = 4
NUM_N = 6
RECT = ((0.35, -0.35), (0.8, 0.4))
GRASP_DIST = 0.30 

def get_xy_list():
    grasp_xy_list = []
    for x in np.linspace(RECT[0][0], RECT[1][0], NUM_X):
        for y in np.linspace(RECT[0][1], RECT[1][1], NUM_Y):
            grasp_xy_list += [(x,y)]

def save_setup_pos():
    arms = PR2Arms()

    grasp_xy_list = get_xy_list()
    grasp_configs = []

    for xy in grasp_xy_list:
        # Move to setup position in the middle of the grasp space
        arms.set_joint_angles(ARM, SETUP_POS_ANGS, 4.)
        arms.wait_for_arm_completion(ARM)
        rospy.sleep(2.)

        if arms.can_move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.))
            print "Moving to pos (%1.2f, %1.2f)" % xy
            arms.move_arm(ARM, [xy[0], xy[1], HOVER_Z], rotY(np.pi / 2.), 4.)
            arms.wait_for_arm_completion(ARM)
            rospy.sleep(2.)
            angs = arms.get_joint_angles(ARM)
            grasp_configs += [(xy, angs)]
        else:
            print "Can't move to to pos (%1.2f, %1.2f)" % xy
            grasp_configs += [(xy, None)]

    save_pickle(grasp_configs, GRASP_CONFIGS_FILE)
    print "Configurations:", grasp_configs

# def grasp_learn():
#     slat = LinearArmTrajectory()
# 
# 
#     for xy in grasp_xy_list:
#         # Move to setup position in the middle of the grasp space
#         q = load_pickle(SETUP_POS_PICKLE)
#         slat.set_joint_angles(ARM, q, 3.)
#         rospy.sleep(4.5)
# 
#         # Do grasping num_n times
#         for i in range(num_n):
# 
#             # Move arm to current grasp position
#             self.slat.move_arm(ARM, [xy[0], xy[1], hover_z], 2.)
#             self.slat.wait_for_arm_completion(ARM)
#             rospy.sleep(1.5)
#         
#             # TODO Begin monitoring sensors
#             
#             grasp_dist = hover_z - table_z
#             # move arm down
#             dur = self.slat.smooth_linear_move_arm(arm, grasp_dist, (0.,0.,-1.),  max_jerk=0.5)
#             self.slat.wait_for_arm_completion(arm)
#             rospy.sleep(1.5)
# 
#             # TODO End monitoring sensors
# 
#             # move arm up
#             dur = self.slat.smooth_linear_move_arm(arm, grasp_dist, (0.,0.,1.),  max_jerk=0.5)
#             self.slat.wait_for_arm_completion(arm)
#             rospy.sleep(1.5)
# 
#         # TODO Find statistics for sensors
# 
#     # TODO save statistics for sensors
            

def main():
    rospy.init_node(node_name)
    save_setup_pos()
    # grasp_learn()
    return 0

if __name__ == "__main__":
    sys.exit(main())
    
