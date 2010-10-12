import numpy as np, math
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib

from hrl_lib.utils import 
from hrl_lib.rutils import 

node_name = "simple_grasp_learner" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

SETUP_POS_PICKLE = "pickles//init_pos_r.pickle"
ARM = 0 # right arm

def grasp_learn(arm=ARM, hover_z=0.4, num_x=10, num_y=10, num_n=6,
                        rect=((0.4,-0.4),(0.8,0.4)), table_z=-0.3):
    slat = SimpleLinearArmTrajectory()

    grasp_xy_list = []
    for x in np.linspace(rect[0][0], rect[1][0], num_x):
        for y in np.linspace(rect[0][1], rect[1][1], num_y):
            grasp_xy_list += [(x,y)]

    for xy in grasp_xy_list:
        # Move to setup position in the middle of the grasp space
        q = load_pickle(SETUP_POS_PICKLE)
        slat.set_joint_angles(arm, q, 3.)
        rospy.sleep(4.5)

        # Do grasping num_n times
        for i in range(num_n):

            # Move arm to current grasp position
            self.slat.move_arm(arm, [xy[0], xy[1], hover_z], 2.)
            self.slat.wait_for_arm_completion(arm)
            rospy.sleep(1.5)
        
            # TODO Begin monitoring sensors
            
            grasp_dist = hover_z - table_z
            # move arm down
            dur = self.slat.linear_move_arm(arm, grasp_dist, (0.,0.,-1.),  max_jerk=0.5)
            self.slat.wait_for_arm_completion(arm)
            rospy.sleep(1.5)

            # TODO End monitoring sensors

            # move arm up
            dur = self.slat.linear_move_arm(arm, grasp_dist, (0.,0.,1.),  max_jerk=0.5)
            self.slat.wait_for_arm_completion(arm)
            rospy.sleep(1.5)

        # TODO Find statistics for sensors

    # TODO save statistics for sensors
            

def main():
    grasp_learn()
    return 0

if __name__ == "__main__":
    sys.exit(main())
    
