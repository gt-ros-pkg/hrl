
import sys
import numpy as np, math

import add_cylinder as ac

import roslib; roslib.load_manifest('arm_navigation_tutorials')
roslib.load_manifest('hrl_pr2_lib')
import rospy
import hrl_pr2_lib.pr2_arms as pa


if __name__ == '__main__':
    print 'Hello World'
    rospy.init_node('pr2_skin_simulate')

    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    raw_input('Touch the object and then hit ENTER.')

    ee_pos, ee_rot = pr2_arms.end_effector_pos(arm)
    print 'ee_pos:', ee_pos.flatten()
    print 'ee_pos.shape:', ee_pos.shape


    trans, quat = pr2_arms.tf_lstnr.lookupTransform('/base_link',
                             '/torso_lift_link', rospy.Time(0))
    print
    self.r_ee_pos = rot*tip + np.matrix(trans).T
    self.r_ee_rot = rot




    
    ac.add_cylinder()




