#! /usr/bin/python

import numpy as np, math
import random

import roslib; roslib.load_manifest('pr2_grasp_behaviors')
import rospy
from grasp_manager import GraspBehavior

class SidewaysGrasp(GraspBehavior):
    def __init__(self, arm, use_coll_detection=False):
        super(OverheadGrasp, self).__init__(arm, use_coll_detection)
        self.SIDE_GRASP_DIST = 0.4
        self.TABLE_Z = -0.2
        self.JOINTS_BIAS = [0.0, 5.0, 0.0, -1.0, 4.0, -1.0, 0.0]
        if arm == 'l':
            for i in [0, 2, 4]:
                self.JOINTS_BIAS[i] *= -1
        self.BIAS_RADIUS = 0.012
        self.INIT_ANGS = [-0.05, -0.3, -3.1, -1.9, 3.1, -1.5, 0.0]
        self.GRASP_TIME = 2.0
        self.SETUP_VELOCITY = 0.8
        self.GRASP_VELOCITY = 0.4
        
    def setup_move(self, params):
        # object location (x, y), approach angle (r)
        self.xyr = params
        rospy.loginfo("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % self.xyr)
        grasp_pose = self.create_goal_pose(self.xyr[0], self.xyr[1], self.TABLE_Z, 
                                          quaternion_about_axis(self.xyr[2], (0, 0, 1)))
        return self.cm.move_arm_pose_biased(grasp_pose, self.JOINTS_BIAS, 
                                             self.SETUP_VELOCITY, blocking = True,
                                             init_angs=self.INIT_ANGS)

    def execute_approach(self, block):
        rospy.loginfo("Moving arm sideways")
        goal_pose = self.create_goal_pose(
                self.xyr[0] + self.SIDE_GRASP_DIST * np.cos(-self.xyr[2]), 
                self.xyr[1] - self.SIDE_GRASP_DIST * np.sin(-self.xyr[2]), 
                self.TABLE_Z,
                quaternion_about_axis(self.xyr[2], (0, 0, 1)))
        goal_pose.header.stamp = rospy.Time.now()
        return self.cm.move_cartesian_ik(goal_pose, collision_aware = False, 
                          blocking = block,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          settling_time = rospy.Duration(self.GRASP_TIME),
                          joints_bias = self.JOINTS_BIAS, bias_radius = self.BIAS_RADIUS,
                          vel = self.GRASP_VELOCITY)

    def execute_retreat(self):
        rospy.logerr("Need to implement this!")

    ##
    # Return random grasp configuration in entire space.
    def random_generator(self):
        x = random.uniform(0.45, 0.75)
        y = random.uniform(-0.55, 0.10)
        r = random.uniform(0., np.pi/2.)
        return x, y, r
