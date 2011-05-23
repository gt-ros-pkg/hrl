#! /usr/bin/python

import sys
import numpy as np, math
import random

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy
from tf.transformations import quaternion_about_axis, quaternion_multiply

from grasp_manager import GraspBehavior
from grasp_behavior_server import GraspBehaviorServer

class OverheadGrasp(GraspBehavior):
    def __init__(self, arm, use_coll_detection=False):
        super(OverheadGrasp, self).__init__(arm, use_coll_detection)

        self.HOVER_Z = -0.10
        self.GRASP_DIST = 0.30
        self.GRASP_VELOCITY = 0.3
        self.BIAS_RADIUS = 0.012
        self.GRASP_TIME = 2.0
        self.SETUP_VELOCITY = 0.5
        self.JOINTS_BIAS = [0.0, -0.25, -1.0, 0.0, 0.0, 0.5, 0.0]
        self.JIGGLE_RESOLUTION = 0.03 # 3cm resolution
        if arm == 'l':
            for i in [0, 2, 4]:
                self.JOINTS_BIAS[i] *= -1

    def grasp_preparation_move(self, blocking):
        joints = [-0.62734204881265387, -0.34601608409943324, -1.4620635485239604, -1.2729772622637399, -7.5123303230158518, -1.5570651396529178, -5.5929916630672727] 
        if self.arm == 'l':
            for i in [0, 2, 4]:
                joints[i] *= -1
        self.cm.command_joint_trajectory([joints], max_joint_vel=0.30, blocking=blocking)

    def grasp_setup_move(self, params):
        self.xyr = params
        rospy.loginfo("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % (self.xyr[0], self.xyr[1], self.xyr[2]))
        grasp_pose = self.create_goal_pose(self.xyr[0], self.xyr[1], self.HOVER_Z,
                                           self.overhead_gripper_pose(self.xyr[2]))
        return self.cm.move_arm_pose_biased(grasp_pose, self.JOINTS_BIAS, 
                                             self.SETUP_VELOCITY, blocking = True)

    def execute_approach(self, block):
        rospy.loginfo("Moving arm down")
        goal_pose = self.create_goal_pose(self.xyr[0], self.xyr[1], 
                                          self.HOVER_Z - self.GRASP_DIST, 
                                          self.overhead_gripper_pose(self.xyr[2]))
        goal_pose.header.stamp = rospy.Time.now()
        return self.cm.move_cartesian_ik(goal_pose, collision_aware = False, 
                          blocking = block,
                          step_size = .005, pos_thres = .005, rot_thres = .1,
                          settling_time = rospy.Duration(self.GRASP_TIME),
                          joints_bias = self.JOINTS_BIAS, bias_radius = self.BIAS_RADIUS,
                          vel = self.GRASP_VELOCITY)

    def execute_retreat(self):
        retreat_pose = self.create_goal_pose(self.xyr[0], self.xyr[1], 
                                             self.HOVER_Z, 
                                             self.overhead_gripper_pose(self.xyr[2]))
        self.cm.move_arm_pose_biased(retreat_pose, self.JOINTS_BIAS, 
                                     self.SETUP_VELOCITY, blocking = True)

    ##
    # Return random grasp configuration in entire space.
    def random_generator(self):
        x = random.uniform(0.40, 0.75)
        y = random.uniform(-0.35, 0.35)
        r = random.uniform(0., np.pi)
        return x, y, r

    ##
    # Attempt to slightly adjust grasp parameters to get a close configuration
    # which will hopefully find an IK solution.
    def jiggle_grasp_params(self, grasp_params):
        x, y, r = grasp_params
        dir_ang = random.uniform(0., 2. * np.pi)
        dx, dy = self.JIGGLE_RESOLUTION * np.cos(dir_ang), self.JIGGLE_RESOLUTION * np.sin(dir_ang)
        dr = random.uniform(-np.pi/12., np.pi/12.) # +/- 15 degrees
        x += dx
        y += dy
        r += dr
        r = self.normalize_rot(r)
        return (x, y, r)

    ##
    # Returns a quaternion for the gripper pose given a gripper rotation
    def overhead_gripper_pose(self, gripper_rot):
        gripper_rot = self.normalize_rot(gripper_rot)
        quat1 = quaternion_about_axis(np.pi/2., (0, 1, 0))
        quat2 = quaternion_about_axis(gripper_rot, (0, 0, 1))
        quat = quaternion_multiply(quat2, quat1)
        return quat

##
# Simple functionality and tests
def main():
    rospy.init_node("overhead_grasp")
    if sys.argv[1] not in ['r', 'l']:
        print "Must specify arm [r, l]"
        return
    if sys.argv[2] == "test":
        og = OverheadGrasp(sys.argv[1], use_coll_detection=True)
        while not rospy.is_shutdown():
            params = og.random_generator()
            print "Grasp Result:", og.perform_grasp(params, is_place=False, collide=True, 
                                                    behavior_name="overhead_grasp", sig_level=0.999)
            params = og.random_generator()
            print "Place Result:", og.perform_grasp(params, is_place=True, collide=True, 
                                                    behavior_name="overhead_grasp", sig_level=0.999)
        return

    if sys.argv[2] == "data":
        og = OverheadGrasp(sys.argv[1], use_coll_detection=True)
        num_times = 0
        while not rospy.is_shutdown():
            params = og.random_generator()
            og.perform_grasp(params, is_place=False, collide=True, data_collecting=True)
            rospy.loginfo("Number of grasps performed: %d", num_times)
            num_times += 1
        return

    if sys.argv[2] == "server":
        og = OverheadGrasp(sys.argv[1], use_coll_detection=True)
        gbs = GraspBehaviorServer(sys.argv[1], og)
        gbs.start_grasping_server("overhead_grasp", "overhead_grasp_setup")
        rospy.spin()
        return
        
    print "Usage: python overhead_grasp_behavior.py [r|l] [test|data|server]"
    print "args:", sys.argv
    return

if __name__ == "__main__":
    main()
