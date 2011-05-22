import numpy as np, math
import sys
import os
from threading import RLock
import threading
import multiprocessing as mp
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
sys.path.append("/usr/lib/python2.6/site-packages")
#import cv
#import arff

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

import actionlib
import tf

from std_msgs.msg import String, Bool
from geometry_msgs.msg import  Point, Pose, Quaternion, PoseStamped, PointStamped
import std_srvs.srv

import hrl_lib.util
from hrl_lib.rutils import GenericListener
import hrl_lib.viz as viz
from hrl_lib import tf_utils
from hrl_lib.keyboard_input import KeyboardInput
from hrl_lib.transforms import rotX, rotY, rotZ
from tf.transformations import *
from visualization_msgs.msg import MarkerArray, Marker
import dynamic_reconfigure.client
from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC, CURSOR_TOPIC, MOUSE_R_CLICK_TOPIC, MOUSE_R_DOUBLE_CLICK_TOPIC
import face_detector.msg
from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal, SingleJointPositionAction, SingleJointPositionGoal

import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult

from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC

from srv import CollisionDetectionStart

#from srv import StartFTDetect
#from msg import GraspStart

##
# Contains functionality for overhead grasping motions, training, and setup.
# The primary functionality is contained in collect_grasp_data, which runs
# empty grasps to train the grasp models, and in perform_grasp, which performs
# the grasping motion
class GraspBehavior(object):
    def __init__(self, arm, use_coll_detection=False):
        self.arm = arm
        self.use_coll_detection = use_coll_detection
        self.GRIPPER_POINT = np.array([0.23, 0.0, 0.0])

        self.cm = ControllerManager(self.arm)
        class CollisionState:
            def __init__(self):
                self.collided = False
            def callback(self, msg):
                self.collided = True
        self.coll_state = CollisionState()
        if self.use_coll_detection:
            rospy.loginfo("Waiting for %s_start_detection" % self.arm)
            rospy.wait_for_service(self.arm + '_collision_monitor/start_detection')
            self.start_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/start_detection', CollisionDetectionStart, persistent=True)
            rospy.loginfo("Waiting for %s_stop_detection" % self.arm)
            rospy.wait_for_service(self.arm + '_collision_monitor/stop_detection')
            self.stop_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/stop_detection', std_srvs.srv.Empty, persistent=True)
            rospy.Subscriber(self.arm + "_collision_monitor/arm_collision_detected", Bool, self.coll_state.callback)
        rospy.loginfo("[grasp_manager] GraspBehavior loaded.")

    ################################################################################
    # virtual functions to be implemented by specific grasp

    ##
    # Move arm to pre-grasp pose
    def setup_move(self, params):
        rospy.logerr("UNIMPLEMENTED!")
    ##
    # Execute the grasping arm motion
    def execute_approach(self, block):
        rospy.logerr("UNIMPLEMENTED!")
    ##
    # Generate a random behavior parameterization for sampling
    # the workspace of possible grasps
    def random_generator(self):
        rospy.logerr("UNIMPLEMENTED!")
    ##
    # Attempt to slightly adjust grasp parameters to get a close configuration
    # which will hopefully find an IK solution.
    def jiggle_grasp_params(self, grasp_params):
        return grasp_params
    ################################################################################

    ##
    # Transforms the given position by the offset position in the given quaternion
    # rotation frame
    def transform_in_frame(self, pos, quat, off_point):
        invquatmat = np.mat(quaternion_matrix(quat))
        invquatmat[0:3,3] = np.mat(pos).T
        trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
        transpos = invquatmat * trans
        return transpos.T.A[0,0:3]

    ##
    # Returns same gripper rotation normalized to [0, pi) range.
    def normalize_rot(self, gripper_rot):
        while gripper_rot >= np.pi:
            gripper_rot -= np.pi
        while gripper_rot < 0.:
            gripper_rot += np.pi
        return gripper_rot

    ##
    # Creates a PoseStamped in the torso_lift_link frame
    def create_goal_pose(self, x, y, z, gripper_pose):
        point = [x, y, z]
        point = self.transform_in_frame(point, gripper_pose, -self.GRIPPER_POINT).tolist()
        pose = point + gripper_pose.tolist()
        goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
        goal_pose.header.stamp = rospy.Time.now()
        return goal_pose

    ##
    # Returns the position of the end effector
    def get_end_effector_pos(self):
        wrist_pose = self.cm.get_current_wrist_pose_stamped("torso_lift_link")
        p = wrist_pose.pose.position
        o = wrist_pose.pose.orientation
        affector_pos = self.transform_in_frame([p.x, p.y, p.z],
                                          [o.x, o.y, o.z, o.w], self.GRIPPER_POINT)
        return affector_pos

    ##
    # Takes special care to kill the arm's movement by canceling
    # the action client and accounting for some edge cases.
    def kill_arm_movement(self):
        rospy.loginfo("Killing arm movement")
        self.cm.joint_action_client.cancel_all_goals()
        rospy.sleep(0.01) 
        # Needed for lag time in the case that the collision occurs
        # before the action is dispatched and due to the fact that
        # the arm doesn't necessarily respond the first time
        while not self.cm.check_joint_trajectory_done():
            self.cm.joint_action_client.cancel_all_goals()
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                self.cm.joint_action_client.cancel_all_goals()

    ##
    # Readys arm for grasp motion.
    def stage_grasp(self, grasp_params, open_gripper=True):
        rospy.loginfo("Staging grasp motion")
        self.setup_move(grasp_params)
        if open_gripper:
            # open gripper
            rospy.loginfo("Opening gripper")
            self.cm.command_gripper(1.00, -1.0, False)
            self.cm.gripper_action_client.wait_for_result(rospy.Duration(2.0))
        self.cm.wait_joint_trajectory_done()

    ##
    # Performs motion where arm approaches object
    def grasp_approach_motion(self, coll_detect=False, behavior_name="", sig_level=0.99):
        # move arm down
        rospy.loginfo("Moving arm down")
        startt = rospy.Time.now().to_sec()
        if coll_detect:
            self.start_detection(behavior_name, sig_level)
        result = self.execute_approach(False)

        if result == "no solution":
            # break without completing the grasp
            #self.apm.end_collision_detection()
            if coll_detect:
                self.stop_detection()
            return "no solution"

        # grasp motion should be successful and moving now

        # wait until the trajectory is complete.
        # this loop will not exit until the arm has stopped moving
        while not self.cm.check_joint_trajectory_done():
            if rospy.is_shutdown():
                self.kill_arm_movement()
                return "shutdown"
            if self.coll_state.collided:
                self.kill_arm_movement()
                break
            rospy.sleep(0.005)

        if coll_detect:
            self.stop_detection()
        if self.coll_state.collided and not coll_detect:
            grasp_result = "Collision"
        else:
            grasp_result = "No collision"
        self.coll_state.collided = False
        endt = rospy.Time.now().to_sec()
        rospy.loginfo("Grasp duration: %f", endt - startt)
        rospy.loginfo("Finished moving arm")
        return grasp_result

    ##
    # Gripper graping action
    def grasping_action(self):
        rospy.loginfo("Closing gripper")
        self.cm.command_gripper(0.0, 30.0, True)
        rospy.loginfo("Gripper closed")

    ##
    # Gripper placing action
    def placing_action(self):
        rospy.loginfo("Opening gripper")
        self.cm.command_gripper(1.00, -1.0, False)
        rospy.sleep(0.5)
        rospy.loginfo("Gripper opened")

    ##
    # Performs full grasping behavior.
    #
    # @param grasp_params A tuple containing the parameters for the behavior.
    # @param is_place If False, perform a grasp.  If True, perform a place.
    # @param collide Whether or not we should detect for collisions.
    # @param data_collecting If True, only perform the approach motion and return after that.
    # @param num_jiggle How many times the parameters should be jiggled before giving up
    # @return grasp_result Result of grasp.
    def perform_grasp(self, grasp_params, is_place=False, collide=True, 
                                          behavior_name="", sig_level=0.99, 
                                          data_collecting=False, num_jiggle=2):
        self.collide = collide

        rospy.loginfo("Performing grasp with parameters: " + str(grasp_params))
        iters = 0
        while not rospy.is_shutdown():
            # Move to grasp position
            self.stage_grasp(grasp_params, not is_place)
            rospy.sleep(0.1)
            approach_result = self.grasp_approach_motion(self.use_coll_detection and self.collide,
                                                         behavior_name=behavior_name, 
                                                         sig_level=sig_level)
            rospy.sleep(0.1)
            if approach_result != "no solution":
                break
            else:
                if num_jiggle == 0:
                    rospy.loginfo("Interpolated IK failed, cannot perform grasp for x = %1.2f, y = %1.2f, rot = %1.2f" % (x, y, gripper_rot))
                    return "IK failure"
                rospy.loginfo("Interpolated IK failed, jiggling configuration for nearby trajectory")
                # Jiggle it to see if it works
                grasp_params = self.jiggle_grasp_params(grasp_params)
                iters += 1
                # Tried too many times
                if iters >= num_jiggle:
                    rospy.loginfo("Cannot find IK solution")
                    return "IK failure"
                else:
                    continue

        if data_collecting:
            return "Grasp motion success"

        if approach_result == "Collision":
            if not is_place:
                self.grasping_action()
            else:
                self.placing_action()

        if not is_place:
            rospy.loginfo("Picking object up")
        else:
            rospy.loginfo("Pulling arm away")
        self.execute_retreat()
        rospy.sleep(0.5)
        
        ################################################################################
        # determine return result
        if approach_result == "Collision":
            if not is_place:
                if self.is_obj_in_gripper():
                    grasp_result = "Object grasped"
                else:
                    grasp_result = "Object missed"
            else:
                grasp_result = "Object placed"
        else:
            if not is_place: 
                grasp_result = "No collision for grasp"
            else:
                if self.is_obj_in_gripper():
                    grasp_result = "No collision for place"
                else:
                    grasp_result = "Object dropped before place"
        ################################################################################
        rospy.loginfo("Grasp result: %s" % grasp_result)
        rospy.loginfo("Grasp complete!")
        return grasp_result

    ##
    # Checks to see if anything is in the gripper currently (gripper is not fully closed).
    def is_obj_in_gripper(self):
        last_opening = self.cm.get_current_gripper_opening()
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            next_opening = self.cm.get_current_gripper_opening()
            print next_opening
            if abs(last_opening - next_opening) < 0.001:
                break
            last_opening = next_opening
        return self.cm.get_current_gripper_opening() > 0.01
    
