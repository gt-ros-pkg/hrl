#! /usr/bin/python

import numpy as np, math
import sys
import os
from threading import RLock
import threading
import multiprocessing as mp
import random
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

from pr2_overhead_grasping.srv import StartFTDetect
from pr2_overhead_grasping.msg import GraspStart
from perception_monitor import ArmPerceptionMonitor
from helpers import *

class GraspBehavior():
    def __init__(self, oger):
        self.oger = oger
    def setup_move(self, params):
        rospy.log_info("UNIMPLEMENTED!")
    def execute_grasp(self):
        rospy.log_info("UNIMPLEMENTED!")
    def random_generator(self):
        rospy.log_info("UNIMPLEMENTED!")

class OverheadGrasp(GraspBehavior):
    def setup_move(self, params):
        self.xyr = params
        log("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % (self.xyr[0], self.xyr[1], self.xyr[2]))
        grasp_pose = self.oger.create_goal_pose(self.xyr[0], self.xyr[1], self.oger.HOVER_Z,
                                           self.oger.overhead_gripper_pose(self.xyr[2]))
        return self.oger.cm.move_arm_pose_biased(grasp_pose, self.oger.JOINTS_BIAS, 
                                             self.oger.SETUP_VELOCITY, blocking = True)

    def execute_grasp(self, block):
        log("Moving arm down")
        goal_pose = self.oger.create_goal_pose(self.xyr[0], self.xyr[1], 
                                          self.oger.HOVER_Z - self.oger.GRASP_DIST, 
                                          self.oger.overhead_gripper_pose(self.xyr[2]))
        goal_pose.header.stamp = rospy.Time.now()
        return self.oger.cm.move_cartesian_ik(goal_pose, collision_aware = False, 
                          blocking = block,
                          step_size = .005, pos_thres = .005, rot_thres = .1,
                          settling_time = rospy.Duration(self.oger.GRASP_TIME),
                          joints_bias = self.oger.JOINTS_BIAS, bias_radius = self.oger.BIAS_RADIUS,
                          vel = self.oger.GRASP_VELOCITY)

    ##
    # Return random grasp configuration in entire space.
    def random_generator(self):
        x = random.uniform(0.40, 0.75)
        y = random.uniform(-0.35, 0.35)
        r = random.uniform(0., np.pi)
        return x, y, r

class SidewaysGrasp(GraspBehavior):
    def setup_move(self, params):
        # object location (x, y), approach angle (r)
        self.xyr = params
        log("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % self.xyr)
        self.SIDE_GRASP_DIST = 0.4
        self.TABLE_Z = -0.2
        self.JOINTS_BIAS = [0.0, 5.0, 0.0, -1.0, 4.0, -1.0, 0.0]
        self.BIAS_RADIUS = 0.012
        self.INIT_ANGS = [-0.05, -0.3, -3.1, -1.9, 3.1, -1.5, 0.0]
        grasp_pose = self.oger.create_goal_pose(self.xyr[0], self.xyr[1], self.TABLE_Z, 
                                          quaternion_about_axis(self.xyr[2], (0, 0, 1)))
        return self.oger.cm.move_arm_pose_biased(grasp_pose, self.JOINTS_BIAS, 
                                             self.oger.SETUP_VELOCITY, blocking = True,
                                             init_angs=self.INIT_ANGS)

    def execute_grasp(self, block):
        log("Moving arm sideways")
        goal_pose = self.oger.create_goal_pose(
                self.xyr[0] + self.SIDE_GRASP_DIST * np.cos(-self.xyr[2]), 
                self.xyr[1] - self.SIDE_GRASP_DIST * np.sin(-self.xyr[2]), 
                self.TABLE_Z,
                quaternion_about_axis(self.xyr[2], (0, 0, 1)))
        goal_pose.header.stamp = rospy.Time.now()
        return self.oger.cm.move_cartesian_ik(goal_pose, collision_aware = False, 
                          blocking = block,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          settling_time = rospy.Duration(self.oger.GRASP_TIME),
                          joints_bias = self.JOINTS_BIAS, bias_radius = self.BIAS_RADIUS,
                          vel = self.oger.GRASP_VELOCITY)

    ##
    # Return random grasp configuration in entire space.
    def random_generator(self):
        x = random.uniform(0.45, 0.75)
        y = random.uniform(-0.55, 0.10)
        r = random.uniform(0., np.pi/2.)
        return x, y, r


LABEL_STRS = ["normal", "external_collision", "table_collision"]
##
# Contains functionality for overhead grasping motions, training, and setup.
# The primary functionality is contained in collect_grasp_data, which runs
# empty grasps to train the grasp models, and in perform_grasp, which performs
# the grasping motion
class OverheadGrasper():
    def __init__(self, arm, active = False, ki = None, load_classifiers=True, 
                 use_classifiers=False):
        if arm == 0:
            self.armc = 'r'
        else:
            self.armc = 'l'
        self.arm = arm

        self.load_parameters()
        self.fos = FileOperations()
        #if use_classifiers:
        #    self.apm = ArmPerceptionMonitor(arm) #, load_classifiers=load_classifiers)

        if active:
            #if use_classifiers:
            #    self.apm.load_classifiers()
            self.spine = actionlib.SimpleActionClient(
                    'torso_controller/position_joint_action',
                    SingleJointPositionAction)
            self.spine.wait_for_server()
            self.cm = ControllerManager(self.armc)
            log("Waiting for %s_start_detection" % self.armc)
            rospy.wait_for_service(self.armc + '_start_detection')
            self.start_detection = rospy.ServiceProxy(self.armc + '_start_detection', std_srvs.srv.Empty, persistent=True)
            log("Waiting for %s_stop_detection" % self.armc)
            rospy.wait_for_service(self.armc + '_stop_detection')
            self.stop_detection = rospy.ServiceProxy(self.armc + '_stop_detection', std_srvs.srv.Empty, persistent=True)
            class CollisionFT:
                def __init__(self):
                    self.collided = False
                def callback(self, msg):
                    self.collided = True
            self.coll_class = CollisionFT()
            rospy.Subscriber(self.armc + "_arm_collision_detected", Bool, self.coll_class.callback)
            #if use_classifiers:
            #    self.apm.activate_sensing(self.cm.tf_listener)

        if ki is None:
            ki = KeyboardInput()
            
        self.ki = ki

    def load_parameters(self):
        self.HOVER_Z = rospy.get_param("/overhead_grasping/hover_z")
        self.GRASP_DIST = rospy.get_param("/overhead_grasping/grasp_dist")
        self.GRASP_VELOCITY = rospy.get_param("/overhead_grasping/grasp_velocity")
        self.GRIPPER_POINT = np.array(rospy.get_param("/overhead_grasping/gripper_point"))
        self.JOINTS_BIAS = np.array(rospy.get_param("/overhead_grasping/joints_bias"))
        self.BIAS_RADIUS = rospy.get_param("/overhead_grasping/bias_radius")
        self.GRASP_TIME = rospy.get_param("/overhead_grasping/grasp_time")
        self.SETUP_VELOCITY = rospy.get_param("/overhead_grasping/setup_velocity")
        self.GRASP_COLL_DATA_DIR = rospy.get_param("/overhead_grasping/grasp_coll_data_dir")

    ##
    # Transforms the given position by the offset position in the given quaternion
    # rotation frame
    #
    # @param pos the current positions
    # @param quat quaternion representing the rotation of the frame
    # @param off_point offset to move the position inside the quat's frame
    # @return the new position as a matrix column
    def transform_in_frame(self, pos, quat, off_point):
        invquatmat = np.mat(quaternion_matrix(quat))
        invquatmat[0:3,3] = np.mat(pos).T
        trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
        transpos = invquatmat * trans
        return transpos.T.A[0,0:3]

    ##
    # Creates a PoseStamped message using the given pose at time now.
    def create_gripper_pose(self, x, y, z, quat):
        point = [x, y, z]
        point = self.transform_in_frame(point, np.array(quat), -self.GRIPPER_POINT).tolist()
        point[0] -= 0.015 # TODO MYSTERY OFFSET!
        pose = point + quat
        goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
        goal_pose.header.stamp = rospy.Time.now()
        return goal_pose

    ##
    # TODO docs
    def overhead_gripper_pose(self, gripper_rot):
        gripper_rot = self.normalize_rot(gripper_rot)
        quat1 = quaternion_about_axis(np.pi/2., (0, 1, 0))
        quat2 = quaternion_about_axis(gripper_rot, (0, 0, 1))
        quat = quaternion_multiply(quat2, quat1)
        return quat

    ##
    # Returns same gripper rotation normalized to [0, pi) range.
    def normalize_rot(self, gripper_rot):
        while gripper_rot >= np.pi:
            gripper_rot -= np.pi
        while gripper_rot < 0.:
            gripper_rot += np.pi
        return gripper_rot

    ##
    # TODO docs
    def create_goal_pose(self, x, y, z, gripper_pose):
        point = [x, y, z]
        point = self.transform_in_frame(point, gripper_pose, -self.GRIPPER_POINT).tolist()
        point[0] -= 0.015 # TODO MYSTERY OFFSET!
        pose = point + gripper_pose.tolist()
        goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
        goal_pose.header.stamp = rospy.Time.now()
        return goal_pose

    def move_spine(self, pos = 0.2):
        self.spine.send_goal(SingleJointPositionGoal(position = pos))
        self.spine.wait_for_result()

    ##
    # Run grasp collision data collection.  
    # Goes to each grasp configuration, performs grasp motion
    # at each configuration a number of times, and stores perception data over each trial.
    # Arms should be unobstructed so that a clean grasp motion is obtained.
    # TODO DOCS
    def collect_collision_data(self, num_grasps, directory):

        grasp_data = []
        grasp_index = {}
        data_ind = 1
        num_collected = 0

        log("Opening gripper")
        self.cm.command_gripper(1.00, -1.0, False)
        log("Moving spine")
        self.move_spine()
        self.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

        while num_collected < num_grasps:
            xyr = self.random_grasp_total()
            if rospy.is_shutdown():
                return
            log("---------------------------------------------------")
            log("%1.2f completion" % (float(num_collected) / num_grasps))
            # Do grasping num_n times
            for i in range(1):

                # Move to grasp position
                log("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % xyr)
                grasp_pose = self.create_goal_pose(xyr[0], xyr[1], self.HOVER_Z,
                                                   self.overhead_gripper_pose(xyr[2]))
                setup_result = self.cm.move_arm_pose_biased(grasp_pose, self.JOINTS_BIAS, 
                                                     self.SETUP_VELOCITY, blocking = True)
                rospy.sleep(0.5)
                if setup_result is not None:

                    goal_pose = self.create_goal_pose(xyr[0], xyr[1], 
                                                      self.HOVER_Z - self.GRASP_DIST, 
                                                      self.overhead_gripper_pose(xyr[2]))
                    if directory != "empty_grasp":
                        wait_for_key(self.ki, ' ')
                        while self.ki.kbhit():
                            pass
                    # start gathering data
                    self.apm.start_data_capture()
                    # move arm down
                    log("Moving arm down")
                    sttime = rospy.Time.now().to_sec()
                    result = self.downward_grasp(goal_pose, block=False)
                    # break if a key is hit or the trajectory is complete
                    cancel, cancelled = False, False
                    while not self.cm.check_joint_trajectory_done():
                        if directory != "empty_grasp":
                            if self.ki.kbhit():
                                length = self.apm.stop_data_capture()
                                cancel = True
                                cancelled = True
                        if cancel:
                            self.cm.joint_action_client.cancel_all_goals()
                        rospy.sleep(0.01)
                        if rospy.is_shutdown():
                            return None
                    if not cancelled:
                        length = self.apm.stop_data_capture()
                    endtime = rospy.Time.now().to_sec()
                    log("dur:", endtime - sttime)
                    log(result)
                    log("Finished moving arm")
                    log("length:", length)
                    log("len/dur", length / (endtime - sttime))

                    rospy.sleep(0.5)
                else:
                    break

            # can't move to initial position
            if setup_result is None:
                continue

            if result != "no solution":
                while True:
                    if directory != "empty_grasp":
                        char_res = self.ki.getch()
                    else:
                        char_res = 'c'
                    if char_res == 'c':
                        gfn = (self.GRASP_COLL_DATA_DIR + "//" + directory
                               + "//data%2d.pickle" % (data_ind))
                        self.fos.save_pickle({"grasp_loc" : xyr, 
                                              "signals" : self.apm.datasets}, gfn)
                                               
                        grasp_index[gfn] = -1.0
                        data_ind += 1
                        num_collected += 1
                        break
                    elif char_res == 'd':
                        break

            else:
                err("Grasp Failed, not adding data")
            self.apm.clear_vars()

        coll_times = grasp_index
        self.fos.save_coll_times(coll_times, self.GRASP_COLL_DATA_DIR + "//" + directory)
        log("Data save complete")

    ##
    # Run grasp collision data collection.  
    # Goes to each grasp configuration, performs grasp motion
    # at each configuration a number of times, and stores perception data over each trial.
    # Arms should be unobstructed so that a clean grasp motion is obtained.
    # TODO DOCS
    def new_collect_collision_data(self, behavior, num_grasps, label, not_collision, ft, coll_detect=False):

        if ft:
            log("Waiting for ft_start_detection")
            rospy.wait_for_service('ft_start_detection')
            ft_start_detection = rospy.ServiceProxy('ft_start_detection', StartFTDetect, persistent=True)
            log("Waiting for ft_stop_detection")
            rospy.wait_for_service('ft_stop_detection')
            ft_stop_detection = rospy.ServiceProxy('ft_stop_detection', std_srvs.srv.Empty, persistent=True)
        elif coll_detect:
            log("Waiting for %s_start_detection" % self.armc)
            rospy.wait_for_service(self.armc + '_start_detection')
            rf_start_detection = rospy.ServiceProxy(self.armc + '_start_detection', std_srvs.srv.Empty, persistent=True)
            log("Waiting for %s_stop_detection" % self.armc)
            rospy.wait_for_service(self.armc + '_stop_detection')
            rf_stop_detection = rospy.ServiceProxy(self.armc + '_stop_detection', std_srvs.srv.Empty, persistent=True)
        class CollisionFT:
            def __init__(self):
                self.collided = False
            def callback(self, msg):
                self.collided = True

        log("Opening gripper")
        self.cm.command_gripper(1.00, -1.0, False)
        log("Moving spine")
        self.move_spine()
        self.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

        pub_start = rospy.Publisher('/grasper/grasp_executing', GraspStart)

        num_collected = 0
        while num_collected < num_grasps:
            params = behavior.random_generator()
            if rospy.is_shutdown():
                return
            log("---------------------------------------------------")
            log("%1.2f completion" % (float(num_collected) / num_grasps))

            # Move to grasp position
            setup_result = behavior.setup_move(params)
            rospy.sleep(0.5)
            if setup_result is not None:

                # if directory != "empty_grasp":
                #     wait_for_key(self.ki, ' ')
                #     while self.ki.kbhit():
                #         pass
                coll_class = CollisionFT()
                if ft:
                    rospy.Subscriber("force_torque_collision", Bool, coll_class.callback)
                else:
                    rospy.Subscriber(self.armc + "_arm_collision_detected", Bool, coll_class.callback)

                # start gathering data
                if ft:
                    ft_start_detection(label, not not_collision)
                elif coll_detect:
                    rf_start_detection()

                # move arm down
                sttime = rospy.Time.now().to_sec()
                gs = GraspStart(x=params[0],y=params[1],r=params[2])
                pub_start.publish(gs)
                result = behavior.execute_grasp(False)

                # break if a key is hit or the trajectory is complete
                if not not_collision:
                    while (not self.cm.check_joint_trajectory_done() and 
                            not coll_class.collided):
                        rospy.sleep(0.01)
                        if rospy.is_shutdown():
                            return None
                else:
                    while (not self.cm.check_joint_trajectory_done()):
                        rospy.sleep(0.01)
                        if rospy.is_shutdown():
                            return None
                self.cm.joint_action_client.cancel_all_goals()

                # end detection
                pub_start.publish(gs)
                if ft:
                    ft_stop_detection()
                elif coll_detect:
                    rf_stop_detection()
                endtime = rospy.Time.now().to_sec()
                log("dur:", endtime - sttime)

                rospy.sleep(0.5)
                num_collected += 1

        log("Data save complete")

    ##
    # Loads grasp with given file name.  Loading is done in separate thread to allow
    # robot to setup while file is still being read into memory.
    # def dynamic_pickle_loader(self, filename):
    #     class GLoader(threading.Thread):
    #         def __init__(self, oger):
    #             threading.Thread.__init__(self)
    #             self.pickle = None
    #             self.oger = oger

    #         def run(self):
    #             self.pickle = self.oger.fos.load_pickle(filename)

    #     gl = GLoader(self)
    #     gl.start()
    #     return gl.pickle

    def get_end_affector_pos(self):
        wrist_pose = self.cm.get_current_wrist_pose_stamped("torso_lift_link")
        p = wrist_pose.pose.position
        o = wrist_pose.pose.orientation
        affector_pos = self.transform_in_frame([p.x, p.y, p.z],
                                          [o.x, o.y, o.z, o.w], self.GRIPPER_POINT)
        point[0] -= 0.015 # TODO MYSTERY OFFSET!
        return affector_pos

    def kill_arm_movement(self):
        log("Killing arm movement")
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

    def stage_grasp(self, x, y, r, open_gripper=True):
        log("Staging grasp motion")
        self.behavior = OverheadGrasp(self)
        self.behavior.setup_move([x, y, r])
        #stage_pose = self.create_goal_pose(x, y, self.HOVER_Z, self.overhead_gripper_pose(r))
        #self.cm.move_arm_pose_biased(stage_pose, self.JOINTS_BIAS, 
        #                             self.SETUP_VELOCITY, blocking = False)
        if open_gripper:
            # open gripper
            log("Opening gripper")
            self.cm.command_gripper(1.00, -1.0, False)
            self.cm.gripper_action_client.wait_for_result(rospy.Duration(2.0))
        self.cm.wait_joint_trajectory_done()

    def grasp_descent_motion(self, x, y, r, z=None):
        goal_pose = self.create_goal_pose(x, y, self.HOVER_Z - self.GRASP_DIST, self.overhead_gripper_pose(r))

        # move arm down
        log("Moving arm down")
        startt = rospy.Time.now().to_sec()
        #if not self.apm.begin_collision_detection(dynamic_detection=True, 
        #                                          callback=self.kill_arm_movement):
        #    return "ERROR"
        #result = self.downward_grasp(goal_pose, block = False)
        self.start_detection()
        result = self.behavior.execute_grasp(False)

        if result == "no solution":
            # break without completing the grasp
            #self.apm.end_collision_detection()
            self.stop_detection()
            return "no solution"

        # grasp should be successful and be moving now

        grasp_result = "collision"
        if z is None:
            # wait until the trajectory is complete.
            # this loop will not exit until the arm has stopped moving
            while not self.cm.check_joint_trajectory_done():
                if rospy.is_shutdown():
                    self.kill_arm_movement()
                    return "shutdown"
                if self.coll_class.collided:
                    self.kill_arm_movement()
                    break
                rospy.sleep(0.005)
        else:
            # wait until we have either collided or have reached the desired height
            while not self.cm.check_joint_trajectory_done():
                affector_pos = self.get_end_affector_pos()
                if affector_pos[2] <= z:
                    log("Reached z position")
                    log("Affector position:", affector_pos)
                    self.kill_arm_movement()
                    grasp_result = "Reached z"
                    break
                if rospy.is_shutdown():
                    self.kill_arm_movement()
                    return "shutdown"
                if self.coll_class.collided:
                    self.kill_arm_movement()
                    break
                rospy.sleep(0.005)
            #if self.apm.collision_detected:
            #    log("Collided before reaching z position")

        #self.apm.end_collision_detection()
        self.stop_detection()
        if self.coll_class.collided:
            grasp_result = "Collision"
        else:
            grasp_result = "No collision"
        self.coll_class.collided = False
        endt = rospy.Time.now().to_sec()
        log("Grasp duration:", endt - startt)
        log("Finished moving arm")
        return grasp_result

    def jiggle_grasp_loc(self, x, y, r):
        RESOLUTION = 0.03 # 3cm resolution
        dir_ang = random.uniform(0., 2. * np.pi)
        dx, dy = RESOLUTION * np.cos(dir_ang), RESOLUTION * np.sin(dir_ang)
        dr = random.uniform(-np.pi/12., np.pi/12.) # +/- 15 degrees
        x += dx
        y += dy
        r += dr
        r = self.normalize_rot(r)
        return x, y, r

    def grasping_action(self):
        log("Closing gripper")
        self.cm.command_gripper(0.0, 30.0, True)
        log("Gripper closed")

    def placing_action(self):
        log("Opening gripper")
        self.cm.command_gripper(1.00, -1.0, False)
        rospy.sleep(0.5)
        log("Gripper opened")

    def grasp_ascent_action(self, x, y, r):
        stage_pose = self.create_goal_pose(x, y, self.HOVER_Z, self.overhead_gripper_pose(r))
        self.cm.move_arm_pose_biased(stage_pose, self.JOINTS_BIAS, self.SETUP_VELOCITY, blocking = True)

    ##
    # Perform collision sensing grasping motion. Arm will move to (x, y, gripper_rot)
    # location at the predefined height and open the gripper.  The grasping motion 
    # will move directly down until a collision is perceived. The gripper will then
    # close and wait until the gripper has stopped moving.  The arm then raises back
    # to its initial grasping pose. There is varying funcitonality based on parameters
    # provided.  If z is provided, grasp motion will end when z coordinate is reached.
    # If is_grasp is False, the gripper actions will not be performed. If is_place is
    # True, gripper will not open before the grasp, and will open instead of close upon
    # collision detection.
    #
    # Grasp results:
    # No collision : The grasp ended without colliding with anything and did not
    #                stop at the z value.  
    # Environmental collision : A collision such as a bump is detected that does not 
    #                           look like a collision with a table object. If this 
    #                           type of collision is detected, the arm will freeze 
    #                           where the collision was detected.
    # Table collision : A collision was detected with what seemed to be an object 
    #                   on the table. This is a successful grasp result.
    # Reached z : No collision was detected but the gripper has reached the z position
    #             requested in the parameters.
    # IK failure : No IK trajectories were found for these parameters.  The arm does
    #              not move in this case
    def perform_grasp(self, x, y, z=None, gripper_rot = np.pi / 2., is_place=False,
                      collide=True, num_jiggle=2, disable_env=True):

        gripper_rot = self.normalize_rot(gripper_rot)
        log("Performing grasp (%1.2f, %1.2f), rotation: %1.2f" % (x, y, gripper_rot))
        log("Beginning grasp")
        iters = 0
        while not rospy.is_shutdown():
            # Move to grasp position
            self.stage_grasp(x, y, gripper_rot, not is_place)
            rospy.sleep(0.1)
            grasp_result = self.grasp_descent_motion(x, y, gripper_rot, z)
            rospy.sleep(0.1)
            if grasp_result != "no solution":
                break
            else:
                if num_jiggle == 0:
                    log("Interpolated IK failed, cannot perform grasp for x = %1.2f, y = %1.2f, rot = %1.2f" % (x, y, gripper_rot))
                    return "IK failure"
                log("Interpolated IK failed, jiggling configuration for nearby trajectory")
                # Jiggle it to see if it works
                x, y, gripper_rot = self.jiggle_grasp_loc(x, y, gripper_rot)
                iters += 1
                # Tried too many times
                if iters >= num_jiggle:
                    log("Cannot find IK solution")
                    return "IK failure"
                else:
                    continue

        #affector_pos = self.get_end_affector_pos()
        #if not disable_env:
        #    if (grasp_result == "Environmental collision" or
        #        grasp_result == "No collision" and 
        #        (z is None or np.fabs(z - affector_pos[2]) < 0.03)):
        #        # Stop here because there is not a successful grasp configuration
        #        return grasp_result

        if grasp_result == "Collision":
            if not is_place:
                self.grasping_action()
            else:
                self.placing_action()

        log("Moving back to grasp position")
        self.grasp_ascent_action(x, y, gripper_rot)
        
        if grasp_result == "Collision":
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
        log("Grasp result: %s" % grasp_result)
        log("Grasp complete!")
        return grasp_result

    ##
    # Checks to see if anything is in the gripper currently (gripper is not fully closed).
    def is_obj_in_gripper(self):
        last_opening = self.cm.get_current_gripper_opening()
        print "is_obj_in_gripper"
        print last_opening
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            next_opening = self.cm.get_current_gripper_opening()
            print next_opening
            if abs(last_opening - next_opening) < 0.001:
                break
            last_opening = next_opening
        return self.cm.get_current_gripper_opening() > 0.01

    ##
    # Uses matplotlib to display grasp data, models, tolerance ranges, and online data.
    # TODO Better docs
    def display_model_coodinate(self, models, percept="gripper_pose", index=2,
                            fig=1, show=True, coll_pt=None, is_impact=True):

        if is_impact:
            features = models[percept]["impact_features"]
            sigmas = models[percept]["i_sigma_list"]
        else:
            features = models[percept]["type_features"]
            sigmas = models[percept]["t_sigma_list"]

        times = models[percept]["time"]
        plt.figure(fig)
        plt.title("Perception: %s, Index: %d" % (percept, index))
        
        for deg in range(3):
            for si, sigma in enumerate(sigmas):
                zip_feats = np.array(zip(*features[deg][si]))
                # zip_vars = np.array(zip(*vars[deg][si]))
                plt.subplot(3, len(sigmas), deg*len(sigmas) + 1 + si)
                plt.plot(times, zip_feats[index], color = 'r')
                if coll_pt is not None:
                    plt.axvline(coll_pt, color='b', linestyle = '--')
                plt.title(percept)
                # plt.title("Degree %d, Sigma %1.2f" % (deg, sigma))
        if show:
            plt.show()

    ##
    # Uses matplotlib to display grasp data, models, tolerance ranges, and online data.
    # TODO Better docs
    def display_models_all(self, models, deg = 0, sigma_ind = 0,
                            fig = 1, show=True, coll_pt = None):
        if is_impact:
            features = models[k]["impact_features"][deg][sigma_ind]
        else:
            features = models[k]["type_features"][deg][sigma_ind]

        plt.figure(fig)
        
        num_plot = 0
        for k in models:
            coords = zip(*features)
            times = models[k]["time"]
            for c_ind, c_data in enumerate(coords):
                plt.subplot(7, 5, num_plot)
                num_plot += 1
                plt.plot(times, c_data, color = 'r')
                if coll_pt is not None:
                    plt.axvline(coll_pt, color='b', linestyle = '--')
                plt.title(k + " %d" % c_ind)
        if show:
            plt.show()

    ##
    # Uses matplotlib to display grasp data, models, tolerance ranges, and online data.
    # TODO Better docs
    def display_datasets_all(self, datasets, fig = 1, show=True, coll_pt = None, sigma= 0.):

        data = datasets

        plt.figure(fig)
        num_plot = 0
        for k in data:
            times = zip(*data[k][0])[0]
            coords = zip(*zip(*data[k][0])[1])
            for c_ind, c_data in enumerate(coords):
                plt.subplot(7, 5, num_plot)
                num_plot += 1
                if sigma != 0.:
                    c_data = self.apm.impact_fgc.convolve_signal(c_data, times, 0, sigma)
                # temp = [0. for i in range(len(times))]
                # temp[len(times)/2] = 1.
                # convolved_data = self.apm.impact_fgc.convolve_signal(temp, times, 0, 0.15)
                plt.plot(times, c_data, color = 'r')
                if coll_pt is not None:
                    plt.axvline(coll_pt, color='b', linestyle = '--')
                plt.title(k + " %d" % c_ind)

        if show:
            plt.show()

    def save_grasp_model_plots(self, models, filename, coll_pt, fig = 1):
        pp = PdfPages(filename)
        for p in self.apm.perception_names:
            for coord in range(self.apm.perception_lengths[p]):
                self.display_model_coodinate(models, p, coord, fig, 
                                         show = False, coll_pt=coll_pt)
                pp.savefig()
                fig += 1
        pp.close()
        return fig

    def process_all_signals(self, directory):
        coll_times = self.fos.load_coll_times(self.GRASP_COLL_DATA_DIR +
                                          "//" + directory)
        for gfn in coll_times:
            log("processing", gfn)
            grasp = self.fos.load_pickle(gfn)
            # TODO TODO TODO TODO TODO TODO 
            models = self.apm.process_signals(grasp[2])
            log("Saving grasp")
            processed_grasp = {"grasp_loc" : grasp[0],
                               "signals" : grasp[2],
                               "processed_signals" : models}
            self.fos.save_pickle(processed_grasp, gfn)
            # TODO TODO TODO TODO TODO TODO 

    def annotate_display(self, datasets, fig = 1):
        data = datasets
        plt.figure(fig)
        times = np.array(zip(*data["accelerometer"][0])[0])
        times -= times[0]
        acc_x = np.array(zip(*zip(*data["accelerometer"][0])[1])[0])
        acc_y = np.array(zip(*zip(*data["accelerometer"][0])[1])[1])
        acc_z = np.array(zip(*zip(*data["accelerometer"][0])[1])[2])
        grip_z  = np.array(zip(*zip(*data["gripper_pose"][0])[1])[2])
        grip_x  = np.array(zip(*zip(*data["gripper_pose"][0])[1])[0])
        grip_y  = np.array(zip(*zip(*data["gripper_pose"][0])[1])[1])
        vals = [acc_x, acc_y, acc_z, grip_z, grip_x, grip_y]
        colors = ['r', 'b', 'g', 'black', 'purple', 'yellow']
        ax = plt.subplot(111)
        for i, temp in enumerate(vals):
            vals[i] -= np.min(vals[i])
            vals[i] = vals[i] / (np.max(vals[i]) - np.min(vals[i]))
            plt.plot(times, vals[i], color = colors[i])
        # plt.xticks(np.linspace(2.0, 4.0, 21))
        
        from matplotlib.ticker import MultipleLocator, FormatStrFormatter
        majorLocator   = MultipleLocator(0.01)
        majorFormatter = FormatStrFormatter('%1.2f')
        minorFormatter = FormatStrFormatter('%1.2f')
        minorLocator   = MultipleLocator(0.01)
        ax.xaxis.set_major_locator(minorLocator)
        ax.xaxis.set_major_formatter(majorFormatter)
        ax.xaxis.set_minor_locator(minorLocator)
        ax.xaxis.set_minor_formatter(minorFormatter)
        ax.set_xlim((0.0, times[-1]))
        ax.xaxis.grid(b=True, color = 'purple', linewidth = 1, which='minor')
        plt.show()

    def annotate_collision_times(self, directory):
        coll_times = self.fos.load_coll_times(self.GRASP_COLL_DATA_DIR +
                                          "//" + directory)
        for gfn_i, gfn in enumerate(coll_times):
            if coll_times[gfn] == -1.0:
                grasp = self.fos.load_pickle(gfn)
                self.annotate_display(grasp["signals"], fig = gfn_i + 1)
                time = input("Collision time: ")
                coll_times[gfn] = time
                print time

                if rospy.is_shutdown():
                    sys.exit()
                
        log(coll_times)
        self.fos.save_coll_times(coll_times, self.GRASP_COLL_DATA_DIR +
                       "//" + directory)

    def save_annotated_plots(self, directory):
        coll_times = self.fos.load_coll_times(self.GRASP_COLL_DATA_DIR +
                                          "//" + directory)
        fig = 1
        for gfn in coll_times:
            grasp = self.fos.load_pickle(gfn)
            models = grasp["processed_signals"]
            times = np.array([])
            # for i in range(200):
            #     sum, num = 0., 0.
            #     for k in models:
            #         sum += models[k]["time"][i]
            #         num += 1.
            #     for k in models:
            #         models[k]["time"][i] = sum / num
            filename = self.fos.get_plot_name(directory, gfn)
            coll_pt = coll_times[gfn]
            if coll_pt == -1.:
                coll_pt = None
            fig = self.save_grasp_model_plots(models, filename, coll_pt, fig)
            log("saved", filename)

    def compile_classification_data(self, directory, percepts=None, coll_num=1.,
                                    percent_split=0.1, seed=1, is_impact=True):
        # COLLISION_TIME_OFFSET = -0.03
        if is_impact:
            degree_dict = self.apm.I_DEGREE_DICT
            feature_index = "impact_features"
            sigma_list = self.apm.i_sigma_list
        else:
            degree_dict = self.apm.T_DEGREE_DICT
            feature_index = "type_features"
            sigma_list = self.apm.t_sigma_list

        coll_times_dict = self.fos.load_coll_times(self.GRASP_COLL_DATA_DIR +
                                          "//" + directory)

        # mark all of the testing datasets using a parallel array which
        # marks those corresponding trajectories with True
        # this code builds that array
        random.seed(seed)
        testing_trajs = [False] * len(coll_times_dict)
        c = 0
        while c < int(percent_split * len(coll_times_dict)):
            place = random.randint(0, len(coll_times_dict) - 1)
            if not testing_trajs[place]:
                testing_trajs[place] = True
                c += 1

        training_data, training_labels = [], []
        testing_data, testing_labels = [], []
        coord_names = None
        training_coll_times, training_times = [], []
        testing_coll_times, testing_times = [], []
        traj_labels = []
        for gfn_i, gfn in enumerate(coll_times_dict):
            log("Processing", gfn)
            grasp = self.fos.load_pickle(gfn)
            coll_time = coll_times_dict[gfn] # + COLLISION_TIME_OFFSET
            grasp_loc, datasets, models = grasp["grasp_loc"], grasp["signals"], grasp["processed_signals"]
            labels = None
            times = None
            data_lists = []
            coord_names = []

            ##
            # This code will lazily average all of the time values in order.
            # It should help reduce headaches in the future.
            i = 0
            while not rospy.is_shutdown():
                sum, num = 0., 0.
                exit = False
                for k in models:
                    if i >= len(models[k]["time"]):
                        exit = True
                        break
                    sum += models[k]["time"][i]
                    num += 1.
                if exit:
                    break
                for k in models:
                    models[k]["time"][i] = sum / num
                i += 1

            ##
            # Compiles all of the models data into feature vectors which can be
            # easily passed into opencv machine learning stuff
            for percept in self.apm.PERCEPTION_ORDER:
                if percepts is not None:
                    if percept not in percepts:
                        continue
                features = models[percept][feature_index]
                times = models[percept]["time"]
                for coord in range(self.apm.perception_lengths[percept]):
                    # ANONYMIZING
                    # data_coord = zip(*zip(*datasets[percept][0])[1])[coord]
                    # data_lists.append(data_coord)
                    # coord_names.append("%s_%d_raw" % (percept, coord))
                    for deg in range(3):
                        if deg not in degree_dict[percept][coord]:
                            continue
                        for si in range(len(sigma_list)):
                            feature_list = zip(*features[deg][si])[coord]
                            cur_times = times

                            ########### PROCESSING ###################
                            
                            # if deg > 0:
                            #     feature_list = np.fabs(feature_list)

                            # begining_time = 1.0
                            # beg_ind = 0
                            # while cur_times[beg_ind] < begining_time:
                            #     beg_ind += 1

                            # feature_list = feature_list[beg_ind:]
                            # cur_times = cur_times[beg_ind:]

                            # ANONYMIZING
                            #if percept == "joint_angles" and deg == 0:
                            #    continue


                            ############################################
                            
                            if coll_num != 0.:
                                div_ind = 0
                                while cur_times[div_ind] < coll_time:
                                    div_ind += 1
                            else:
                                div_ind = len(feature_list) + 1

                            if is_impact:
                                data_lists.append(feature_list)
                            else:
                                # only use collision data
                                data_lists.append(feature_list[div_ind:])
                                cur_times = cur_times[div_ind:]

                            if labels is None:
                                # this section of code only needs to run once
                                # per file so it will run on the first coordinate
                                # processed
                                if is_impact:
                                    labels = ([0.] * (div_ind - 1) + 
                                              [coll_num] * (len(feature_list) + 1
                                                            - div_ind))
                                else:
                                    labels = ([coll_num] * (len(feature_list) + 1
                                                            - div_ind))
                                if not testing_trajs[gfn_i]:
                                    training_labels.extend(labels)
                                    training_coll_times.extend([coll_time]*len(labels))
                                    training_times.extend(cur_times)
                                else:
                                    testing_labels.extend(labels)
                                    if gfn_i >= len(testing_times):
                                        testing_coll_times.append(coll_time)
                                        testing_times.append(cur_times)
                            coord_names.append("%s_%d_%d_%d" % (percept, coord, deg, si))

            # add other features

            # MADE ANONYMOUS
            # data_lists.append(times)
            # coord_names.append("time")
            # data_lists.append([grasp_loc[0] for i in times])
            # coord_names.append("x_location")
            # data_lists.append([grasp_loc[1] for i in times])
            # coord_names.append("y_location")
            # data_lists.append([grasp_loc[2] for i in times])
            # coord_names.append("gripper_rot")

            if not testing_trajs[gfn_i]:
                training_data.extend(zip(*data_lists))
            else:
                testing_data.extend(zip(*data_lists))

            traj_labels.extend([(directory, gfn)]*len(data_lists[0]))

        return (training_data, training_labels, testing_data, testing_labels, 
                coord_names, training_times, training_coll_times, testing_times, testing_coll_times, traj_labels)

    def multiple_dataset_compilation(self, directory_dict, fn = "data", 
                                     percent_split = 0.1, seed = 1, 
                                     save_arff = False, is_impact=True):
        training_data_full, training_labels_full = [], []
        testing_data_full, testing_labels_full = [], []
        coord_names = None
        training_times_full, training_coll_times_full = [], []
        testing_times_full, testing_coll_times_full = [], []
        traj_labels_full = []
        for d_i, directory in enumerate(directory_dict):
            log("Compiling data for directory %s" % directory)
            (training_data, training_labels, 
             testing_data, testing_labels,
             coord_names, training_times, training_coll_times, testing_times,
             testing_coll_times, traj_labels) = self.compile_classification_data(directory, 
                                                  percent_split = percent_split,
                                                  coll_num = directory_dict[directory],
                                                  seed = seed, is_impact=is_impact)
            training_data_full.extend(training_data)
            training_labels_full.extend(training_labels)
            testing_data_full.extend(testing_data)
            testing_labels_full.extend(testing_labels)
            training_times_full.extend(training_times)
            training_coll_times_full.extend(training_coll_times)
            testing_times_full.extend(testing_times)
            testing_coll_times_full.extend(testing_coll_times)
            traj_labels_full.extend(traj_labels)

        log("Total number of instances - training: %d | testing: %d" % 
                            (len(training_labels_full), len(testing_labels_full)))
        compiled_dataset = {"training_data" : np.matrix(training_data_full, dtype='float32').T, 
                            "training_labels" : np.matrix(training_labels_full, dtype='float32'),
                            "training_times_list" : np.array(training_times_full), 
                            "training_colls_list" : np.array(training_coll_times_full), 
                            "testing_data" : np.matrix(testing_data_full, dtype='float32').T,
                            "testing_labels" : np.matrix(testing_labels_full, dtype='float32'),
                            "testing_times_list" : np.array(testing_times_full), 
                            "testing_colls_list" : np.array(testing_coll_times_full),
                            "coordinate_names" : coord_names,
                            "trajectory_labels" : traj_labels_full}

        # if save_arff:
        #     self.arff_data_writer(training_data_full, training_labels_full, coord_names, 
        #                                 fn = fn + "_training.arff")
        #     self.arff_data_writer(testing_data_full, testing_labels_full, coord_names, 
        #                                 fn = fn + "_testing.arff")
        #     self.fos.save_pickle(data_params, fn + "_arff_recreation.pickle")

        return compiled_dataset

    ##
    # Parse eval_fn file which contains copied results from the weka
    # classifications.  Also loads derivative data from the arff creations
    # returns a list of testing trajectories consisting of 4-tuples
    # (time, actual label, predicted label, trajectory collision time)
    def eval_predicts(self, eval_fn, recreate_fn):
        pickled = self.fos.load_pickle(recreate_fn + "_arff_recreation.pickle")
        labels = pickled["testing_labels"] 
        times_list = pickled["testing_times_list"]
        colls_list = pickled["testing_colls_list"]
        eval_file = open(eval_fn, 'r')
        traj_list, traj = [], []
        l_ind, t_ind, ct_ind = 0, 0, 0
        while True:
            line = eval_file.readline()
            pred_num = int(line[line.rfind(":") - 1]) - 1
            # log(len(times_list[t_ind]), len(traj), l_ind, ct_ind, len(labels), len(times_list), t_ind)
            traj.append([times_list[t_ind][ct_ind], labels[l_ind], 
                         pred_num, colls_list[t_ind]])
            # log(line)
            l_ind += 1
            ct_ind += 1
            if ct_ind >= len(times_list[t_ind]):
                traj_list.append(traj)
                traj = []
                t_ind += 1
                ct_ind = 0
                if t_ind >= len(times_list):
                    break
        return traj_list

    def svm_cross_valid(self, train, responses, num_folds = 10):
        tp, fp, tn, fn, total = 0., 0., 0., 0., 0.
        for fold_i in range(num_folds):
            log("Fold:", fold_i + 1)
            rinds = [random.randint(0,len(train)-1) for rind in range(len(train)/num_folds)]
            rninds = []
            for i in range(len(train)):
                if not i in rinds:
                    rninds.append(i)
            fold = np.matrix([train[ind].A[0] for ind in rinds], dtype='float32')
            fold_resp = np.matrix([responses[0,ind] for ind in rinds], dtype='float32')
            new_train = np.matrix([train[ind].A[0] for ind in rninds], dtype='float32')
            new_resp = np.matrix([responses[0,ind] for ind in rninds], dtype='float32')
            log(np.shape(fold))
            log(np.shape(fold_resp))
            log(np.shape(new_train))
            log(np.shape(new_resp))

            svm = cv.SVM(new_train, new_resp)
            svm.train(new_train, new_resp, None, None, {'svm_type' : cv.SVM_C_SVC, 'kernel_type' : cv.SVM_LINEAR, 'degree' : 1})

            for i, sample in enumerate(new_train):
                pre = svm.predict(sample) == 1.
                act = new_resp[0,i] == 1.
                if pre:
                    if act:
                        tp += 1.
                    else:
                        fp += 1.
                else:
                    if act:
                        fn += 1.
                    else:
                        tn += 1.
                total += 1.

        total = 1.
        log("           actual")
        log("           pos,  neg")
        log("pred pos  %4f, %4f" % (tp / total, fp / total))
        log("     neg  %4f, %4f" % (fn / total, tn / total))

    def arff_data_writer(self, x_vals_t, y_vals_t, coord_names, fn = "data.arff"):
        train = np.matrix(x_vals_t, dtype='float32')
        responses = np.matrix(y_vals_t, dtype='float32')
        # write data to arff file
        arff_file = file(self.fos.package_loc + "//arff_files//" + fn, "w")
        alist = [(name, 1, []) for name in coord_names]
        alist.append(("collision_class", 0, LABEL_STRS))
        # coord_names.append("collision_class")
        resp_labels = [LABEL_STRS[int(v)] for v in responses.A[0]]
        arff_data = []
        for row_i, row in enumerate(train):
            arff_data.append(row.tolist()[0] + [resp_labels[row_i]])
        arff.arffwrite(arff_file, alist, arff_data)

    ##
    # Return random grasp configuration in entire space.
    def random_grasp_total(self):
        x = random.uniform(0.35, 0.75)
        y = random.uniform(-0.35, 0.35)
        r = random.uniform(0., np.pi)
        return x, y, r

##
# Contains external functions which employ OverheadGrasper to do various demos,
# tests, and evalutations. All functionality outside of specifically performing
# the grasp is contained here.

def testing(arm):
    if True:
        oger = OverheadGrasper(arm, active = True)
        # oger.random_forest_load()
        all_times, all_counts = [], []
        i = 0
        while not rospy.is_shutdown(): # and i < 4:
            oger.ki.pauser()
            oger.perform_grasp(0.5, 0.0)
            # times, counts = zip(*oger.apm.normal_dict_counts)
            # all_times.append(times)
            # all_counts.append(counts)
            # i += 1
        colors = ['red', 'blue', 'red', 'blue']
        i = 0
        for times, counts in zip(all_times, all_counts):
            plt.figure(1)
            plt.plot(times, counts, colors[i])
            i += 1
        plt.show()
        return

    oger = OverheadGrasper(arm, active = False)

    micro_dict = oger.fos.load_yaml_file("labels_dict_micro.yaml")
    mini_dict = oger.fos.load_yaml_file("labels_dict_mini.yaml")
    small_dict = oger.fos.load_yaml_file("labels_dict_small.yaml")
    large_dict = oger.fos.load_yaml_file("labels_dict.yaml")

    if False:
        train_data, train_res, test_data, test_res, coord_names, traj_labels, data_params = oger.multiple_dataset_compilation(large_dict, seed = 3, percent_split = 0.1, save_arff = False)
        traj_list = oger.eval_predicts("results_seed_3", "data_testing_seed_3")
        oger.analyze_testing_results(traj_list)
        oger.plot_testing_results(traj_list)
        return

    if False:
        train_data, train_res, test_data, test_res, coord_names, traj_labels, data_params = oger.multiple_dataset_compilation(large_dict, seed = 3, percent_split = 0.0, save_arff = False)
        oger.random_forest_build(train_data, train_res, num_learners = 20)
        oger.random_forest_split()
        oger.random_forest_load()
        log("Done")
        return

    return
    # oger.multiple_dataset_compilation(['big_medicine_1', 'big_medicine_2', 'big_medicine_3', 'empty_grasp'], fn = "big_medicine_data.arff")
    # return
    # oger.confusion_matrix_stats(np.mat([[42, 1, 6], [3, 100, 4], [10, 20, 80]]))
    # return
    for dir in large_dict:
        oger.process_all_signals(dir)

    train_data, train_res, test_data, test_res, coord_names, traj_labels, data_params = oger.multiple_dataset_compilation(large_dict, seed = 3, percent_split = 0.0, save_arff = False)
    oger.random_forest_build(train_data, train_res)
    for n_l in [20, 30, 50, 70, 90, 100]:
        log("X-valid with %d learners" % (n_l))
        oger.rtrees_cross_valid(train_data, train_res, traj_labels, data_params, num_learners = n_l)
        return
    # oger.random_forest_build(train_data, train_res)
    return

    traj_list = oger.eval_predicts("results_seed_2", "data_testing_seed_2")
    oger.analyze_testing_results(traj_list)
    return
    oger.plot_testing_results(traj_list)
    return

    oger.multiple_dataset_compilation(large_dict, fn = "data_testing_seed_2", seed = 2, save_arff = True)

    return

    # oger = OverheadGrasper(arm, active = True)
    # oger.apm.start_data_capture()
    # oger.ki.getch()
    # oger.apm.stop_data_capture()
    # oger.display_datasets_all(oger.apm.datasets)
    # oger.ki.getch()
    # return

    oger = OverheadGrasper(arm, active = False)
    grasp = oger.fos.load_pickle("collision_data//test//data 1.pickle")
    pp = PdfPages("test_data2.pdf")
    # oger.display_models_all(grasp[3], fig = 1)
    for sigma_i, sigma in enumerate(np.linspace(0.01, 0.2, 20)):
        oger.display_datasets_all(grasp[2], fig = sigma_i+1, show = False, sigma = sigma)
        log(sigma_i, sigma)
        pp.savefig()
    oger.display_datasets_all(grasp[2], fig = 22, show = False, sigma = 0.)
    pp.savefig()
    pp.close()
    return


    oger.compile_classification_data()
    return

    oger.save_annotated_plots()
    return

    oger.annotate_collision_times()
    return

    gname = "grasp_cdata_table1005498-000565-021608.pickle"
    # oger.process_signals(gname)
    grasp = self.fos.load_pickle(gname)
    oger.display_model_coodinate(grasp, 'gripper_pose', 2, coll_pt = 2.24)
    # oger.save_grasp_model_plots(grasp, "test.pdf")
    return
    # grasp = oger.fos.load_pickle("grasp_model0052--002-0039.pickle")
    # oger.display_model_coodinate(grasp, 'gripper_pose', 0)
    # return
    # oger.fix()
    # return
    # oger.generate_mean_models()
    oger.write_model_index_file()

##
# Main functionality entry point for stand alone use.
def grasping_main(arm, mode, args):

    if mode == "collision":
        oger = OverheadGrasper(arm, active=True, use_classifiers=True)
        oger.fos.make_directories(args[1])
        grasp_data = oger.collect_collision_data(args[0], args[1])

    elif mode == "new_collision":
        oger = OverheadGrasper(arm, active=True)
        behavior = OverheadGrasp(oger)
        if False:
            behavior = SidewaysGrasp(oger)
        oger.new_collect_collision_data(behavior, args[0], args[1], args[2], args[3], args[4])

    elif mode == "process_coll":
        oger = OverheadGrasper(arm, active=False)
        oger.fos.make_directories(args[0])
        oger.process_all_signals(args[0])

    elif mode == "process_dict":
        oger = OverheadGrasper(arm, active=False)
        dirs_dict = oger.fos.load_yaml_file(args[0])
        for dir in dirs_dict:
            oger.process_all_signals(dir)

    elif mode == "build_dict":
        oger = OverheadGrasper(arm, active=False)
        dirs_dict = oger.fos.load_yaml_file(args[0])
        # i_dirs_dict = {}
        # for dir in dirs_dict:
        #     coll_type = dirs_dict[dir]
        #     if coll_type > 0.:
        #         coll_type = 1.
        #     i_dirs_dict[dir] = coll_type
        # i_compiled_dataset = oger.multiple_dataset_compilation(
        #                          i_dirs_dict, seed = random.randint(0, 10000), 
        #                          percent_split = 0.0, save_arff = False,
        #                          is_impact=True)
        # oger.apm.impact_classifier.build(i_compiled_dataset,
        #                                  oger.apm.I_RTREE_CLASSIFIER)
        t_dirs_dict = {}
        for dir in dirs_dict:
            coll_type = dirs_dict[dir]
            if coll_type == 0.:
                continue
            t_dirs_dict[dir] = coll_type
        t_compiled_dataset = oger.multiple_dataset_compilation(
                                 t_dirs_dict, seed = random.randint(0, 10000), 
                                 percent_split = 0.0, save_arff = False,
                                 is_impact=False)
        oger.apm.impact_classifier.build(t_compiled_dataset,
                                         oger.apm.T_RTREE_CLASSIFIER)


    elif mode == "plots":
        oger = OverheadGrasper(arm, active=False)
        oger.fos.make_directories(args[0])
        oger.save_annotated_plots(args[0])

    elif mode == "annotate":
        oger = OverheadGrasper(arm, active=False)
        oger.fos.make_directories(args[0])
        oger.annotate_collision_times(args[0])

    elif mode == "grasp":
        oger = OverheadGrasper(arm, active = True, use_classifiers=True)
        rospy.sleep(1.)
        while not rospy.is_shutdown():
            oger.ki.pauser()
            oger.perform_grasp(args[0], args[1], gripper_rot=args[2])

    elif mode == "testing":
        testing(arm)

    else:
        err("Bad mode name")

##
# Main.
def main(args):
    rospy.init_node(node_name)
    # rospy.on_shutdown(die)
 
    ################### Options #######################
    import optparse
    p = optparse.OptionParser()
    p.add_option('-f', '--left', action='store_true', dest='left', default=False,
                 help="Use left arm (else right).")
    p.add_option('-c', '--collision', action='store_true', dest='collision', default=False,
                 help="Collect collision data.")
    p.add_option('-w', '--ncollision', action='store_true', dest='new_collision', default=False,
            help="Collect collision data: new function.")
    p.add_option('-n', '--num_colls', action='store', dest='num_colls', type="int",
                 help="Number of collisions to perform when collecting collisions data.")
    p.add_option('-l', '--label', action='store', dest='label', type="int",
                 help="Integer label of trajectory.")
    p.add_option('-s', '--notcollision', action='store_true', dest='not_collision', default=False,
            help="Free grasp.")
    p.add_option('-r', '--ft', action='store_true', dest='use_ft', default=False,
            help="Should we watch the ft sensor?")
    p.add_option('-e', '--detect', action='store_true', dest='detect', default=False,
            help="Should we detect collisions?")
    p.add_option('-x', '--prefix', action='store', dest='prefix', type="string",
                 help="Prefix directory to files when collecting collisions data.")
    p.add_option('-i', '--pcoll', action='store_true', dest='process_coll', default=False,
                 help="Run data processing for collision data.")
    p.add_option('-d', '--pcolldict', action='store_true', dest='process_dict', 
                 default=False, help="Run data processing for dict of dirs.")
    p.add_option('-b', '--buildforest', action='store_true', dest='build_dict', 
                 default=False, help="Build random forest (--pcolldict must be run first).")
    p.add_option('-y', '--dict', action='store', dest='dict', type="string",
                 help="Yaml file of directories to process for pcolldict.")
    p.add_option('-p', '--plots', action='store_true', dest='plot_grasps', default=False,
                 help="Make pdf plots of collision data.")
    p.add_option('-a', '--annotate', action='store_true', dest='annotate', default=False,
                 help="Run annotation suite for labeling collisions.")
    p.add_option('-g', '--grasp', action='store', dest='grasp', type="string",
                 help="Perform grasps repeatedly at x,y,rot (--grasp 0.5,0.0,0.3)")
    p.add_option('-t', '--testing', action='store_true', dest='testing', default=False,
                 help="Testing (UNSTABLE).")
    opt, opt_args = p.parse_args()
    ####################################################

    mode = ""
    if opt.left:
        arm = 1
    else:
        arm = 0
    args = []
    if opt.collision:
        mode = "collision"
        args.append(opt.num_colls)
        args.append(opt.prefix)
    if opt.new_collision:
        mode = "new_collision"
        args.append(opt.num_colls)
        args.append(opt.label)
        args.append(opt.not_collision)
        args.append(opt.use_ft)
        args.append(opt.detect)
    if opt.process_coll:
        mode = "process_coll"
        args.append(opt.prefix)
    if opt.process_dict:
        mode = "process_dict"
        args.append(opt.dict)
    if opt.build_dict:
        mode = "build_dict"
        args.append(opt.dict)
    if opt.plot_grasps:
        mode = "plots"
        args.append(opt.prefix)
    if opt.annotate:
        mode = "annotate"
        args.append(opt.prefix)
    if opt.grasp is not None:
        mode = "grasp"
        g_args = [float(v) for v in opt.grasp.split(",")]
        args.extend(g_args)
    if opt.testing:
        mode = "testing"
    if mode != "":
        grasping_main(arm, mode, args)
        return 0

    err("Must provide mode option")
    return 1

if __name__ == "__main__":
    sys.exit(main(sys.argv))
    
