#! /usr/bin/python

#TODO For release TODO
# Code cleanup!
# Remove print statements and replace with log statements
# Put grasp functions in own class

import numpy as np, math
import sys
import os
from threading import RLock
import threading

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

import actionlib

from std_msgs.msg import String
from geometry_msgs.msg import  Point, Pose, Quaternion, PoseStamped, PointStamped

import random
import hrl_lib.util
from hrl_lib.rutils import GenericListener
import hrl_lib.viz as viz
from hrl_lib import tf_utils
from hrl_lib.keyboard_input import KeyboardInput
from hrl_lib.transforms import rotX, rotY, rotZ
from hrl_lib.data_process import signal_list_variance, signal_smooth
from tf.transformations import *
# from hrl_pr2_lib.pr2_arms import PR2Arms
from perception_monitor import ArmPerceptionMonitor, generate_mean_grasp
from visualization_msgs.msg import MarkerArray, Marker
import dynamic_reconfigure.client
import tf
from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC, CURSOR_TOPIC, MOUSE_R_CLICK_TOPIC, MOUSE_R_DOUBLE_CLICK_TOPIC

# from pr2_gripper_reactive_approach.controller_manager import ControllerManager
from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult

from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC
from helpers import log, err, node_name

#SETUP_POS = (0.62, 0.0, 0.035)
#SETUP_POS_ANGS = [-0.6260155429349421, -0.53466276262236689, -1.9803303473514324, -1.1593322538276705, -0.89803655400181404, -1.4467120153069799, -2.728422563953746]
#MAX_JERK = 0.2

# TODO Externalize parameters

NUM_X = 7#3#9#7 #4#7
NUM_Y = 11#3#15#9 #4#20
NUM_N = 3 #4
NUM_ROT = 6
# near to far, left to right
RECT = ((0.45, 0.25), (0.70, -0.25))#((0.4, 0.35), (0.8, -0.35))#((0.45, 0.10), (0.65, -0.10))#((0.4, 0.20), (0.7, -0.20))#((0.4, 0.35), (0.8, -0.35)) #((0.55, 0.10), (0.7, -0.10)) #((0.4, -0.77), (0.75, 0.23))

ARM = 0 # right arm
if ARM == 0:
    armc = 'r'
else:
    armc = 'l'
HOVER_Z = -0.10
GRASP_DIST = 0.30 
GRASP_VELOCITY = 0.28 # 0.15
GRIPPER_POINT = np.array([0.23, 0.0, 0.0])
JOINTS_BIAS = [0., -.25, -1., 0., 0., 0.5, 0.]
BIAS_RADIUS = 0.012

GRASP_TIME = 2.0
SETUP_VELOCITY = 0.4 # 0.15

STD_DEV = 2.3 #3.5 #1.9
NOISE_DEV = 0.0
DETECT_ERROR = -0.02
STD_DEV_DICT = { "accelerometer" : np.array([6.4, 6.9, 6.9]),
                 "joint_angles" : np.array([4.8, 4.8, 4.8, 4.8, 400.0, 3.25, 400.0]),
                 "joint_efforts" : np.array([30.0, 20.0, 14.0, 23.0, 18.0, 10.0, 125.0]),
                 "joint_velocities" : np.array([3.4, 6.4, 18.4, 3.4, 3.4, 3.4, 3.4]),
                 "r_finger_periph_pressure" : np.array([10.0]*6), 
                 "r_finger_pad_pressure" : np.array([10.0]*15), 
                 "l_finger_periph_pressure" : np.array([10.0]*6), 
                 "l_finger_pad_pressure" : np.array([10.0]*15),
                 "gripper_pose" : np.array([1.0, 1.0, 4.0, 3.0, 3.0, 3.0, 3.0])}
TOL_THRESH_DICT = { "accelerometer" : np.array([1.3, 1.3, 1.3]),
                    "joint_velocities" : np.array([0.45]*7),
                    "joint_angles" : np.array([0.08, 0.08, 0.08, 0.08, 0.08, 0.06, 0.08]),
                    "joint_efforts" : np.array([4.0, 2.0, 1.0, 1.0, 1.0, 2.0, 2.0]),
                    "r_finger_periph_pressure" : np.array([3.0]*6), 
                    "r_finger_pad_pressure" : np.array([3.0]*15), 
                    "l_finger_periph_pressure" : np.array([3.0]*6), 
                    "l_finger_pad_pressure" : np.array([3.0]*15),
                    "gripper_pose" : np.array([0.1, 0.1, 0.3, 100.1, 100.1, 100.1, 100.1])}
# STD_DEV_DICT = { "accelerometer" : np.array([4.4, 4.9, 4.9]),
#                  "joint_angles" : np.array([2.8, 2.8, 3.8, 2.8, 400.0, 1.25, 400.0]),
#                  "joint_efforts" : np.array([30.0, 15.0, 11.0, 16.0, 12.0, 3.0, 125.0]),
#                  "joint_velocities" : np.array([3.4, 6.4, 18.4, 3.4, 3.4, 3.4, 3.4]),
#                  "r_finger_periph_pressure" : np.array([60.0]*6), 
#                  "r_finger_pad_pressure" : np.array([60.0]*15), 
#                  "l_finger_periph_pressure" : np.array([60.0]*6), 
#                  "l_finger_pad_pressure" : np.array([60.0]*15),
#                  "gripper_pose" : np.array([30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0])}
# TOL_THRESH_DICT = { "accelerometer" : np.array([0.3, 0.3, 0.3]),
#                     "joint_velocities" : np.array([0.45]*7),
#                     "joint_angles" : np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.04, 0.05]),
#                     "joint_efforts" : np.array([3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
#                     "r_finger_periph_pressure" : np.array([10.0]*6), 
#                     "r_finger_pad_pressure" : np.array([10.0]*15), 
#                     "l_finger_periph_pressure" : np.array([10.0]*6), 
#                     "l_finger_pad_pressure" : np.array([10.0]*15),
#                     "gripper_pose" : np.array([30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0])}
RESAMPLE_RATE = 8
PERCEPT_MON_LIST = None #["accelerometer"]
PERCEPT_GRASP_LIST = None 
PRESSURE_LIST =["r_finger_periph_pressure", 
                "r_finger_pad_pressure", 
                "l_finger_periph_pressure",
                "l_finger_pad_pressure"]
#                 "joint_efforts" ] #["joint_angles"]#["joint_velocities"] #["r_finger_periph_pressure"] #["joint_efforts"] #["joint_angles"]
##STD_DEV_DICT = None
MONITOR_WINDOW = 4 # 50


# TODO Externalize parameters into launch

PICKLES_LOC = "//pickles//"
GRASP_CONFIGS_FILE = "grasp_configs.pickle"
GRASP_DATA_FILE = "grasp_data.pickle"
GRASP_MODELS_FILE = "grasp_models.pickle"
GRASP_MODELS_TRIMMED_FILE = "grasp_models_trimmed.pickle"
ZEROS_FILE = "current_zeros.pickle"
SQR_DIFF_MAP = "sqr_diff_map.pickle"
GRASP_IND_PREFIX = "model_indexed//grasp_model"
GRASP_INDEX_FILE = "model_indexed//grasp_model_index.pickle"
GRASP_DATA_INDEX_FILE = "data_indexed//grasp_data_index.pickle"
GRASP_DATA_PREFIX = "data_indexed//grasp_data"
TEST_DATA_FILE = "test_data.pickle"

def _load_pickle(fn):
    global PACKAGE_LOC
    return hrl_lib.util.load_pickle(PACKAGE_LOC + PICKLES_LOC + fn)

def _save_pickle(p, fn):
    global PACKAGE_LOC
    hrl_lib.util.save_pickle(p, PACKAGE_LOC + PICKLES_LOC + fn)

def _file_exists(fn):
    global PACKAGE_LOC
    return os.path.exists(PACKAGE_LOC + PICKLES_LOC + fn)

def _setup_package_loc():
    global PACKAGE_LOC
    import os
    grep = os.popen("rospack find pr2_overhead_grasping|grep pr2_overhead_grasping")
    PACKAGE_LOC = grep.readlines()[0].rstrip()
##
# Transforms the given position by the offset position in the given quaternion
# rotation frame
#
# @param pos the current positions
# @param quat quaternion representing the rotation of the frame
# @param off_point offset to move the position inside the quat's frame
# @return the new position as a matrix column

def pauser(ki):
    if ki.kbhit():
        ch = ki.getch()
        if ch == 'p':
            log("PAUSED")
            while not rospy.is_shutdown() and ch != 'c':
                ch = ki.getch()
                rospy.sleep(0.1)
            log("CONTINUING")


##
# Contains functionality for overhead grasping motions, training, and setup.
# The primary functionality is contained in collect_grasp_data, which runs
# empty grasps to train the grasp models, and in perform_grasp, which runs
class OverheadGrasper():
    def __init__(self, arm = ARM, cm = None, apm = None, ki = None):
        if cm is None:
            cm = ControllerManager(armc)
        if apm is None:
            apm = ArmPerceptionMonitor(arm, tf_listener = cm.tf_listener,
                                       percept_mon_list=PERCEPT_GRASP_LIST) 
        if ki is None:
            ki = KeyboardInput()
            
        self.cm = cm
        self.apm = apm
        self.ki = ki

    ##
    # Transforms pos location by the off_point offset in the frame defined by pos and quat
    def transform_in_frame(self, pos, quat, off_point):
        invquatmat = np.mat(quaternion_matrix(quat))
        invquatmat[0:3,3] = np.mat(pos).T
        trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
        transpos = invquatmat * trans
        return transpos.T.A[0,0:3]

    ##
    # Returns a list of the (x, y, gripper rotations) grasp configurations the parameters
    # specify the algorithm is able to handle
    def get_xyr_list(self):
        grasp_xyr_list = []
        for x in np.linspace(RECT[0][0], RECT[1][0], NUM_X):
            for y in np.linspace(RECT[0][1], RECT[1][1], NUM_Y):
                for r in np.linspace(0., np.pi, NUM_ROT+1)[0:-1]:
                    grasp_xyr_list += [(x,y,r)]
        return grasp_xyr_list

    ##
    # Creates a PoseStamped message using the given pose at time now.
    def create_gripper_pose(self, x, y, z, quat):
        point = [x, y, z]
        point = self.transform_in_frame(point, np.array(quat), -GRIPPER_POINT).tolist()
        pose = point + quat
        goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
        goal_pose.header.stamp = rospy.Time.now()
        return goal_pose

    ##
    # TODO docs
    def get_gripper_pose(self, gripper_rot):
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
        point = self.transform_in_frame(point, gripper_pose, -GRIPPER_POINT).tolist()
        pose = point + gripper_pose.tolist()
        goal_pose = cf.create_pose_stamped(pose, "torso_lift_link")
        goal_pose.header.stamp = rospy.Time.now()
        return goal_pose

    ##
    # Moves arm to initial grasping position where grasping motion will begin.
    # TODO REMOVE
    def move_to_grasp_pos(self, grasp_pose, block=True):
        grasp_pose.header.stamp = rospy.Time.now()
        self.cm.move_cartesian_ik(grasp_pose, collision_aware = False, blocking = block,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          settling_time = rospy.Duration(1.0),
                          vel = SETUP_VELOCITY)

    ##
    # Bring the arm back up to the initial grasping position.
    # TODO REMOVE
    def upward_lift(self, grasp_pose, block=True):
        grasp_pose.header.stamp = rospy.Time.now()
        self.cm.move_cartesian_ik(grasp_pose, collision_aware = False, blocking = block,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          settling_time = rospy.Duration(1.0),
                          vel = GRASP_VELOCITY,
                          joints_bias = -np.array(JOINTS_BIAS), bias_radius = BIAS_RADIUS)

    ##
    # Perform overhead grasping motion.
    def downward_grasp(self, goal_pose, block=True):
        goal_pose.header.stamp = rospy.Time.now()
        return self.cm.move_cartesian_ik(goal_pose, collision_aware = False, blocking = block,
                          step_size = .005, pos_thres = .02, rot_thres = .1,
                          settling_time = rospy.Duration(GRASP_TIME),
                          joints_bias = JOINTS_BIAS, bias_radius = BIAS_RADIUS,
                          vel = GRASP_VELOCITY)

    ##
    # Write the index file for the model grasp configurations.
    def write_model_index_file(self, prefix = GRASP_IND_PREFIX, index = GRASP_INDEX_FILE):
        grasps = []
        xs = (RECT[1][0] - RECT[0][0]) / (NUM_X - 1)
        ys = (RECT[0][1] - RECT[1][1]) / (NUM_Y - 1)
        rs = np.pi / NUM_ROT
        st = [RECT[0][0] + xs/2., RECT[1][1] + ys/2., rs/2.]
        num_gen = 0
        for i in range(NUM_X - 1):
            for j in range(NUM_Y - 1):
                for k in range(NUM_ROT):
                    xyr = st
                    filename = prefix + "%04d-%04d-%04d.pickle" % (int(xyr[0]*100),int(xyr[1]*100),int(xyr[2]*100))
                    if _file_exists(filename):
                        grasps.append([xyr, filename])
                    st[2] += rs
                st[1] += ys
                st[2] = rs/2.
            st[0] += xs
            st[1] = RECT[1][1] + ys/2.
        _save_pickle(grasps, index)
    
    ##
    # Write the index file for the grasp data configurations.
    def write_index_file(self, prefix = GRASP_DATA_PREFIX, index = GRASP_DATA_INDEX_FILE):
        grasp_xyr_list = self.get_xyr_list()
        grasps = []
        for xyr in grasp_xyr_list:
            filename = prefix + "%04d-%04d-%04d.pickle" % (int(xyr[0]*100),int(xyr[1]*100),int(xyr[2]*100))
            log(filename, xyr)
            if _file_exists(filename):
                grasps.append([xyr, filename])
        _save_pickle(grasps, index)

    ##
    # Run grasp data collection.  Goes to each grasp configuration, performs grasp motion
    # at each configuration a number of times, and stores perception data over each trial.
    # Arms should be unobstructed so that a clean grasp motion is obtained.
    def collect_grasp_data(self, generate_models=False, skip_grasp=False):

        #    if restart:
        #        grasp_data = _load_pickle(GRASP_DATA)
        #        grasp_xyr_list = zip(*grasp_data)[0]
        #    else:
        #        grasp_xyr_list = get_xy_list()
        #        grasp_data = []
        grasp_xyr_list = self.get_xyr_list()
        grasp_data = []
        grasp_index = []

        log("Opening gripper")
        self.cm.command_gripper(0.08, -1.0, False)
        self.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

        for c, xyr in enumerate(grasp_xyr_list):
            # if c < 257:
            #     continue
            if rospy.is_shutdown():
                return
            log("---------------------------------------------------")
            log("%1.2f completion" % (float(c) / len(grasp_xyr_list)))
            zeros = None
            # Do grasping num_n times
            for i in range(NUM_N):
                pauser(self.ki)

                # Move to grasp position
                log("Moving to grasp position (%1.2f, %1.2f, %1.2f)" % xyr)
                grasp_pose = self.create_goal_pose(xyr[0], xyr[1], HOVER_Z, self.get_gripper_pose(xyr[2]))
                setup_result = self.cm.move_arm_pose_biased(grasp_pose, JOINTS_BIAS, SETUP_VELOCITY, 
                                        blocking = True)
                rospy.sleep(0.5)
                if skip_grasp:
                    break
                if setup_result is not None:
                    if zeros is None:
                        # wait for a bit to get the zeros here
                        rospy.sleep(0.5)
                        zeros = self.apm.get_zeros(0.6)

                    goal_pose = self.create_goal_pose(xyr[0], xyr[1], 
                                                  HOVER_Z - GRASP_DIST, self.get_gripper_pose(xyr[2]))
                    # start gathering data
                    self.apm.start_training()
                    # move arm down
                    log("Moving arm down")
                    sttime = rospy.Time.now().to_sec()
                    result = self.downward_grasp(goal_pose, block=True)
                    endtime = rospy.Time.now().to_sec()
                    log("dur:", endtime - sttime)
                    log(result)
                    log("Finished moving arm")
                    length = self.apm.stop_training()
                    log("length:", length)
                    log("len/dur", length / (endtime - sttime))
                    rospy.sleep(0.5)
                else:
                    break

            # can't move to initial position
            if setup_result is None or skip_grasp:
                continue

            fn = None
            if result != "no solution" and generate_models:
                models = self.apm.generate_models()
            else:
                models = None
            if result != "no solution":
                # grasp_data += [(xyr, None, self.apm.datasets, models, zeros)]
                fn = self.save_grasp([xyr, None, self.apm.datasets, models, zeros], GRASP_DATA_PREFIX)
            else:
                err("Grasp Failed, not adding data")
            grasp_index.append([xyr, fn])
            self.apm.clear_vars()

        log("Saving Data")
        #_save_pickle(grasp_data, GRASP_DATA_FILE)
        _save_pickle(grasp_index, GRASP_DATA_INDEX_FILE)
        log("Data save complete")
        return None #grasp_data

    ##
    # Generate models for the data collected.
    # TODO REMOVE
    def load_data_and_generate(self, grasp_data):
        ret = []
        n = 0
        for grasp in grasp_data:
            apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
            apm.datasets = grasp[2]
            apm.generate_models()
            ret += [(grasp[0], grasp[1], grasp[2], apm.models, grasp[4])]
            n += 1
            log("Generated %d models" % (n))
        # _save_pickle(ret, GRASP_MODELS_FILE)
        return ret

    ##
    # Averages over all trials of each (x, y, r) configuration to produce mean datasets.
    def generate_mean_models(self):
        grasps = []
        data_index = _load_pickle(GRASP_DATA_INDEX_FILE)
        num_gen = 0
        for grasp_i in data_index:
            log("Number of datasets smoothed:", num_gen)
            num_gen += 1
            grasp = _load_pickle(grasp_i[1])
            grasp = [grasp[0], grasp[1], grasp[2], generate_mean_grasp(grasp[2]), grasp[4]]
            self.save_grasp(grasp, GRASP_DATA_PREFIX)

    ##
    # Loads grasp with given file name.  Loading is done in separate thread to allow
    # robot to setup while file is still being read into memory.
    def grasp_loader(self, filename):
        class GLoader(threading.Thread):
            def __init__(self):
                threading.Thread.__init__(self)
                self.grasp = []

            def run(self):
                tg = _load_pickle(filename)
                for v in tg:
                    self.grasp.append(v)

        gl = GLoader()
        gl.start()
        return gl.grasp
    
#     ##
#     # Loads grasp with given xyr configuration.
#     def load_grasp(self, xyr, xyr_index = GRASP_DATA_INDEX_FILE):
#         return self.get_grasp_model(xyr[0], xyr[1], xyr[2], xyr_index = xyr_index, no_wait=False)
# 
#     ##
#     # TODO What?
#     def load_grasp(self, x, y, r, xyr_index = None):
#         if xyr_index is None:
#             xyr_index = _load_pickle(GRASP_INDEX_FILE)
#         for xyr in xyr_index:
#             if np.allclose([x, y, r], xyr[0]):
#                 return _load_pickle(xyr[1])
#         return None

    ##
    # Finds the closest grasp model for the given configuration. If no_wait is true,
    # the function will load the grasp model in a separate thread so the robot can
    # setup while waiting for the pickle to be loaded into memory.
    def get_grasp_model(self, x, y, r, xyr_index = None, grasp_data = None, no_wait = True):
        r = self.normalize_rot(r)
        if grasp_data is None:
            if xyr_index is None:
                xyr_index = _load_pickle(GRASP_INDEX_FILE)
        else:
            xyr_index = grasp_data
        if len(xyr_index) > 0:
            def dist(o):
                if np.allclose(o[0][2], 0.0):
                    # for loop around
                    rot_dist = min((np.pi - r) ** 2, (0. - r) ** 2)
                else:
                    rot_dist = (o[0][2] - r) ** 2
                return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2 + rot_dist
            xyr = min(xyr_index, key=dist)
            log("Distance to grasp model:", dist(xyr), "rotation diff:", xyr[0][2] - r )
            if grasp_data is None:
                if not no_wait:
                    return _load_pickle(xyr[1])

                return self.grasp_loader(xyr[1])
            else:
                return xyr
        else:
            err("Bad index file")
            return None
    ##
    # Creates model trajectories by averaging over the 8 corners of the cube of
    # interpolated points in (x, y, r) space.
    def generate_space_models(self):
        # TODO FIX
        if False:
            self.generate_mean_models()
        data_index = _load_pickle(GRASP_DATA_INDEX_FILE)


        xs = (RECT[1][0] - RECT[0][0]) / (NUM_X - 1)
        ys = (RECT[0][1] - RECT[1][1]) / (NUM_Y - 1)
        rs = np.pi / NUM_ROT
        st = [RECT[0][0] + xs/2., RECT[1][1] + ys/2., rs/2.]
        num_gen = 0
        for i in range(NUM_X - 1):
            for j in range(NUM_Y - 1):
                for k in range(NUM_ROT):
                    log("Number of models generated:", num_gen)
                    log("Current model:", st)
                    if rospy.is_shutdown():
                        return
                    num_gen += 1
                    if True or num_gen >= 230:
                        pauser(self.ki)
                        close_models = []
                        def dist1(o):
                            return np.fabs(o[0][0] - st[0]) + np.fabs(o[0][1] - st[1])
                        def rotdist(o):
                                if np.allclose(o[0][2], 0.0):
                                    # for loop around
                                    return min(np.fabs(np.pi - st[2]), np.fabs(0. - st[2]))
                                else:
                                    return np.fabs(o[0][2] - st[2])
                        if num_gen % 30 == 0:
                            # log(st)
                            # log(data_index)
                            log(map(dist1, data_index)[::100])
                        for ind, dist in enumerate(map(dist1, data_index)):
                            if dist < (1.1*xs/2. + 1.1*ys/2.) and rotdist(data_index[ind]) < 1.1*rs/2.:
                                close_models += [_load_pickle(data_index[ind][1])[3]]

                        if len(close_models) < 5:
                            err("Rejecting %1.2f, %1.2f, %1.2f with number close models = %d" %
                                    (st[0], st[1], st[2], len(close_models)))
                            st[2] += rs
                            continue

                        if len(close_models) > 8:
                            err(len(close_models), "problem")
                            err(st)
                            return
                        # find the average case over the several runs
                        models = {}
                        lens = [len(cm[cm.keys()[0]]) for cm in close_models]
                        max_len = np.max(lens)
                        zipped_cm = {}
                        for p in close_models[0]:
                            signals = []
                            for cm in close_models:
                                signals.append(zip(*cm[p]))
                            zipped_cm[p] = zip(*signals)
                            
                        for p in zipped_cm:
                            ammodels, vmodels = [], []
                            for signals in zipped_cm[p]:
                                avg_means_model = np.array([0.] * max_len)
                                sqr_means_model = np.array([0.] * max_len)
                                for i in range(max_len):
                                    n = 0
                                    for j in range(len(signals)):
                                        if i < len(signals[j]):
                                            avg_means_model[i] += signals[j][i]
                                            sqr_means_model[i] += signals[j][i] ** 2
                                            n += 1
                                    avg_means_model[i] /= n
                                    sqr_means_model[i] /= n

                                vars_model = sqr_means_model - avg_means_model ** 2
                                #vars_model = signal_smooth(sqr_means_model, 30) - avg_means_model ** 2
                                ammodels.append(avg_means_model)
                                vmodels.append(vars_model)

                            models[p] = {}
                            models[p]["mean"] = zip(*ammodels)
                            models[p]["variance"] = zip(*vmodels)

                        grasp = [st, None, None, models, None]
                        self.save_grasp(grasp, GRASP_IND_PREFIX)


                    st[2] += rs
                st[1] += ys
                st[2] = rs/2.
            st[0] += xs
            st[1] = RECT[1][1] + ys/2.
        self.write_index_file(prefix = GRASP_IND_PREFIX, index = GRASP_INDEX_FILE)

    ##
    # Saves zeros
    # TODO REMOVE
    def save_current_zeros(self, filename=ZEROS_FILE):
        apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
        zeros = apm.get_zeros()
        log("Zeros:", zeros)
        _save_pickle(zeros, filename)

    ##
    # Loads saved zeros
    # TODO REMOVE
    def load_current_zeros(self, filename=ZEROS_FILE):
        return _load_pickle(filename)

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
    def perform_grasp(self, x, y, z=None, gripper_rot = np.pi / 2., grasp=None, is_place=False,
                      is_grasp=True, collide = True, return_pose=True,
                      zeros = None, grasp_data=None):
        log("Performing grasp (%1.2f, %1.2f), rotation: %1.2f" % (x, y, gripper_rot))
        gripper_rot = self.normalize_rot(gripper_rot)

        if grasp is None:
            grasp = self.get_grasp_model(x, y, gripper_rot, grasp_data = grasp_data)

        #apm.datasets = grasp[2]
        #apm.models = grasp[3]
        #apm.model_zeros = grasp[4]
        #log("Model zeros:", grasp[4])

        log("Beginning grasp")

        result = "no solution"
        iters = 0
        while True:
            # Move to grasp position
            log("Moving to initial grasp position")
            # self.cm.command_joint_trajectory([grasp[1]], SETUP_VELOCITY, blocking = True)

            # log("Moving to final grasp position")
            grasp_pose = self.create_goal_pose(x, y, HOVER_Z, self.get_gripper_pose(gripper_rot))
            self.cm.move_arm_pose_biased(grasp_pose, JOINTS_BIAS, SETUP_VELOCITY, blocking = False)
            if not is_place and is_grasp:
                # open gripper
                log("Opening gripper")
                self.cm.command_gripper(0.08, -1.0, False)
                self.cm.gripper_action_client.wait_for_result(rospy.Duration(2.0))
            self.cm.wait_joint_trajectory_done()
            rospy.sleep(0.5)

            #if zeros is None:
            # zeros = apm.get_zeros(0.6)
            # log("Current zeros:", zeros)

            goal_pose = self.create_goal_pose(x, y, HOVER_Z - GRASP_DIST, self.get_gripper_pose(gripper_rot))

            class Collision():
                def __init__(self, cm):
                    self.cm = cm
                    self.collided = False
                # start gathering data
                def on_collision(self):
                    log("Collision!")
                    self.collided = True
                    while not self.cm.check_joint_trajectory_done():
                        self.cm.joint_action_client.cancel_all_goals()
                        rospy.sleep(0.01)
                        if rospy.is_shutdown():
                            return None
            col = Collision(self.cm)
            def rotate_acceleration(data):
                return data
                data -= zeros["accelerometer"]
                a = gripper_rot
                data[1] = np.cos(a) * data[1] - np.sin(a) * data[2]
                data[2] = np.sin(a) * data[1] + np.cos(a) * data[2]
                data += zeros["accelerometer"]
                return data
            class GripperSmooth():
                def __init__(self):
                    self.last_quat = None
                def smooth_angles(self, data):
                    quat = data[3:7]
                    if self.last_quat is None:
                        self.first = False
                        self.last_quat = quat
                        data[3:7] = 0.
                        return data
                    data[3:7] = 2. * np.arccos(np.sum(quat * self.last_quat))
                    self.last_quat = quat
                    return data


            gsmooth = GripperSmooth()
            transforms = { "accelerometer" : rotate_acceleration,
                           "gripper_pose" : gsmooth.smooth_angles}
                
            while not rospy.is_shutdown() and len(grasp) < 5:
                rospy.sleep(0.1)

            self.apm.begin_monitoring(grasp[3], 
                                 model_zeros = grasp[4], 
                                 sampling_rate = RESAMPLE_RATE,
                                 contingency=col.on_collision, window_size=MONITOR_WINDOW,
                                 current_zeros=zeros, std_dev_default=STD_DEV, 
                                 std_dev_dict = STD_DEV_DICT,
                                 tol_thresh_dict = TOL_THRESH_DICT,
                                 noise_dev_default=NOISE_DEV, collide=collide,
                                 transform_dict = transforms) 
            # move arm down
            log("Moving arm down")
            startt = rospy.Time.now().to_sec()
            result = self.downward_grasp(goal_pose, block = False)
            log("Grasp result:", result)
            if result == "no solution":
                # Jiggle it to see if it works
                self.apm.end_monitoring()
                dx = random.uniform(-0.030, 0.030)
                dy = random.uniform(-0.030, 0.030)
                dr = random.uniform(-0.2, 0.2)
                x += dx
                y += dy
                gripper_rot += dr
                gripper_rot = self.normalize_rot(gripper_rot)
                iters += 1
                # Tried too many times
                if iters == 6:
                    err("Cannot find IK solution!!!" * 8)
                    return
                else:
                    continue
            else:
                break

        if z is None:
            while not self.cm.check_joint_trajectory_done():
                if col.collided:
                    self.cm.joint_action_client.cancel_all_goals()
                rospy.sleep(0.01)
                if rospy.is_shutdown():
                    return None
        else:
            while not self.cm.check_joint_trajectory_done():
                if col.collided:
                    self.cm.joint_action_client.cancel_all_goals()
                wrist_pose = self.cm.get_current_wrist_pose_stamped("torso_lift_link")
                p = wrist_pose.pose.position
                o = wrist_pose.pose.orientation
                affector_pos = self.transform_in_frame([p.x, p.y, p.z],
                                                  [o.x, o.y, o.z, o.w], GRIPPER_POINT)
                if affector_pos[2] <= z:
                    log("Reached z position")
                    log("Affector position:", affector_pos)
                    while not self.cm.check_joint_trajectory_done():
                        self.cm.joint_action_client.cancel_all_goals()
                        rospy.sleep(0.01)
                    break
                rospy.sleep(0.01)
                if rospy.is_shutdown():
                    return None

        endt = rospy.Time.now().to_sec()
        log("Grasp duration:", startt - endt)
        log("Finished moving arm")

        avg_list = self.apm.end_monitoring()
        rospy.sleep(0.1)
        global z_sum
        for k in self.apm.z_avg:
            if not k in z_sum:
                z_sum[k] = np.copy(self.apm.z_avg[k])
            else:
                z_sum[k] += self.apm.z_avg[k]

        if not is_place and is_grasp:
            log("Closing gripper")
            self.cm.command_gripper(0.0, 30.0, True)
            log("Gripper closed")
        elif is_place:
            log("Opening gripper")
            self.cm.command_gripper(0.08, -1.0, True)
            log("Gripper opened")

        if return_pose and is_grasp:
            log("Moving back to grasp position")
            self.cm.move_arm_pose_biased(grasp_pose, JOINTS_BIAS, SETUP_VELOCITY, blocking = True)
        #self.upward_lift(self.cm, grasp_pose, block=True)
        log("Grasp complete!")
        
        return grasp, avg_list, col.collided, zeros


    ##
    # Uses matplotlib to display grasp data, models, tolerance ranges, and online data.
    # TODO Better docs
    def display_grasp_data_new(self, grasps, percept="accelerometer", indicies=range(3), 
                           std_dev=STD_DEV, noise_dev_add=NOISE_DEV, monitor_data=[], 
                           std_dev_dict=STD_DEV_DICT,
                           tol_thresh_dict=TOL_THRESH_DICT,
                           model_zeros=None, monitor_zeros=None,
                           plot_data=False, colors=None):
        import matplotlib.pyplot as plt
        if colors is None:
            colors = ['r', 'b', 'g', 'c']
        j = 0
        rows = len(grasps)
        cols = len(indicies)
        for i, grasp in enumerate(grasps):
            models = grasp[3]
            means = models[percept]["mean"]
            vars = models[percept]["variance"]
            if "smoothed_signals" in models[percept]:
                signals = models[percept]["smoothed_signals"]
                zip_signals = [ zip(*sig) for sig in signals ]
            else:
                signals = None
                zip_signals = []
            if "noise_variance" in models[percept]:
                noise_var = models[percept]["noise_variance"]
                noise_dev = np.sqrt(noise_var)
            else:
                noise_var = None
                noise_dev = None
            log("noise variance", noise_var)

            zip_means = np.array(zip(*means))
            zip_vars = np.array(zip(*vars))
            # log("monitor_data", monitor_data)
            zip_monitors = []
            for d in monitor_data:
                zip_monitors += [zip(*d[percept])]
            graph_means, graph_devs, mmax, mmin = [], [], [], []
            for w in indicies:
                graph_means += [zip_means[w]]
                graph_devs += [np.sqrt(zip_vars[w])]

                if std_dev_dict is not None and percept in std_dev_dict:
                    std_dev = std_dev_dict[percept][w]
                if tol_thresh_dict is not None and percept in tol_thresh_dict:
                    tol_thresh = tol_thresh_dict[percept][w]
                else:
                    tol_thresh = 0.

                if noise_dev is not None:
                    noise_dev_term = noise_dev[w] * noise_dev_add
                else:
                    noise_dev_term = 0.

                
                # mmax += graph_means[w] + graph_devs[w] * std_dev + noise_dev_term 
                #                                                                  + tol_thresh]
                # mmin += [graph_means[w] - graph_devs[w] * std_dev - noise_dev_term
                #                                                                  - tol_thresh]

            for k in range(len(indicies)):
                plt.subplot(rows, cols, i*cols + 1 + k)
                cnum = 0
                if plot_data:
                    for stream in grasp[2][percept]:
                        # s_mag = [np.sqrt(x[1][0]**2 + x[1][1]**2 + x[1][2]**2) for x in stream]
                        # plt.plot(s_mag,colors[cnum])
                        plt.plot([x[1][k] for x in stream],colors[cnum])
                        cnum += 1
                        cnum %= len(colors)
                for sig in zip_signals:
                    if indicies[k] < len(sig):
                        plt.plot(sig[indicies[k]], colors[2])
                if i < len(zip_monitors):
                    zip_monitor = zip_monitors[i]
                    len_diff = len(graph_means[k]) - len(zip_monitor[indicies[k]])
                    add_vals = [zip_monitor[indicies[k]][0]] * MONITOR_WINDOW
                    # add_vals = [zip_monitor[indicies[k]][0]] * len_diff
                    log(len_diff)
                    g = np.array(add_vals + list(zip_monitor[indicies[k]]))
                    # g += grasp[4][percept][indicies[k]] - monitor_zeros[i][percept][indicies[k]]
                    plt.plot(g.tolist(),colors[3])
                plt.plot(graph_means[k], colors[0])
                plt.plot(graph_means[k] + graph_devs[k], colors[1])
                plt.plot(graph_means[k] - graph_devs[k], colors[1])
                # plt.plot(mmax[k], colors[1])
                # plt.plot(mmin[k], colors[1])
                plt.title("%1.2f, %1.2f: coord %d" % (grasp[0][0], grasp[0][1], k))
            j += 1

        plt.show()

##
# Contains external functions which employ OverheadGrasper to do various demos,
# tests, and evalutations. All functionality outside of specifically performing
# the grasp is contained here.
class OverheadGraspManager():
    def __init__(self, arm = ARM, cm = None, apm = None, ki = None):
        if cm is None:
            cm = ControllerManager(armc)
        if apm is None:
            apm = ArmPerceptionMonitor(arm, tf_listener = cm.tf_listener,
                                       percept_mon_list=PERCEPT_GRASP_LIST) 
        if ki is None:
            ki = KeyboardInput()
            
        self.arm = arm
        self.cm = cm
        self.apm = apm
        self.ki = ki
        self.oger = OverheadGrasper(arm, cm, apm, ki)

    ##
    # Checks to see if anything is in the gripper currently (gripper is not fully closed).
    def is_obj_in_gripper(self):
        return self.cm.get_current_gripper_opening() > 0.01

    ##
    # Close gripper. Will continue applying force.
    def close_gripper(self, blocking = False):
        self.cm.command_gripper(0.0, 30.0, False)
        if blocking:
            self.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    ##
    # Open gripper fully.
    def open_gripper(self, blocking = False):
        self.cm.command_gripper(0.08, -1.0, False)
        if blocking:
            self.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    ##
    # This function proves that adding more than 3-4 grasps per location is unnecessary
    # Grasping trajectories are very repeatable given a specific grasping location.
    # The issue comes from more from shifting of the actual trajectory from expected
    # given a grasping location just a few cm off.  The models can handle the trajectory
    # so long as it is not much more than 5cm away from the training model.
    # TODO Does this still work?
    def test_num_samples(self, percept="accelerometer"):

        grasp_data = _load_pickle(GRASP_DATA_FILE)[0:1]
        data_list = grasp_data[0][2][percept]
        for i in [3, 6, 9]:
            self.apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_MON_LIST)
            grasp_data[0][2][percept] = data_list[0:i+1]
            self.apm.datasets = grasp_data[0][2]
            self.apm.generate_models()
            grasp_data[0][3][percept] = self.apm.models[percept]
            if i != 3:
                colors = ['m', 'y', 'k', 'r']
            else:
                colors = None
            display_grasp_data(grasp_data, percept, rows=1)

    ##
    # Uses the object_detection service detect objects on the table. Returns a list
    # of pairs [ [ x, y, z], rot ] which represent the centers and z orientation of
    # each object detected on the table.
    def detect_tabletop_objects(self):
        cbbf = ClusterBoundingBoxFinder()
        object_detector = rospy.ServiceProxy("/object_detection", TabletopDetection)
        detects = object_detector(True, False).detection
        object_detector.close()
        if detects.result != 4:
            err("Detection failed (err %d)" % (detects.result))
            return []
        table_z = detects.table.pose.pose.position.z
        objects = []
        for cluster in detects.clusters:
            (object_points, 
             object_bounding_box_dims, 
             object_bounding_box, 
             object_to_base_link_frame, 
             object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
            # log("bounding box:", object_bounding_box)
            (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
            angs = euler_from_quaternion(object_quat)
            log("angs:", angs)
            # position is half of height
            object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
            log("pos:", object_pos)
            log("table_z:", table_z)
            objects += [[object_pos, angs[2]]]
        return objects

    ##
    # Grasps the object defined by obj, a pair of location and rotation created by
    # detect_tabletop_objects. If collide is False, grasping will not detect collisions.
    def grasp_object(self, obj, grasp=None, collide=True, is_place=False, zeros=None):
        if collide:
            return self.oger.perform_grasp(obj[0][0], obj[0][1], is_place=is_place,
                                 gripper_rot=obj[1], grasp=grasp, zeros=zeros)
        else:
            return self.oger.perform_grasp(obj[0][0], obj[0][1], z=obj[0][2], is_place=is_place,
                                 gripper_rot=obj[1], grasp=grasp, zeros=zeros)


    ##
    # Given an (x, y) location on a table, grasp the closest object detected.
    # If repeat is True, will keep trying if the grasp fails.
    def grasp_closest_object(self, x, y, grasp=None, collide=True, repeat=True, zeros=None):
        def dist(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2

        grasped = False
        num_tries = 0

        self.point_head([x,y,-0.2])

        while not grasped and num_tries < 4:
            log("Detect in")
            self.change_projector_mode(True)
            rospy.sleep(0.6)
            detect_tries = 0
            objects = None
            while (objects is None or len(objects) == 0):
                objects = self.detect_tabletop_objects()
                rospy.sleep(0.6)
                detect_tries += 1
                if detect_tries == 3 and (objects is None or len(objects) == 0):
                    err("Cannot detect any objects")
                    return None, None, None, None, None
            log("Detect out")
            if len(objects) > 0:
                obj = min(objects, key=dist)

                # Get better look
                if True:
                    self.point_head(obj[0])
                    rospy.sleep(0.2)
                    log("Detect2 in")
                    objects = self.detect_tabletop_objects()
                    log("Detect2 out")
                    obj = min(objects, key=dist)

                self.change_projector_mode(False)
                (grasp, avg_list, collided, zeros) = self.grasp_object(obj, grasp=grasp, 
                                                           collide=collide, zeros=zeros)
                rospy.sleep(0.2)
                grasped = self.is_obj_in_gripper()
                if repeat and not grasped:
                    num_tries += 1
                    continue
                if grasped:
                    log("Grasp success!")
                    return obj, grasp, avg_list, collided, zeros
                else:
                    err("Grasp failure")
                    return obj, grasp, avg_list, collided, zeros
            else:
                log("No objects near point")
                return None, None, None, None, None

    ##
    # Object swap demo.
    # TODO Does this still work?
    def grasp_demo(self):
        grasps, avg_lists = [], []
        (obj1, grasp, avg_list, collided, zeros) = self.grasp_closest_object(0.45, 0.1)
        grasps += [grasp]
        avg_lists += [avg_list]
        obj1pl = [[obj1[0][0], obj1[0][1] - 0.18], obj1[1]]
        (grasp, avg_list, collided, zeros) = self.grasp_object(obj1pl, is_place=True, zeros=zeros)
        grasps += [grasp]
        avg_lists += [avg_list]
        (obj2, grasp, avg_list, collided, zeros) = self.grasp_closest_object(0.7, 0.1, zeros=zeros)
        grasps += [grasp]
        avg_lists += [avg_list]
        (grasp, avg_list, collided, zeros) = self.grasp_object(obj1, is_place=True, zeros=zeros)
        grasps += [grasp]
        avg_lists += [avg_list]
        (obj3, grasp, avg_list, collided, zeros) = self.grasp_closest_object(obj1pl[0][0], obj1pl[0][1], zeros=zeros)
        grasps += [grasp]
        avg_lists += [avg_list]
        (grasp, avg_list, collided, zeros) = self.grasp_object(obj2, is_place=True, zeros=zeros)
        grasps += [grasp]
        avg_lists += [avg_list]
        display_grasp_data(grasps[0:3], "accelerometer", monitor_data=avg_lists[0:3])
        display_grasp_data(grasps[3:-1], "accelerometer", monitor_data=avg_lists[3:-1])


    ##
    # Make models file that has no grasp data.
    # TODO REMOVE
    def trim_test_data(self, grasp_data=None):
        if grasp_data is None:
            grasp_data = _load_pickle(GRASP_DATA_FILE)
        new_data = []
        for grasp in grasp_data:
            new_data += [(grasp[0], grasp[1], None, grasp[3], grasp[4])]
        _save_pickle(new_data, GRASP_MODELS_FILE)
        return new_data

    ##
    # Saves a grasp configuration to a file.
    def save_grasp(self, grasp, prefix):
        filename = prefix + "%04d-%04d-%04d.pickle" % (int(grasp[0][0]*100),int(grasp[0][1]*100),int(grasp[0][2]*100))
        _save_pickle(grasp, filename)
        return filename

    ##
    # Breaks single grasp models file into indexed directory of individual files.
    def split_model_data(self, data=None):
        if data is None:
            data = _load_pickle(GRASP_MODELS_FILE)
        xyr_index = []
        n = 1
        for grasp in data:
            filename = GRASP_IND_PREFIX + "%03d-%03d-%03d.pickle" % (int(grasp[0][0]*100),int(grasp[0][1]*100),int(grasp[0][2]*100))
            xyr_index += [[grasp[0], filename]]
            _save_pickle(grasp, filename)
            n += 1
        _save_pickle(xyr_index, GRASP_INDEX_FILE)

    ##
    # Resamples model data that is already split.  Only every sampling_rate'th sample
    # is included in the new data.
    def resample_split_model_data(self, apm, models=None, sampling_rate=RESAMPLE_RATE):
        if models is None:
            models = _load_pickle(GRASP_MODELS_FILE)
        new_models = []
        for model in models:
            new_model = self.apm.resample_and_thin_data(model[3], sampling_rate)
            new_models.append([model[0], None, None, new_model, model[4]])
        self.split_model_data(new_models)

    
    ##
    # TODO DOCS
    def resample_model_data(self, apm, sampling_rate=RESAMPLE_RATE):
        xyr_index = _load_pickle(GRASP_INDEX_FILE)
        for xyr in xyr_index:
            grasp = _load_pickle(xyr[1])
            new_model = self.apm.resample_and_thin_data(grasp[3], sampling_rate)
            _save_pickle([grasp[0], None, None, new_model, grasp[4]], xyr[1])



    ##
    # TODO REMOVE
    def compute_sqr_diff_map(self, percept="accelerometer"):
        xys = get_xy_list()
        def get_xy(i, j):
            return xys[i * NUM_Y + j]
        def model_sqr_err(m1, xy, i, j):
            n = 0
            sum = 0.
            cxy = get_xy(i, j)
            g2 = self.get_grasp_model(cxy[0], cxy[1])
            if np.allclose(g2[0], xy):
                return [-1.]
            m2 = g2[3][percept]["mean"]
            for i in range(min(len(m1),len(m2))):
                sum += (m1[i] - m2[i]) ** 2
                n += 1
            return sum / n
        xy_index = _load_pickle(GRASP_INDEX_FILE)
        if len(xys) > 0:
            map = [] 
            for i in range(NUM_X):
                map += [[]]
                for j in range(NUM_Y):
                    cen_xy = get_xy(i, j)
                    cen_grasp = self.get_grasp_model(cen_xy[0], cen_xy[1])
                    cen_grasp_model = cen_grasp[3][percept]["mean"]
                    if not np.allclose(cen_xy, cen_grasp[0]):
                        map[-1] += [0.]
                        continue
                    else:
                        err = 0.
                        n = 0
                        new_err = model_sqr_err(cen_grasp_model, cen_xy, i+1, j)
                        if new_err[0] != -1.:
                            err += new_err
                            n += 1
                        new_err = model_sqr_err(cen_grasp_model, cen_xy, i-1, j)
                        if new_err[0] != -1.:
                            err += new_err
                            n += 1
                        new_err = model_sqr_err(cen_grasp_model, cen_xy, i, j+1)
                        if new_err[0] != -1.:
                            err += new_err
                            n += 1
                        new_err = model_sqr_err(cen_grasp_model, cen_xy, i, j-1)
                        if new_err[0] != -1.:
                            err += new_err
                            n += 1
                        err = np.linalg.norm(err) ** 2 / len(err)
                        if n > 0:
                            map[-1] += [err / n]
                        else:
                            map[-1] += [0.]
            return map
        else:
            err("Bad index file")
            return None

    ##
    # TODO REMOVE
    def sqr_diff_viz(self):
        # _save_pickle(self.compute_sqr_diff_map(), SQR_DIFF_MAP)
        map = _load_pickle(SQR_DIFF_MAP)
        map.reverse()
        maxarg = max([max(row) for row in map])
        log(np.mat(map))
        for i in range(len(map)):
            for j in range(len(map[i])):
                if map[i][j] == -1.:
                    map[i][j] = maxarg*1.5
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.matshow(map)
        plt.show()

    ##
    # TODO REMOVE
    def process_data(self, grasp_data=None):
        log("Loading data")
        if grasp_data is None:
            grasp_data = _load_pickle(GRASP_DATA_FILE)
        log("Data loaded, generating models")
        grasp_data = self.load_data_and_generate(grasp_data)
        log("Saving models")
        log("Trimming test data")
        model_data = self.trim_test_data(grasp_data)
        log("Splitting models")
        self.split_model_data(model_data)
        log("Done processing data")

    ##
    # TODO REMOVE
    def calc_model(self, x, y, r, window_len= 200, samples=50, xyr_index = None, resample = 10):
        if xyr_index is None:
            xyr_index = _load_pickle(GRASP_INDEX_FILE)
        xs = (RECT[1][0] - RECT[0][0]) / (NUM_X - 1)
        ys = (RECT[0][1] - RECT[1][1]) / (NUM_Y - 1)
        rs = np.pi / NUM_ROT
        def dist1(o):
            return np.fabs(o[0][0] - x) + np.fabs(o[0][1] - y)
        def dist2(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2
        def rotdist(o):
                if np.allclose(o[0][2], 0.0):
                    # for loop around
                    return min(np.fabs(np.pi - r), np.fabs(0. - r))
                else:
                    return np.fabs(o[0][2] - r)

        n = 1

        close_models = []
        for i, dist in enumerate(map(dist1, xyr_index)):
            if np.allclose([x, y, r], xyr_index[i][0]):
                close_models = [[xyr_index[i][0], _load_pickle(xyr_index[i][1])]]
                break
            if dist < xs + ys and rotdist(xyr_index[i]) < rs:
                close_models += [[xyr_index[i][0], _load_pickle(xyr_index[i][1])]]

        ret_models = {}

        for k in close_models[0][1][3]:
            close_stream_means = []
            minlen = min([len(cm[1][3][k]["mean"]) for cm in close_models])
            for close_model in close_models:
                mean = close_model[1][3][k]["mean"][0:minlen]
                stream_means = zip(*mean)
                close_stream_means += [[close_model[0], np.array(stream_means)]]

            expected_means, expected_vars = [], []
            for i in range(len(close_stream_means[0][1])):
                close_signals = []
                sum = np.array([0.]*len(close_stream_means[0][1][0]))
                distsum = 0.
                for close_stream in close_stream_means:
                    close_signals += [close_stream[1][i]]
                    dist = dist2(close_stream)
                    sum += close_stream[1][i] / dist
                    distsum += 1. / dist
                expected_mean = sum / distsum
                expected_var = signal_list_variance(close_signals, expected_mean, 
                                                                      window_len, samples,
                                                                      resample)
                expected_means += [expected_mean[::resample]]
                expected_vars += [expected_var]

            ret_model = { "mean" : zip(*expected_means), "variance" : zip(*expected_vars) }
            ret_models[k] = ret_model
        return [[x, y, r], None, None, ret_models, None]


    ##
    # Return random grasp configuration in trained space.
    def random_grasp(self):
        xs = (RECT[1][0] - RECT[0][0]) / (NUM_X - 1)
        ys = (RECT[0][1] - RECT[1][1]) / (NUM_Y - 1)
        x = random.uniform(RECT[0][0]-xs/2., RECT[1][0]+xs/2.)
        y = random.uniform(RECT[1][1]-ys/2., RECT[0][1]+ys/2.)
        r = random.uniform(0., np.pi)
        return x, y, r

    ##
    # Return random grasp configuration that has specifically been trained on.
    def random_known_grasp(self):
        xyr_index = _load_pickle(GRASP_INDEX_FILE)
        ind = random.randint(0,len(xyr_index)-1)
        return xyr_index[ind][0]

    ##
    # TODO REMOVE
    def test_random_grasps(self, num_grasps=100):
        monitor_data = []
        grasps = []
        collided_list = []
        num_collided = 0
        test_data = []
        zeros = None
        self.open_gripper()

        for i in range(num_grasps):
            if rospy.is_shutdown():
                break
            x, y, rot = self.random_grasp()

            (grasp, monitor, collided, zeros) = self.oger.perform_grasp(x, y, gripper_rot = rot, 
                                               collide=False, is_grasp = False, 
                                               zeros=zeros)
            test_data += [{"loc" : (x, y, rot), 
                #"model" : grasp, 
                           "monitor" : monitor, 
                           "collided" : collided,
                           "zeros" : zeros}]
        
            if collided:
                log("COLLIDED\n"*10)
                num_collided += 1

        log("Accuracy: %d collisions out of %d grasps (%1.2f)" % (num_collided, num_grasps, 1. - float(num_collided) / num_grasps))
        _save_pickle(test_data, TEST_DATA_FILE)

    ##
    # TODO Does this work?
    def calc_false_pos(self, test_data=None, xyr_index=None, apm=None):
        if test_data is None:
            test_data = _load_pickle(TEST_DATA_FILE)
        if apm is None:
            apm = ArmPerceptionMonitor(ARM, percept_mon_list=PERCEPT_GRASP_LIST)
        num_collisions = 0
        indicies = {}
        for test in test_data:
            model = self.get_grasp_model(test["loc"][0], test["loc"][1], test["loc"][2], 
                                    xyr_index=xyr_index)
            collision = self.apm.simulate_monitoring(test["monitor"], models=model[3], 
                                 model_zeros = test["zeros"],
                                 window_size=MONITOR_WINDOW,
                                 std_dev_default=STD_DEV, 
                                 std_dev_dict = STD_DEV_DICT,
                                 tol_thresh_dict = TOL_THRESH_DICT,
                                 noise_dev_default=NOISE_DEV)
            if collision is not None:
                num_collisions += 1
                k, index, diff = collision
                if not k in indicies:
                    indicies[k] = [0]*len(model[3][k]["mean"][0])
                indicies[k][index] += 1
        log("Accuracy: %d collisions out of %d grasps (%1.2f)" % (num_collisions, len(test_data), 1. - float(num_collisions) / len(test_data)))
        log(indicies)
        return float(num_collisions) / len(test_data)

    ##
    # Place markers on all of the detected objects so we can see them in rviz.
    # TODO REMOVE
    def visualize_objs(self):
        objs = self.detect_tabletop_objects()
        pubm = rospy.Publisher('grasp_obj_positions', Marker)
        pubma = rospy.Publisher('grasp_obj_positions_array', MarkerArray)
        markers = []
        for obj in objs:
            pos, angle = np.mat(obj[0]).T, obj[1]
            marker = viz.single_marker(pos, np.mat(quaternion_about_axis(angle, (0, 0, 1))).T, "arrow", "torso_lift_link")
            marker.header.stamp = rospy.Time.now()
            markers.append(marker)
            pubm.publish(marker)
        ma = MarkerArray(markers)
        i = 0
        while not rospy.is_shutdown():
            pubm.publish(markers[i])
            rospy.sleep(2.1)
            i = (i + 1) % len(markers)
        log(objs)
        rospy.spin()

    ##
    # Waits for a laser double click.  Must have laser_interface nodes running.
    # The waiting loop has two modes.  In the inital mode, the loop will wait until
    # a cursor3d message is posted and the mouse is double clicked delay_time apart.
    # The function then breaks and returns the last cursor3d pt and False.  The second
    # mode has the same functionality and is initialized by double clicking the right
    # button.  When beginning the second mode the head will be pointed up.  To exit,
    # simply double click the right button again and the robot will look back down.
    # The second mode will return a True as the second argument.
    # TODO Fix face detect code.
    def get_laser_dclick(self, tf_listener, frame = "/torso_lift_link", delay_time = 3.0):
        c3d_lis = GenericListener("c3dlisnode", PointStamped, CURSOR_TOPIC, 10.0)
        dc_lis = GenericListener("dclisnode", String, MOUSE_DOUBLE_CLICK_TOPIC, 10.0)
        rdc_lis = GenericListener("rdclisnode", String, MOUSE_R_DOUBLE_CLICK_TOPIC, 10.0)
        # face_detect_cli = actionlib.SimpleActionClient('face_detector_action', face_detector.msg.FaceDetectorAction)
        log("Waiting for laser click...")
        while not rospy.is_shutdown():
            if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                log("Double click heard")
                msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
                log(msg)
                if msg is not None:
                    # @TODO contain this code?
                    if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
                        now = rospy.Time.now()
                        tf_listener.waitForTransform(msg.header.frame_id, frame, 
                                                     now, rospy.Duration(4.0))
                        tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
                        tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
                        pt = tfmat[0:3,3]
                        #(trans, rot) = tf_listener.lookupTransform(msg.header.frame_id, frame, now)
                        #pt = [msg.point.x - trans[0], msg.point.y - trans[1], msg.point.z - trans[2]]
                        log("pt", pt)
                        return [pt[0,0], pt[1,0], pt[2,0]], False

            if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                log("Right double click heard")
                log("Double click on person to hand-off")
                self.point_head([1.0, 0.0, 0.05], block = True)

                # begin waiting for person selection
                while not rospy.is_shutdown():
                    if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                        log("Double click heard")
                        msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
                        log(msg)
                        if msg is not None:
                            if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
                                now = rospy.Time.now()
                                tf_listener.waitForTransform(msg.header.frame_id, frame, 
                                                             now, rospy.Duration(4.0))
                                tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
                                tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
                                lspt = tfmat[0:3,3]
                                lspt = [ lspt[0,0], lspt[1,0], lspt[2,0]]
                                log("pt", lspt)
                    
                                # log("Waiting for face detection server")
                                # face_detect_cli.wait_for_server()
                                # face_req = face_detector.msg.FaceDetectorGoal()
                                # face_detect_cli.send_goal(face_req)
                                # face_detect_cli.wait_for_result()
                                # face_poss = face_detect_cli.get_result().face_positions
                                # log("Got face results:", face_poss)

                                # # transform face positions to our frame
                                # face_pts = []
                                # for face_pos in face_poss:
                                #     now = rospy.Time.now()
                                #     tf_listener.waitForTransform(face_pos.header.frame_id, frame, 
                                #                                  now, rospy.Duration(4.0))
                                #     tfmat = tf_utils.transform(frame, face_pos.header.frame_id, tf_listener)
                                #     tfmat *= np.mat([[face_pos.pos.x], [face_pos.pos.y], [face_pos.pos.z], [1.0]])
                                #     fcpt = tfmat[0:3,3]
                                #     fcpt = [ fcpt[0,0], fcpt[1,0], fcpt[2,0]]
                                #     face_pts.append(fcpt)

                                # log("Face locations", face_pts)
                                # for face_pt in face_pts:
                                #     dist = np.sqrt((face_pt[0] - lspt[0])**2 + (face_pt[1] - lspt[1])**2)
                                #     if dist < 0.35 and face_pt[2] > 0.3:
                                #         return face_pt, True
                                lspt[2] += 0.2
                                return lspt, True

                    if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                        log("Right double click heard")
                        log("Going back to table")
                        self.point_head([0.5, 0.0, -0.2], block = True)
                        break

                    rospy.sleep(0.01)
                    


            rospy.sleep(0.01)
        return None

    ##
    # Points head at given point in given frame.
    def point_head(self, point, velocity = 0.6, frame="/torso_lift_link", block = True):
        head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
        head_action_client.wait_for_server()
        goal = PointHeadGoal()
        goal.target = cf.create_point_stamped(point, frame)
        goal.pointing_frame = "/narrow_stereo_optical_frame"
        goal.max_velocity = velocity

        head_action_client.send_goal(goal)

        if not block:
            return 0

        finished_within_time = head_action_client.wait_for_result(rospy.Duration(20))
        if not finished_within_time:
            head_action_client.cancel_goal()
            rospy.logerr("timed out when asking the head to point to a new goal")
            return 0

        return 1

    ##
    # Commands the robot to move to a joint configuration directed at the given
    # point as if handing off in the general direction.  The point does not have
    # to be reachable by the gripper and is generally assumed to be outside of the
    # robot's reach.  The final pose is biased so that the arm usually looks natural,
    # i.e., it is similar to the way a human would handoff.
    def hand_over_object(self, x, y, z, offset = 0.2, blocking = True):
        if x < 0.2 or z < -1.0 or z > 1.0 or y > 2. or y < -2.:
            err("Cannot handoff to this location")
            return

        pt = np.array([x,y,z])
        quat = quaternion_about_axis(0.9, (0, 0, 1))
        dist = np.linalg.norm(pt)
        start_angles = self.cm.get_current_arm_angles()
        log(start_angles)
        ptnorm = pt / dist
        dist -= offset
        joints = None
        while not rospy.is_shutdown() and pt[0] > 0.2:
            pt = dist * ptnorm
            log("dist", dist)
            pose = self.oger.create_gripper_pose(pt[0], pt[1], pt[2], quat.tolist())
            log(pose)

            
            HANDOFF_BIAS = [0., -.25, -100., 200., 0.0, 200.0, 0.]
            joints = self.cm.ik_utilities.run_biased_ik(pose, HANDOFF_BIAS, num_iters=30)
            if joints is not None:
                break
            dist -= 0.1
    #   joints = [-0.1, -0.15, -1.2, -0.7, 0., -0.2, 0.]
        self.cm.command_joint_trajectory([joints], 0.27, blocking = blocking)


    ##
    # Turns the projector_mode to auto and the trigger mode on the narrow to either
    # with the projector (on = True) or without the projector (on = False).
    def change_projector_mode(self, on):
        client = dynamic_reconfigure.client.Client("camera_synchronizer_node")
        node_config = client.get_configuration()
        node_config["projector_mode"] = 2
        if on:
            node_config["narrow_stereo_trig_mode"] = 3
        else:
            node_config["narrow_stereo_trig_mode"] = 4
        client.update_configuration(node_config)

    ##
    # Move arm to setup position, outside of vision area so that the arm is not in
    # the way when looking at the table.
    def move_to_setup(self, blocking = True):
        joints = [-0.62734204881265387, -0.34601608409943324, -1.4620635485239604, -1.2729772622637399, -7.5123303230158518, -1.5570651396529178, -5.5929916630672727] 
        self.cm.command_joint_trajectory([joints], 0.62, blocking = blocking)


    ##
    # TODO REMOVE
    def laser_interface_demo(self):
        self.move_to_setup()

        x, y, z = self.get_laser_dclick(self.cm.tf_listener)
    #   x, y, z = 0.7, 0.0, -0.3
        self.point_head([x,y,z], block = True)

        self.grasp_closest_object(x, y)
        self.change_projector_mode(on=False)

        x, y, z = self.get_laser_dclick(self.cm.tf_listener)
    #   x, y, z = 1.2, 0.0, 0.3
        self.point_head([x,y,z+0.2], block = True)
        # self.hand_over_object(x, y, z, cm, apm)
    
    ##
    # Block until a noticable difference in fingertip pressure has been noticed.
    # Monitor begins by zeroing out the sensors and using the perception monitor
    # to determine when an abnormal pressure event has occured.
    def monitor_pressure(self):
        apm = ArmPerceptionMonitor(ARM, percept_mon_list=PRESSURE_LIST)
        rospy.sleep(1.0)

        models = {}
        for k in PRESSURE_LIST:
            if "periph" in k:
                models[k] = {}
                models[k]["mean"] = np.array([np.array([0.]*6)])
                models[k]["variance"] = np.array([np.array([40.]*6)])
            if "pad" in k:
                models[k] = {}
                models[k]["mean"] = np.array([np.array([0.]*15)])
                models[k]["variance"] = np.array([np.array([40.]*15)])

        self.apm.begin_monitoring(models, only_pressure=True)
        log("Monitoring pressure...")
        while not rospy.is_shutdown() and not self.apm.failure:
            rospy.sleep(0.01)
        log("Pressure triggered")

    ##
    # Runs looping grasping demos. Each of the three modes will continue repeating until
    # the program is exited.  Mode "random" will do random blind grasps in its training
    # region.  Mode "vision" will use vision perception to grasp the object closest
    # to a point roughly in the center of the table.  Mode "laser" requires laser_interface
    # nodes to be running and will allow the user to control the grasps with the mouse.
    def grasping_demos(self, mode):
        self.open_gripper(blocking = True)
        
        monitor_data = []
        monitor_zeros = []
        grasps = []
        collided_list = []
        num_grasps = 100
        num_collided = 0
        zeros = None
        global z_sum
        z_sum = {}
        self.move_to_setup()

        for i in range(num_grasps):
            self.point_head([0.5, 0.0, -0.2], block=False)
            if rospy.is_shutdown():
                break
            #x, y, rot = random.uniform(.45, .65), random.uniform(-.2, .2), random.uniform(0., 3.14)

            if mode == "random":
                #rot = 0.
                #global STD_DEV_DICT
                #STD_DEV_DICT = None
                if True:
                    # TODO FIX THIS
                    x, y, rot = self.random_grasp()
                else:
                    x, y, rot = self.random_known_grasp()
                (grasp, monitor, collided, zeros) = self.oger.perform_grasp(x, y, gripper_rot = rot, 
                                                   collide=True, is_grasp = False, 
                                                   #grasp_data = grasp_data,
                                                   zeros=zeros)
            elif mode == "vision":
                x, y = 0.5, 0.0
                (obj, grasp, monitor, collided, zeros) = self.grasp_closest_object(x, y, collide=True, zeros=zeros)
                rospy.sleep(0.3)
                if self.is_obj_in_gripper():
                    x, y, rot = self.random_grasp()
                    self.oger.perform_grasp(x, y, gripper_rot=rot, zeros = zeros, collide=True, is_place=True)
            elif mode == "laser":
                obj = None
                while obj is None:
                    # keep looping until we have grasped an object
                    laser_pt, is_handoff = self.get_laser_dclick(self.cm.tf_listener)
                    x, y, z = laser_pt
                    if not is_handoff:
                        (obj, grasp, monitor, collided, zeros) = self.grasp_closest_object(x, y, collide=True, zeros=zeros)
                # get next location for either grasp or handoff
                laser_pt, is_handoff = self.get_laser_dclick(self.cm.tf_listener)
                x, y, z = laser_pt
                log(" X Y Z")
                log(x, y, z)
                if is_handoff:
                    self.point_head([x, y, z], block = False)
                    self.hand_over_object(x, y, z)
                    self.monitor_pressure()
                    self.open_gripper()
                else:
                    self.oger.perform_grasp(x, y, gripper_rot=obj[1], zeros = zeros, collide=True, is_place=True)
                self.move_to_setup(blocking = False)
            grasps += [grasp]
            monitor_data += [monitor]
            collided_list += [collided]
            monitor_zeros += [zeros]
        
            if collided:
                log("COLLIDED")
                log("COLLIDED")
                log("COLLIDED")
                num_collided += 1

            rospy.sleep(2.0)
            if self.ki.kbhit():
                if PERCEPT_GRASP_LIST is None:
                    percept = "joint_efforts"
                else:
                    percept = PERCEPT_GRASP_LIST[0]
                percept = "accelerometer"
                inds = range(3) #[3, 4, 5, 6]
                display_grasp_data(grasps[-3:], percept, monitor_data=monitor_data[-3:], monitor_zeros=monitor_zeros[-3:], indicies=inds)
                # log(monitor_data[-3:])
                break

        log("Accuracy: %d collisions out of %d grasps (%1.2f)" % (num_collided, num_grasps, 1. - float(num_collided) / num_grasps))
        test_data = [grasps, monitor_data, collided_list]
        for k in z_sum:
            log(z_sum[k] / num_grasps)
        return 0
        _save_pickle(test_data, "test_runs.pickle")

    ##
    # Main functionality entry point for stand alone use.
    def grasping_main(self, mode):
        if mode == "collect":
            grasp_data = self.oger.collect_grasp_data(generate_models=False, skip_grasp=False)

        elif mode == "process":
            self.oger.generate_space_models()
            self.oger.write_model_index_file(prefix = GRASP_IND_PREFIX, index = GRASP_INDEX_FILE)
            self.resample_model_data(self.apm)

        elif mode == "random":
            self.grasping_demos(mode)

        elif mode == "vision":
            self.grasping_demos(mode)

        elif mode == "laser":
            self.grasping_demos(mode)

        else:
            err("Bad mode name")

##
# Main.
def main(args):
    rospy.init_node(node_name)
    # rospy.on_shutdown(die)
    _setup_package_loc()
 
    ################### Options #######################
    import optparse
    p = optparse.OptionParser()
    p.add_option('-d', '--cdata', action='store_true', dest='collect_data', default=False,
                 help="Run data collection")
    p.add_option('-p', '--pdata', action='store_true', dest='process_data', default=False,
                 help="Run data processing")
    p.add_option('-r', '--randgrasp', action='store_true', dest='random_grasps', default=False,
                 help="Grasp demo with random grasps, no vision.")
    p.add_option('-v', '--visiongrasp', action='store_true', dest='vision_grasps', default=False,
                 help="Grasp demo that picks up closest object to a point in the center of the table, with vision.")
    p.add_option('-l', '--lasergrasp', action='store_true', dest='laser_grasps', default=False,
                 help="Grasp demo with laser pointer control, with vision.")
    opt, args = p.parse_args()
    ####################################################

    mode = ""
    if opt.collect_data:
        mode = "collect"
    if opt.process_data:
        mode = "process"
    if opt.random_grasps:
        mode = "random"
    if opt.vision_grasps:
        mode = "vision"
    if opt.laser_grasps:
        mode = "laser"
    if mode != "":
        ogm = OverheadGraspManager()
        ogm.grasping_main(mode)
        return 0

    err("Must provide mode option")
    return 1

if __name__ == "__main__":
    sys.exit(main(sys.argv))
    
