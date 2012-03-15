#! /usr/bin/python

import numpy as np, math
import scipy.stats as stats
from threading import RLock
import random
import sys
import multiprocessing as mp

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy
import tf

from hrl_lib.msg import FloatArray
from hrl_lib.rutils import GenericListener, ros_to_dict, RateCaller
from tf.transformations import *
from helpers import log, err, node_name, FileOperations
import classifiers as cfiers

from std_msgs.msg import Float64MultiArray
from pr2_msgs.msg import AccelerometerState, PressureState
from sensor_msgs.msg import JointState

import threading
import functools as ft
import Queue
import matplotlib.pyplot as plt

import time, string

# def pool_loading(fns):
#     NUM_PROCESSES = 4
#     pool = mp.Pool(NUM_PROCESSES)
#     pool_result = pool.map(hard_load_pickle, fns) 

class FastGaussConvolve(object):
    def __init__(self, rate, filter_beg_delay, filter_cen_delay, sigma_list):
        self.kernels = []
        self.filter_len = int(filter_beg_delay / rate)
        for deg in range(3):
            kernel_sigma_list = []
            for sigma_i, sigma in enumerate(sigma_list):
                inds = np.linspace((filter_cen_delay - filter_beg_delay) / sigma, 
                                   filter_cen_delay / sigma, 
                                   self.filter_len)
                kernel = []
                for v in inds:
                    if deg == 0:
                        kernel.append(self.gauss(v))
                    elif deg == 1:
                        kernel.append(self.gauss_d1(v))
                    elif deg == 2:
                        kernel.append(self.gauss_d2(v))
                kernel = np.array(kernel)
                kernel /= np.sum(np.fabs(kernel))
                kernel_sigma_list.append(kernel)
            self.kernels.append(kernel_sigma_list)
        # print self.kernels
        # plt.figure(1)
        # colors = ['blue', 'red', 'green']
        # for i in range(3):
        #     for j in range(4):
        #         plt.plot(self.kernels[i][j], c=colors[i])
        #         print np.sum(self.kernels[i][j]), len(self.kernels[i][j])
        # plt.show()
        # print assdf
    
    ##
    # Convolves the last values in the signal with the filter
    def convolve_pt(self, signal, deg, sigma_i):
        return np.convolve(signal, 
                           self.kernels[deg][sigma_i], mode='valid')[-1]

    def convolve_signal(self, signal, deg, sigma_i):
        window_len = 100

        avg_sig = sum(signal[0:20]) / 20.
        s=np.r_[[avg_sig]*window_len,signal,[avg_sig]*window_len]

        ret_sig = np.convolve(s, self.kernels[deg][sigma_i], mode='same')

        return list(ret_sig[window_len:-window_len])

    def gauss(self, x, sig=1.):
        return 1./np.sqrt(2. * np.pi * sig ** 2) * np.exp(-x**2/(2*sig**2))

    def gauss_d1(self, x, sig=1.):
        return (-2. * x * 1./(np.sqrt(2. * np.pi * sig ** 2) * 2 * sig**2) * 
                np.exp(-x**2/(2*sig**2)))

    def gauss_d2(self, x, sig=1.):
        return (-2. * 1./(np.sqrt(2. * np.pi * sig ** 2) * 2 * sig**2) * 
                np.exp(-x**2/(2*sig**2)) +
                4. * x**2 * 1./(np.sqrt(2. * np.pi * sig ** 2) * 4 * sig**4) * 
                np.exp(-x**2/(2*sig**2)))

##
# Processes the AccelerometerState message, returning an average of the
# sample values and the timestamp in nanoseconds
#
# @param msg AccelometerState message
# @return (t, (x, y, z))

def accel_state_processor(msg):
    xyz = np.array([0.]*3)
    if msg.samples is None or len(msg.samples) == 0:
        return None
    for samp in msg.samples:
        xyz[0] += samp.x
        xyz[1] += samp.y
        xyz[2] += samp.z
    xyz /= len(msg.samples)
    return (rospy.Time.now().to_sec(), xyz)

r_jt_idx_list = [17, 18, 16, 20, 19, 21, 22]
l_jt_idx_list = [29, 30, 28, 32, 31, 33, 34]
joint_nm_list = ['shoulder_pan', 'shoulder_lift', 'upper_arm_roll',
                 'elbow_flex', 'forearm_roll', 'wrist_flex',
                 'wrist_roll']
##
# Callback for /joint_states topic. Updates current joint
# angles and efforts for the arms constantly
#
# @param data JointState message recieved from the /joint_states topic
def joints_state_processor(msg, right_arm=True, angles_velocities_efforts=0):
    ret = np.array([0.] * 7)
    for i,nm in enumerate(joint_nm_list):
        if right_arm:
            idx = r_jt_idx_list[i]
            if msg.name[idx] != 'r_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', msg.name[idx], i))
            if angles_velocities_efforts == 1:
                ret[i] = msg.velocity[idx]
            elif angles_velocities_efforts == 2:
                ret[i] = msg.effort[idx]
            else:
                ret[i] = msg.position[idx]
        else:

            idx = l_jt_idx_list[i]
            if msg.name[idx] != 'l_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('l_'+nm+'_joint', msg.name[idx], idx))
            if angles_velocities_efforts == 1:
                ret[i] = msg.velocity[idx]
            elif angles_velocities_efforts == 2:
                ret[i] = msg.effort[idx]
            else:
                ret[i] = msg.position[idx]
    return (rospy.Time.now().to_sec(), ret)

def pressure_state_processor(msg, right_finger_tip=True, indicies=None):
    ret = np.array([0.] * len(indicies))
    if indicies is None:
        indicies = range(len(msg.r_finger_tip))
    for i, ind in enumerate(indicies):
        if right_finger_tip:
            ret[i] = msg.r_finger_tip[ind]
        else:
            ret[i] = msg.l_finger_tip[ind]
    return (rospy.Time.now().to_sec(), ret)

def pressure_state_sum_processor(msg, right_finger_tip=True, indicies=None):
    ret = 0.
    if indicies is None:
        indicies = range(len(msg.r_finger_tip))
    for i, ind in enumerate(indicies):
        if right_finger_tip:
            ret += msg.r_finger_tip[ind]
        else:
            ret += msg.l_finger_tip[ind]
    return (rospy.Time.now().to_sec(), np.array([ret]))

##
# Monitors perception channels on the robot arms. Important: rate must be the same for both
# data_capture and monitoring. Values are gathered timestep by timestep.
#
# Usecase:
# apm = ArmPerceptionMonitor(0)
# for trajectory in trajectories:
#     apm.start_data_capture()
#     trajectory.run()
#     trajectory.wait_for_completion()
#     apm.stop_data_capture()
# mean_function, variance_function = apm.generate_model(...)
# 
class ArmPerceptionMonitor( ):

    ##
    # Initializes internal variables
    #
    # @param arm 0 if right, 1 if left
    # @param percept_mon_list list of perceptions to monitor; if None, do all
    def __init__(self, arm):
        self.load_parameters()
        ############## Classifiers #################
        self.impact_classifier = cfiers.classifiers_dict["small_refined_random_forest"]
        self.coll_type_classifier = cfiers.classifiers_dict["large_refined_random_forest"]

        log("Loading impact classifier")
        self.impact_classifier.load(self.I_RTREE_CLASSIFIER)
        log("Loading collision type classifier")
        self.coll_type_classifier.load(self.T_RTREE_CLASSIFIER)

        if arm == 0:
            self.armc = "r"
            self.is_right_arm = True
        else:
            self.armc = "l"
            self.is_right_arm = False

        self.perceptions = {}

        self.perception_names = [ "accelerometer",
                                  "joint_angles",
                                  "joint_velocities",
                                  "joint_efforts",
                                  "r_finger_periph_pressure",
                                  "r_finger_pad_pressure", 
                                  "l_finger_periph_pressure",
                                  "l_finger_pad_pressure",
                                  "gripper_pose"]
        self.perception_lengths = { "accelerometer" : 3,
                                  "joint_angles" : 7,
                                  "joint_velocities" : 7,
                                  "joint_efforts" : 7,
                                  "r_finger_periph_pressure" : 1,
                                  "r_finger_pad_pressure" : 1, 
                                  "l_finger_periph_pressure" : 1,
                                  "l_finger_pad_pressure" : 1,
                                  "gripper_pose" : 7}

        self.percept_mon_list = self.perception_names

        self.impact_fgc = FastGaussConvolve(self.SAMPLING_RATE, self.I_FILTER_BEG_DELAY,
                                     self.I_FILTER_CEN_DELAY, self.i_sigma_list)
        self.type_fgc = FastGaussConvolve(self.SAMPLING_RATE, self.T_FILTER_BEG_DELAY,
                                     self.T_FILTER_CEN_DELAY, self.t_sigma_list)

        self.perception_started = False
        # self.detect_ready = False
        self.fos = FileOperations()

    def load_parameters(self):
        self.SAMPLING_RATE = rospy.get_param("/overhead_grasping/data_sampling_rate")
        self.PERCEPTION_ORDER= rospy.get_param("/overhead_grasping/perception_order")

        # impact parameters
        self.I_FILTER_BEG_DELAY = rospy.get_param("/overhead_grasping/i_filter_beg_delay")
        self.I_FILTER_CEN_DELAY = rospy.get_param("/overhead_grasping/i_filter_cen_delay")
        self.I_MIN_SIGMA = rospy.get_param("/overhead_grasping/i_min_sigma")
        self.I_MAX_SIGMA = rospy.get_param("/overhead_grasping/i_max_sigma")
        self.I_NUM_SIGMA = rospy.get_param("/overhead_grasping/i_num_sigma")
        self.i_sigma_list = np.linspace(self.I_MIN_SIGMA,
                                        self.I_MAX_SIGMA,
                                        self.I_NUM_SIGMA)
        self.I_DEGREE_DICT = rospy.get_param("/overhead_grasping/i_degree_dict")
        self.i_instance_len = 0
        for k in self.PERCEPTION_ORDER:
            for coord in self.I_DEGREE_DICT[k]:
                for deg in coord:
                    self.i_instance_len += self.I_NUM_SIGMA
        self.I_RTREE_CLASSIFIER = rospy.get_param("/overhead_grasping/i_rtree_classifier")

        # collision type parameters
        self.T_FILTER_BEG_DELAY = rospy.get_param("/overhead_grasping/t_filter_beg_delay")
        self.T_FILTER_CEN_DELAY = rospy.get_param("/overhead_grasping/t_filter_cen_delay")
        self.T_MIN_SIGMA = rospy.get_param("/overhead_grasping/t_min_sigma")
        self.T_MAX_SIGMA = rospy.get_param("/overhead_grasping/t_max_sigma")
        self.T_NUM_SIGMA = rospy.get_param("/overhead_grasping/t_num_sigma")
        self.t_sigma_list = np.linspace(self.T_MIN_SIGMA,
                                        self.T_MAX_SIGMA,
                                        self.T_NUM_SIGMA)
        self.T_DEGREE_DICT = rospy.get_param("/overhead_grasping/t_degree_dict")
        self.t_instance_len = 0
        for k in self.PERCEPTION_ORDER:
            for coord in self.T_DEGREE_DICT[k]:
                for deg in coord:
                    self.t_instance_len += self.T_NUM_SIGMA
        self.T_RTREE_CLASSIFIER = rospy.get_param("/overhead_grasping/t_rtree_classifier")

        self.STATIC_SIGMA_INDEX = rospy.get_param("/overhead_grasping/static_sigma_index")
        self.STATIC_DERIV_THRESH = rospy.get_param("/overhead_grasping/static_deriv_thresh")

    ##
    # Initializes the listeners on the sensor topics.  This must
    # be started before any data can be collected from the arms.
    def activate_sensing(self, tf_listener=None):

        log("Initializing arm perception listeners")
        self.tf_listener = tf_listener

        if "accelerometer" in self.percept_mon_list:
            accel_listener = GenericListener("accel_mon_node", AccelerometerState, 
                                     "accelerometer/" + self.armc + "_gripper_motor",
                                     self.SAMPLING_RATE, accel_state_processor)
            self.perceptions["accelerometer"] = accel_listener.read

        if "joint_angles" in self.percept_mon_list:
            joint_angles_processor = ft.partial(joints_state_processor, 
                                                right_arm=self.is_right_arm, 
                                                angles_velocities_efforts=0)
            joint_angles_listener = GenericListener("joint_angles_mon_node", JointState, 
                                               "joint_states", self.SAMPLING_RATE, joint_angles_processor)
            self.perceptions["joint_angles"] = joint_angles_listener.read

        if "joint_velocities" in self.percept_mon_list:
            joint_velocities_processor = ft.partial(joints_state_processor, 
                                                right_arm=self.is_right_arm, 
                                                angles_velocities_efforts=1)
            joint_velocities_listener = GenericListener("joint_vels_mon_node", JointState, 
                                              "joint_states", self.SAMPLING_RATE, joint_velocities_processor)
            self.perceptions["joint_velocities"] = joint_velocities_listener.read

        if "joint_efforts" in self.percept_mon_list:
            joint_efforts_processor = ft.partial(joints_state_processor, 
                                                right_arm=self.is_right_arm, 
                                                angles_velocities_efforts=2)
            joint_efforts_listener = GenericListener("joint_efforts_mon_node", JointState, 
                                              "joint_states", self.SAMPLING_RATE, joint_efforts_processor)
            self.perceptions["joint_efforts"] = joint_efforts_listener.read

        if "r_finger_periph_pressure" in self.percept_mon_list:
            r_finger_periph_pressure_processor = ft.partial(pressure_state_sum_processor, 
                                            right_finger_tip=True, indicies=range(1,7))
            r_finger_periph_pressure_listener = GenericListener(self.armc + "_pressure_r_periph_mon_node", PressureState, 
                                                         "pressure/" + self.armc + "_gripper_motor", self.SAMPLING_RATE, 
                                                         r_finger_periph_pressure_processor)
            self.perceptions["r_finger_periph_pressure"] = r_finger_periph_pressure_listener.read

        if "r_finger_pad_pressure" in self.percept_mon_list:
            r_finger_pad_pressure_processor = ft.partial(pressure_state_sum_processor, 
                                            right_finger_tip=True, indicies=range(7,22))
            r_finger_pad_pressure_listener = GenericListener(self.armc + "_pressure_r_pad_mon_node", PressureState, 
                                                         "pressure/" + self.armc + "_gripper_motor", self.SAMPLING_RATE, 
                                                         r_finger_pad_pressure_processor)
            self.perceptions["r_finger_pad_pressure"] = r_finger_pad_pressure_listener.read

        if "l_finger_periph_pressure" in self.percept_mon_list:
            l_finger_periph_pressure_processor = ft.partial(pressure_state_sum_processor, 
                                            right_finger_tip=False, indicies=range(1,7))
            l_finger_periph_pressure_listener = GenericListener(self.armc + "_pressure_l_periph_mon_node", PressureState, 
                                                         "pressure/" + self.armc + "_gripper_motor", self.SAMPLING_RATE, 
                                                         l_finger_periph_pressure_processor)
            self.perceptions["l_finger_periph_pressure"] = l_finger_periph_pressure_listener.read

        if "l_finger_pad_pressure" in self.percept_mon_list:
            l_finger_pad_pressure_processor = ft.partial(pressure_state_sum_processor, 
                                            right_finger_tip=False, indicies=range(7,22))
            l_finger_pad_pressure_listener = GenericListener(self.armc + "_pressure_l_pad_mon_node", PressureState, 
                                                         "pressure/" + self.armc + "_gripper_motor", self.SAMPLING_RATE, 
                                                         l_finger_pad_pressure_processor)
            self.perceptions["l_finger_pad_pressure"] = l_finger_pad_pressure_listener.read

        for k in self.perceptions:
            # Make sure we have recieved a message first
            self.perceptions[k](allow_duplication=False, willing_to_wait=True)
            self.perceptions[k] = ft.partial(self.perceptions[k], willing_to_wait=False, 
                                             quiet=True,
                                             warn=False, allow_duplication=True)

        if "gripper_pose" in self.percept_mon_list:
            def gripper_pose_lookup():
                if self.tf_listener is not None:
                    lct = self.tf_listener.getLatestCommonTime("/torso_lift_link", "/" + self.armc + "_wrist_roll_link")
                    pos, quat = self.tf_listener.lookupTransform("/torso_lift_link", "/" + self.armc + "_wrist_roll_link", lct)
                    return (lct.to_sec(), np.array(pos + quat))
                else:
                    return (0., np.array([0.]*7))

            self.perceptions["gripper_pose"] = gripper_pose_lookup


        # all callbacks should return data in this format:
        # (t, np.array([x1, x2, ...]))
        # t is time in seconds

        self.clear_vars()
        self.perception_started = True
        
        log("Finished initialization")

    ##
    # Initialize variables
    def clear_vars(self):
        self.datasets = {}
        self.models = {}
        for k in self.perceptions:
            self.datasets[k] = []
            self.models[k] = {"mean" : None, "variance" : None}

    ##
    # Begin capturing peception data for all of the listeners
    #
    # @param duration If None, continue capturing until stop is called.
    #                 Else, stop capturing after duration seconds have passed.
    def start_data_capture(self, duration=None):
        if not self.perception_started:
            err("Perception hasn't been started!")
            return False

        self.logger = RateCaller(self._gather_perception, self.SAMPLING_RATE)

        for k in self.perceptions:
            self.datasets[k] += [[]]
        self.logger.run()

        if not duration is None:
            threading.Timer(self.stop_data_capture, duration)

        return True

    def _gather_perception(self):
        for k in self.perceptions:
            self.datasets[k][-1].append(self.perceptions[k]())

    ##
    # Stop capturing perception data.  Store output data in datasets list for
    # later statistics.
    def stop_data_capture(self):
        self.logger.stop()
        num_datapts = len(self.datasets[self.datasets.keys()[0]][-1])
        log("%d datapoints collected", num_datapts)
        return num_datapts


    ##
    # do processing of signals in current coord group
    # sigs_proc contains a list of three signal sets
    # each corresponds to a list of convolutions of the signals
    # with a degree derivative of a gaussian with various
    # scales
    def process_signals(self, datasets):
        
        models = {}

        sum, num = 0., 0.
        for p in datasets:
            models[p] = {}
            i_sigs_proc = [ [[] for x in range(self.I_NUM_SIGMA)] for y in range(3)]
            t_sigs_proc = [ [[] for x in range(self.T_NUM_SIGMA)] for y in range(3)]
            data_list = datasets[p]
            num_coords = len(data_list[0][0][1])
            times = None
            for coord in range(num_coords):
                stream = data_list[0]
                def conv(v):
                    if type(v) == type([]) or type(v) == type(np.array([])):
                        return v[0]
                    return v
                signal = [conv(v) for v in zip(*zip(*stream)[1])[coord]]

                time_ind = np.array(zip(*stream)[0], dtype=np.float64)

                # normalize time indicies by offsetting to first value
                time_ind_beg = time_ind[0]
                for i, v in enumerate(time_ind):
                    time_ind[i] = float(time_ind[i] - time_ind_beg) 

                times = time_ind
                for deg in range(3):
                    for sigma_ind, sigma in enumerate(self.i_sigma_list):
                        s = self.impact_fgc.convolve_signal(signal, deg, sigma_ind)
                        i_sigs_proc[deg][sigma_ind].append(s)
                    for sigma_ind, sigma in enumerate(self.t_sigma_list):
                        s = self.type_fgc.convolve_signal(signal, deg, sigma_ind)
                        t_sigs_proc[deg][sigma_ind].append(s)

            if rospy.is_shutdown():
                sys.exit()

            models[p]["impact_features"] = [ [[] for x in range(self.I_NUM_SIGMA)] for y in range(3)]
            models[p]["type_features"] = [ [[] for x in range(self.T_NUM_SIGMA)] for y in range(3)]
            for deg in range(3):
                for sigma in range(self.I_NUM_SIGMA):
                    models[p]["impact_features"][deg][sigma] = zip(*i_sigs_proc[deg][sigma])
                for sigma in range(self.T_NUM_SIGMA):
                    models[p]["type_features"][deg][sigma] = zip(*t_sigs_proc[deg][sigma])
            models[p]["time"] = times
            models[p]["i_sigma_list"] = self.i_sigma_list
            models[p]["t_sigma_list"] = self.t_sigma_list

        return models

    # def ready_collision_detection(self):
    #     # wait for the classifiers to finish loading
    #     self.impact_classifier.finish_loading()
    #     # self.coll_type_classifier.finish_loading()
    #     self.detect_ready = True

    ##
    # Begin monitoring peception data to make sure it doesn't deviate from
    # the model provided.
    #
    # TODO DOCS
    # @param duration If None, continue capturing until stop is called.
    #                 Else, stop capturing after duration seconds have passed.
    def begin_collision_detection(self, dynamic_detection=False, 
                                  callback=None, static_collision_type=None,
                                  debug=False):

        if not self.perception_started:
            err("Perception hasn't been started!")
            return False
        # if not self.detect_ready:
        #     err("ready_collision_detection hasn't been called")
        #     return False

        self.is_dynamic = dynamic_detection
        self.callback = callback
        if not dynamic_detection:
            self._setup_collision_type(static_collision_type)
        self.collision_detected = False
        self.collision_finished = False
        self.debug = debug
        self.collision_type = "No collision"
        self.cur_iter = 0
        self.cur_inds = {}

        for k in self.perceptions:
            self.cur_inds[k] = 0

        self.i_data_buffers = {}
        self.t_data_buffers = {}
        for k in self.perceptions:
            self.i_data_buffers[k] = [ np.zeros((self.impact_fgc.filter_len, self.impact_fgc.filter_len)) for i in range(self.perception_lengths[k])]
            self.t_data_buffers[k] = [ np.zeros((self.type_fgc.filter_len, self.type_fgc.filter_len)) for i in range(self.perception_lengths[k])]
        self.end_monitor_lock = threading.Lock()

        self.data_thread_spawner_lock1 = threading.Lock()
        self.data_thread_spawner_lock2 = threading.Lock()
        self.data_thread_spawner = RateCaller(self._data_spawn_thread, self.SAMPLING_RATE ) 
        self.data_thread_spawner.run()

        self.beg_time = rospy.Time.now().to_sec()

        self.t_sum, self.t_num = 0., 0.
        self.normal_dict_counts = []
        self.temp_dict = {}
        for k in self.perceptions:
            self.temp_dict[k] = [[] for i in range(self.perception_lengths[k])]

        return True

    def _setup_collision_type(self, static_collision_type):
        if static_collision_type == "all":
            static_dict = {"accelerometer" : range(3),
                           "joint_angles" : range(7),
                           "joint_velocities" : range(7),
                           "joint_efforts" : range(7),
                           "r_finger_periph_pressure" : range(1),
                           "r_finger_pad_pressure" : range(1), 
                           "l_finger_periph_pressure" : range(1),
                           "l_finger_pad_pressure" : range(1),
                           "gripper_pose" : range(7)}
        elif static_collision_type in self.perception_names:
            static_dict = {static_collision_type : range(
                                    self.perception_lengths[static_collision_type])}
        elif static_collision_type == "pressure":
            static_dict = {"r_finger_periph_pressure" : range(1),
                           "r_finger_pad_pressure" : range(1), 
                           "l_finger_periph_pressure" : range(1),
                           "l_finger_pad_pressure" : range(1)}
        elif type(static_collision_type) == type({}):
            static_dict = static_collision_type
        else:
            err("Bad static_collision_type")
        self.static_dict = static_dict

    def _data_spawn_thread(self):
        # only one thread allowed to wait at a time
        # all other threads will pass through
        if self.data_thread_spawner_lock1.acquire(False):
            if self.data_thread_spawner_lock2.acquire(True):
                self.data_thread_spawner_lock1.acquire(False)
                self.data_thread_spawner_lock1.release()
                mt = self.DataThread(self)
                mt.start()

    class DataThread(threading.Thread):
        def __init__(self, apm):
            threading.Thread.__init__(self)
            self.apm = apm

        def run(self):
            self.apm._monitor_data_collector()
            self.apm.data_thread_spawner_lock2.acquire(False)
            self.apm.data_thread_spawner_lock2.release()

    def _monitor_data_collector(self):
        st_time = rospy.Time.now().to_sec()
        #########################################
        if rospy.is_shutdown():
            self._trigger_collision()
            self._finish_collision("rospy shutdown")
        add_data = []
        for k in self.perceptions:
            p_vals = self.perceptions[k]()[1]
            for i, v in enumerate(p_vals):
                # populate impact collision buffers
                for b in range(self.impact_fgc.filter_len - 1, -1, -1):
                    self.i_data_buffers[k][i][(
                        b - self.cur_iter) % self.impact_fgc.filter_len, b] = v
                    if self.debug:
                        if b == 0 and k == "joint_angles": # and ((i == 0) or (i == 1)):
                            print v,
                # populate collision type detection buffers
                for b in range(self.type_fgc.filter_len - 1, -1, -1):
                    self.t_data_buffers[k][i][(
                        b - self.cur_iter) % self.type_fgc.filter_len, b] = v

        if self.debug:
            print


        if self.is_dynamic:
            if not self.collision_detected:
                self._dynamic_collision_detection()
            else:
                self._dynamic_collision_classifying()
        else:
            self._static_collision_detection()
        self.cur_iter += 1
        #########################################
        end_time = rospy.Time.now().to_sec()
        self.t_sum += end_time - st_time
        self.t_num += 1.

    def _dynamic_collision_detection(self):

        if self.cur_iter < self.impact_fgc.filter_len:
            return
        instance = np.zeros((self.i_instance_len, 1))
        i_place = 0
        for k in self.PERCEPTION_ORDER:
            for coord in range(len(self.i_data_buffers[k])):
                for deg in range(3):
                    if deg not in self.I_DEGREE_DICT[k][coord]:
                        continue
                    for sigma_i, sigma_t in enumerate(self.i_sigma_list):
                        val = self.impact_fgc.convolve_pt(
                                self.i_data_buffers[k][coord][(
                                    self.cur_iter % self.impact_fgc.filter_len)], 
                                    deg, sigma_i)
                        instance[i_place, 0] = val
                        i_place += 1

        has_collided = self.impact_classifier.predict(instance)

        if has_collided and not self.debug:
            if not self.collision_detected:
                self._trigger_collision()


    def _dynamic_collision_classifying(self):

        if self.cur_iter < self.type_fgc.filter_len:
            # TODO HANDLE THIS CASE
            return
        if ((self.cur_iter - self.coll_iter) > self.type_fgc.filter_len / 2):
            return
        instance = np.zeros((self.t_instance_len, 1))
        i_place = 0
        for k in self.PERCEPTION_ORDER:
            for coord in range(len(self.t_data_buffers[k])):
                for deg in range(3):
                    if deg not in self.T_DEGREE_DICT[k][coord]:
                        continue
                    for sigma_i, sigma_t in enumerate(self.t_sigma_list):
                        val = self.type_fgc.convolve_pt(
                                self.t_data_buffers[k][coord][(
                                    self.cur_iter % self.type_fgc.filter_len)], 
                                    deg, sigma_i)
                        instance[i_place, 0] = val
                        i_place += 1

        collision_class = self.coll_type_classifier.predict(instance)

        if collision_class == 1.:
            type = "Environmental collision"
        else:
            type = "Table collision"
        self._finish_collision(type)


    def _static_collision_detection(self):
        if self.cur_iter < self.impact_fgc.filter_len:
            return
        for k in self.PERCEPTION_ORDER:
            if k not in self.static_dict:
                continue
            for coord in self.static_dict[k]:
                if k == "gripper_pose" and coord >= 3:
                    # Quaternions are unreliable, this should be fixed
                    # at some point
                    if self.debug:
                        self.temp_dict[k][coord].append(0.)
                    continue
                val = self.impact_fgc.convolve_pt(self.i_data_buffers[k][coord][self.cur_iter % self.impact_fgc.filter_len], 1, self.STATIC_SIGMA_INDEX)
                if self.debug:
                    self.temp_dict[k][coord].append(val)
                if (np.fabs(val) >= self.STATIC_DERIV_THRESH[k][coord] 
                    and not self.debug):
                    print "Collision:", val, "Thresh:", self.STATIC_DERIV_THRESH[k][coord]
                    self._trigger_collision()
                    self._finish_collision(k + " collision", coord)
                # if k == "r_finger_pad_pressure" and random.randint(0,40) == 0:
                #     print val


    def _trigger_collision(self):
        self.collision_detected = True
        self.coll_iter = self.cur_iter - self.impact_fgc.filter_len / 2
        if not self.callback is None:
            self.callback()

    def _finish_collision(self, type = "Unlabled", coord = 0):
        if not self.collision_finished:
            print '-' * 60
            print
            print '                       %s detected' % type
            print
            print '-' * 60
            log("%s reported" % type)
            self.collision_type = type
            self.collision_coord = coord
            if not self.callback is None:
                self.callback()
            self.collision_finished = True
            self.end_collision_detection()
            
    ##
    # Stop capturing perception data.  Store output data in datasets list for
    # later statistics.
    def end_collision_detection(self):
        while (not self.collision_finished and self.collision_detected 
                and not rospy.is_shutdown()):
            rospy.sleep(0.01)
        with self.end_monitor_lock:
            if not self.data_thread_spawner is None:
                print "Avg cvpt: %1.9f" % (self.t_sum / self.t_num)
                print "cvpt sum: %1.5f" % (self.t_sum)
                print self.t_num
                self.data_thread_spawner.stop()
                self.data_thread_spawner_lock1.acquire(False)
                self.data_thread_spawner_lock1.release()
                self.data_thread_spawner_lock2.acquire(False)
                self.data_thread_spawner_lock2.release()
                self.data_thread_spawner = None
            return

def main(args):

    if args[1] == "static_test":
        log("Test 2")
        apm = ArmPerceptionMonitor(0, active = True,
                                   tf_listener = None)
        rospy.sleep(1.)
        apm.begin_collision_detection(dynamic_detection=False, static_collision_type="all")
        while not apm.collision_detected:
            rospy.sleep(0.01)

        log("Collision detected, exiting...")

        return 

    if args[1] == "test1":
        log("Test 1")
        apm = ArmPerceptionMonitor(0, active = True,
                                   tf_listener = None)
        apm.begin_collision_detection()
        rospy.sleep(8.)
        apm.end_collision_detection()
        return

    if args[1] == "test2":
        log("Test 2")
        tf_listener = tf.TransformListener()
        apm = ArmPerceptionMonitor(1, active=True,
                                   tf_listener=tf_listener)
        rospy.sleep(1.)
        apm.begin_collision_detection(dynamic_detection=False, static_collision_type="pressure",
                             debug=True)
        while not apm.collision_detected:
            rospy.sleep(0.01)
        # rospy.sleep(20.)
        apm.end_collision_detection()
        means = {}
        std_devs = {}
        for k in apm.temp_dict:
            means[k] = []
            std_devs[k] = []
            for c_i, coord in enumerate(apm.temp_dict[k]):
                print "Perception %s, coord %d" % (k, c_i)
                log("min: %1.3f, max: %1.3f, median: %1.3f, mean: %1.3f, std: %1.3f" % (np.min(coord), np.max(coord), np.median(coord), np.mean(np.fabs(coord)), np.std(coord)))
                means[k].append(np.mean(coord))
                std_devs[k].append(4 * np.std(coord))
        # print apm.temp_dict
        print means
        print std_devs
        return

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)
    sys.exit(main(sys.argv))
