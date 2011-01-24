#! /usr/bin/python

import numpy as np, math
import scipy.stats as stats
from threading import RLock

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

from hrl_lib.util import save_pickle, load_pickle
from hrl_lib.msg import FloatArray
from hrl_lib.rutils import GenericListener, ros_to_dict, RateCaller
from hrl_lib.data_process import signal_smooth, signal_variance, signal_list_variance
from tf.transformations import *
from helpers import log, err, node_name

from std_msgs.msg import Float64MultiArray
from pr2_msgs.msg import AccelerometerState, PressureState
from sensor_msgs.msg import JointState

import threading
import functools as ft
import Queue

import time, string

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
    xyz[0] /= len(msg.samples)
    xyz[1] /= len(msg.samples)
    xyz[2] /= len(msg.samples)
    return (msg.header.stamp.to_nsec(), xyz)

r_jt_idx_list = [17, 18, 16, 20, 19, 21, 22]
l_jt_idx_list = [31, 32, 30, 34, 33, 35, 36]
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
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', msg.name[idx], i))
            if angles_velocities_efforts == 1:
                ret[i] = msg.velocity[idx]
            elif angles_velocities_efforts == 2:
                ret[i] = msg.effort[idx]
            else:
                ret[i] = msg.position[idx]
    return (msg.header.stamp.to_nsec(), ret)

def pressure_state_processor(msg, right_finger_tip=True, indicies=None):
    ret = np.array([0.] * len(indicies))
    if indicies is None:
        indicies = range(len(msg.r_finger_tip))
    for i, ind in enumerate(indicies):
        if right_finger_tip:
            ret[i] = msg.r_finger_tip[ind]
        else:
            ret[i] = msg.l_finger_tip[ind]
    return (msg.header.stamp.to_nsec(), ret)


##
# Monitors perception channels on the robot arms. Important: rate must be the same for both
# training and monitoring. Values are gathered timestep by timestep.
#
# Usecase:
# apm = ArmPerceptionMonitor(0)
# for trajectory in trajectories:
#     apm.start_training()
#     trajectory.run()
#     trajectory.wait_for_completion()
#     apm.stop_training()
# mean_function, variance_function = apm.generate_model(...)
# 
class ArmPerceptionMonitor( ):

    ##
    # Initializes the listeners on the perception topics
    #
    # @param arm 0 if right, 1 if left
    # @param rate the rate at which the perception should capture states
    # @param percept_mon_list list of perceptions to monitor; if None, do all
    def __init__(self, arm, tf_listener=None, rate=0.001, 
                 percept_mon_list=None, model_zeros=None):
        log("Initializing arm perception listeners")

        self.rate = rate

        if arm == 0:
            armc = "r"
            is_right_arm = True
        else:
            armc = "l"
            is_right_arm = False

        self.model_zeros = model_zeros

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

        if percept_mon_list is None:
            percept_mon_list = self.perception_names

        if "accelerometer" in percept_mon_list:
            accel_listener = GenericListener("accel_mon_node", AccelerometerState, 
                                     "accelerometer/" + armc + "_gripper_motor",
                                     rate, accel_state_processor)
            self.perceptions["accelerometer"] = accel_listener.read

        if "joint_angles" in percept_mon_list:
            joint_angles_processor = ft.partial(joints_state_processor, 
                                                right_arm=is_right_arm, 
                                                angles_velocities_efforts=0)
            joint_angles_listener = GenericListener("joint_angles_mon_node", JointState, 
                                               "joint_states", rate, joint_angles_processor)
            self.perceptions["joint_angles"] = joint_angles_listener.read

        if "joint_velocities" in percept_mon_list:
            joint_velocities_processor = ft.partial(joints_state_processor, 
                                                right_arm=is_right_arm, 
                                                angles_velocities_efforts=1)
            joint_velocities_listener = GenericListener("joint_efforts_mon_node", JointState, 
                                              "joint_states", rate, joint_velocities_processor)
            self.perceptions["joint_velocities"] = joint_velocities_listener.read

        if "joint_efforts" in percept_mon_list:
            joint_efforts_processor = ft.partial(joints_state_processor, 
                                                right_arm=is_right_arm, 
                                                angles_velocities_efforts=2)
            joint_efforts_listener = GenericListener("joint_efforts_mon_node", JointState, 
                                              "joint_states", rate, joint_efforts_processor)
            self.perceptions["joint_efforts"] = joint_efforts_listener.read

        if "r_finger_periph_pressure" in percept_mon_list:
            r_finger_periph_pressure_processor = ft.partial(pressure_state_processor, 
                                            right_finger_tip=True, indicies=range(1,7))
            r_finger_periph_pressure_listener = GenericListener("pressure_mon_node", PressureState, 
                                                         "pressure/" + armc + "_gripper_motor", rate, 
                                                         r_finger_periph_pressure_processor)
            self.perceptions["r_finger_periph_pressure"] = r_finger_periph_pressure_listener.read

        if "r_finger_pad_pressure" in percept_mon_list:
            r_finger_pad_pressure_processor = ft.partial(pressure_state_processor, 
                                            right_finger_tip=True, indicies=range(7,22))
            r_finger_pad_pressure_listener = GenericListener("pressure_mon_node", PressureState, 
                                                         "pressure/" + armc + "_gripper_motor", rate, 
                                                         r_finger_pad_pressure_processor)
            self.perceptions["r_finger_pad_pressure"] = r_finger_pad_pressure_listener.read

        if "l_finger_periph_pressure" in percept_mon_list:
            l_finger_periph_pressure_processor = ft.partial(pressure_state_processor, 
                                            right_finger_tip=False, indicies=range(1,7))
            l_finger_periph_pressure_listener = GenericListener("pressure_mon_node", PressureState, 
                                                         "pressure/" + armc + "_gripper_motor", rate, 
                                                         l_finger_periph_pressure_processor)
            self.perceptions["l_finger_periph_pressure"] = l_finger_periph_pressure_listener.read

        if "l_finger_pad_pressure" in percept_mon_list:
            l_finger_pad_pressure_processor = ft.partial(pressure_state_processor, 
                                            right_finger_tip=False, indicies=range(7,22))
            l_finger_pad_pressure_listener = GenericListener("pressure_mon_node", PressureState, 
                                                         "pressure/" + armc + "_gripper_motor", rate, 
                                                         l_finger_pad_pressure_processor)
            self.perceptions["l_finger_pad_pressure"] = l_finger_pad_pressure_listener.read

        for k in self.perceptions:
            self.perceptions[k] = ft.partial(self.perceptions[k], willing_to_wait=False, 
                                             quiet=True,
                                             warn=False, allow_duplication=True)

        if "gripper_pose" in percept_mon_list:
            def gripper_pose_lookup():
                lct = tf_listener.getLatestCommonTime("/torso_lift_link", "/" + armc + "_wrist_roll_link")
                pos, quat = tf_listener.lookupTransform("/torso_lift_link", "/" + armc + "_wrist_roll_link", lct)
                return (lct.to_nsec(), np.array(pos + quat))

            self.perceptions["gripper_pose"] = gripper_pose_lookup


        # all callbacks should return data in this format:
        # (t, np.array([x1, x2, ...]))
        # t is time in nanoseconds



        self.clear_vars()

        self.logger = RateCaller(self._gather_perception, self.rate)
        
        log("Finished initialization")

    def _gather_perception(self):
        t1 = rospy.Time.now().to_sec()
        for k in self.perceptions:
            self.datasets[k][-1] += [self.perceptions[k]()]
        t2 = rospy.Time.now().to_sec()
        import random
        if random.randint(0,100)==0:
            log("Time:", t1-t2)

    ##
    # Initialize variables
    def clear_vars(self):
        self.datasets = {}
        self.models = {}
        for k in self.perceptions:
            self.datasets[k] = []
            self.models[k] = {"mean" : None, "variance" : None}

        self.active = False

    ##
    # Begin capturing peception data for all of the listeners
    #
    # @param duration If None, continue capturing until stop is called.
    #                 Else, stop capturing after duration seconds have passed.
    def start_training(self, duration=None):
        if self.active:
            log("Perception already active.")
            return
        self.active = True

        for k in self.perceptions:
            self.datasets[k] += [[]]
        self.logger.run()

        if not duration is None:
            threading.Timer(self.stop_training, duration)

    ##
    # Stop capturing perception data.  Store output data in datasets list for
    # later statistics.
    def stop_training(self):
        if not self.active:
            log("Nothing to stop.")
            return

        self.logger.stop()

        self.active = False
        return len(self.datasets[self.datasets.keys()[0]][-1])

    ##
    # Save training data as a pickle with given filename
    #
    # @param filename name of the pickle
    def save(self, filename):
        save_pickle((self.datasets, self.models), filename)

    ##
    # Load training data as a pickle with given filename
    #
    # @param filename name of the pickle
    def load(self, filename):
        self.datasets, self.models = load_pickle(filename)

    def get_zeros(self, time=4.):
        self._zeros = {}
        for k in self.perceptions:
            self._zeros[k] = None
        self._n = 0
            
        monitor = RateCaller(self._sum_values, self.rate)
        monitor.run()
        rospy.sleep(time)
        monitor.stop()

        for k in self.perceptions:
            self._zeros[k] /= self._n
        return self._zeros

    
    ##
    # Summer used in finding zeros.
    def _sum_values(self):
        for k in self.perceptions:
            add_data = np.array(self.perceptions[k]()[1])
            if self._zeros[k] is None:
                self._zeros[k] = np.copy(add_data)
            else:
                self._zeros[k] += add_data
        self._n += 1

    ##
    # Sets up monitoring parameters
    def setup_monitoring(self, models,
                               std_dev_dict=None, noise_dev_dict=None, duration=None,
                               std_dev_default=2.0, noise_dev_default=0.25,
                               tol_thresh_dict=None,
                               contingency=None, window_size=70, current_zeros=None,
                               sampling_rate = 1,
                               transform_dict=None, verbose=True, collide=True):
        self.std_dev_default = std_dev_default
        self.noise_dev_default = noise_dev_default

        self.models = models
        self.current_data = {}
        self.std_dev_dict = std_dev_dict
        self.noise_dev_dict = {}
        self.contingency = contingency
        self.window_size = window_size
        self.current_zeros = {}
        self.transform_dict = transform_dict
        self.tol_thresh_dict = tol_thresh_dict
        self.sampling_rate = sampling_rate
        self.verbose = verbose
        self.collide = collide
        self.sum_data = {}
        self.cur_pt = 0
        self.failure = False
        self.dur_timer = None
        self.avg_list = {}
        self.cum_avg = {}
        self.c = {}
        self.locks = {}

        for k in self.perceptions:
            self.current_data[k] = [None] * window_size
            self.sum_data[k] = None
            self.avg_list[k] = []
            self.c[k] = None
            self.locks[k] = threading.Lock()

        self.create_max_min()


    ##
    # Creates perception maximum and minimum thresholds. Only used in visualization.
    def create_max_min(self):
        for k in self.models:
            deviation = np.array(self.models[k]["variance"])
            # noise_deviation = np.sqrt(self.models[k]["noise_variance"])
            if self.std_dev_dict is not None and k in self.std_dev_dict:
                std_dev = np.array(self.std_dev_dict[k])
            else:
                std_dev = self.std_dev_default
            # if self.noise_dev_dict is not None and k in self.noise_dev_dict:
            #     noise_dev = np.array(self.noise_dev_dict[k])
            # else:
            #     noise_dev = self.noise_dev_default
            if self.tol_thresh_dict is not None and k in self.tol_thresh_dict:
                tol_thresh = np.array(self.tol_thresh_dict[k])
            else:
                tol_thresh = 0.

            import random
            if random.randint(0, 10) == 0 and True:
                log(deviation[0:20], len(deviation))
            self.models[k]["dev"] = (std_dev * deviation + tol_thresh)
            # self.models[k]["dev"] = (std_dev * deviation + noise_dev * noise_deviation + tol_thresh)


            # This is the monitoring equation
            self.models[k]["max"] = self.models[k]["mean"] + self.models[k]["dev"]
            self.models[k]["min"] = self.models[k]["mean"] - self.models[k]["dev"]

    ##
    # Begin monitoring peception data to make sure it doesn't deviate from
    # the model provided.
    #
    # TODO DOCS
    # @param duration If None, continue capturing until stop is called.
    #                 Else, stop capturing after duration seconds have passed.
    def begin_monitoring(self, models=None, model_zeros=None,
                               std_dev_dict=None, noise_dev_dict=None, duration=None,
                               std_dev_default=2.0, noise_dev_default=0.25,
                               tol_thresh_dict=None,
                               contingency=None, window_size=70, current_zeros=None,
                               sampling_rate = 1,
                               only_pressure=False,
                               transform_dict=None, verbose=True, collide=True):
        if self.active:
            log("Perception already active.")
            return
        self.active = True


        self.setup_monitoring(models, 
                               std_dev_dict=std_dev_dict, noise_dev_dict=noise_dev_dict,
                               duration=duration, std_dev_default=std_dev_default, 
                               noise_dev_default=noise_dev_default,                         
                               tol_thresh_dict=tol_thresh_dict,
                               contingency=contingency, window_size=window_size, 
                               sampling_rate=2.*sampling_rate,
                               current_zeros=current_zeros, transform_dict=transform_dict, 
                               verbose=verbose, collide=collide)
        # if models is not None:
        #     self.models = models
        if model_zeros is not None:
            self.model_zeros = model_zeros
        self.cur_pt = 0
        self.collision_times = {}
        self.collision_sums = {}
        self.cur_col_time = 0
        self.current_zeros = {}
        self.z_sum = {}
        self.z_rsum = {}
        self.z_list = {}
        self.min_prob = 100000.0
        self.cur_mals = {}
        self.mals = None
        self.only_pressure = only_pressure

        self.monitor = RateCaller(self._monitor_data, self.rate * self.sampling_rate)
        self.monitor.run()
        # self.sumcalls = {}
        # self.sumsave = {}

        if not duration is None:
            self.dur_timer = threading.Timer(self.end_monitoring, duration)
            self.dur_timer.start()

    ##
    # Stable Kahan summation algorithm used to keep a running sum of recent percept
    # data.
    def _stable_summer(self, percept, data):
        # kahanSum
        k = percept
        # if percept not in self.sumcalls:
        #     self.sumcalls[percept] = 1
        #     self.sumsave[percept] = []
        # else:
        #     self.sumcalls[percept] += 1

        if self.c[k] is None:
            self.c[k] = np.array([0.]*len(data))
        if self.sum_data[k] is None:
            self.sum_data[k] = np.copy(data)
        else:
            y = data - self.c[k]
            t = self.sum_data[k] + y
            self.c[k] = (t - self.sum_data[k]) - y
            self.sum_data[k] = t
            # self.sumsave[percept] += [self.sum_data[k]]


    ##
    # Called periodically to check for collisions.
    def _monitor_data(self):
        avg_dict = {}
        sttime = rospy.Time.now().to_sec()
        for k in self.perceptions:
            # update the running sum
            add_data = self.perceptions[k]()[1]

            # apply necessary transforms
            # if self.transform_dict is not None:
            #     if k in self.transform_dict:
            #         add_data = self.transform_dict[k](add_data)
                    
            self._stable_summer(k, add_data)

            # If we have enough data to monitor, we check to see if the values are in range
            if self.cur_pt >= self.window_size: #self.current_data[k].full():
                # self.prev_sum = copy.deepcopy(self.sum_data)
                avg_dict[k] = self.sum_data[k] / self.window_size
                if not k in self.current_zeros:
                    if not self.only_pressure:
                        mean = np.array(self.models[k]["mean"][self.cur_pt])
                    else:
                        mean = np.array(self.models[k]["mean"][0])
                    self.current_zeros[k] = mean - avg_dict[k]
                    # self.current_zeros[k] = - avg_dict[k]

                avg_dict[k] += self.current_zeros[k]
                self.avg_list[k] += [avg_dict[k]] * 2 # * self.sampling_rate
                # if self.cur_pt == self.window_size:
                #     log(self.avg_list[k])

                # self.prev_avg = copy.deepcopy(avg)
                # offset zeros into original perception frame
                # if self.current_zeros is not None:
                #     if False and self.model_zeros is not None:
                #         avg_dict[k] += self.model_zeros[k] - self.current_zeros[k]
                #     else:
                #         # this is hacky, need to use zeros during training instead of first pt
                #         # log("NOOOOOOOOOOOO!\n"*10)
                #         avg_dict[k] +=  np.array(self.models[k]["mean"][self.window_size]) - self.current_zeros[k]

                # if collision_detected:
                #     log("Value %d of the perception %s failed with difference %f"
                #                                                % (index, k, diff))
                #   # if diff > 5. and k == "accelerometer":
                #   #     log("avg_list", self.avg_list)
                #   #     log("avg", avg)
                #   #     for i in range(self.window_size+1):
                #   #         d = self.current_data[k].get()
                #   #         log(d)
                #   #         self.current_data[k].put(d)


                #   self.failure = True
                #   self.contingency()
                #   self.monitor.stop()

                rem_data = self.current_data[k][self.cur_pt % self.window_size]
                self._stable_summer(k, -rem_data)

            self.current_data[k][self.cur_pt % self.window_size] = add_data

        if self.cur_pt >= self.window_size: #self.current_data[k].full():
            collision_detected = self.collision_detect(avg_dict)
            if self.collide and collision_detected:
                log("collision reported")
                self.failure = True
                if not self.contingency is None:
                    self.contingency()
                self.monitor.stop()
                return


        self.cur_pt += 2

        if not self.only_pressure:
            if self.cur_pt >= 1.00 * len(self.models[self.models.keys()[0]]["mean"]):
                log("ending early:", self.cur_pt * self.sampling_rate)
                self.end_monitoring()
        endtime = rospy.Time.now().to_sec()
        if self.cur_pt % 10 == 0:
            log("dur:", sttime - endtime)



    ##
    # Stop capturing perception data.  Store output data in datasets list for
    # later statistics.
    def end_monitoring(self):
        if not self.active:
            log("Nothing to stop.")
            return self.avg_list 

        log("-------------------------")
        log("z averages:")
        self.z_avg = {}
        for k in self.z_sum:
            self.z_avg[k] = self.z_sum[k] / self.cur_pt
            log(k, ":", self.z_avg[k])
        log("-------------------------")
        log("MINIMUM PROB")
        log(self.min_prob)
        log("-------------------------")
        # log("Sum calls", self.sumcalls)
        # if self.sumcalls["r_finger_periph_pressure"] == 202:
        #     log(self.sumsave)
        self.monitor.stop()
        if self.dur_timer is not None:
            self.dur_timer.cancel()
            self.dur_timer = None

        self.active = False
        return self.avg_list

    ##
    # Uses current average perceptions to determine if a collision has occured.
    # TODO PICK FILTER AND DELETE OTHER
    def collision_detect(self, avg_dict):
        # is_outside_range_dict = {}
        z = {}
        mal_sum = 0.
        for k in avg_dict:

            # This is the monitoring equation
            # is_outside_range_dict[k] = ((avg_dict[k] > self.models[k]["max"][self.cur_pt * self.sampling_rate]) +
            #                             (avg_dict[k] < self.models[k]["min"][self.cur_pt * self.sampling_rate]))
            # Uses both variance from various grasp tries and general noise variance
            if not self.only_pressure:
                mean = self.models[k]["mean"][self.cur_pt]
                dev = self.models[k]["dev"][self.cur_pt]
            else:
                mean = self.models[k]["mean"][0]
                dev = self.models[k]["dev"][0]

            cur_mal = ((avg_dict[k] - mean) / dev) ** 2
            self.cur_mals[k] = cur_mal
            mal_sum += np.add.reduce( ( (avg_dict[k] - mean) / dev) ** 2 )
            z[k] = np.fabs(avg_dict[k] - mean) / dev 

            if not k in self.z_sum:
                self.z_sum[k] = np.copy(z[k])
            else:
                self.z_sum[k] += z[k]

        # log("incoldet")
        # collision_detected = self.collision_filtering(is_outside_range_dict)

        # collision_detected = self.collision_filtering_mal(np.sqrt(mal_sum))
        # return collision_detected

        collision_detected = self.collision_filtering(z)

        # log("coldete", collision_detected)
        return collision_detected

    ##
    # Collision detection using Malhalonobis distance measure
    def collision_filtering_mal(self, mal):
        prob = 1.
        TIME_WINDOW = 4
        if self.mals is None:
            self.mals = np.array([0.] * TIME_WINDOW)
        self.mals[self.cur_pt % TIME_WINDOW] = mal

        if self.cur_pt < TIME_WINDOW:
            return False
        mal_avg = np.add.reduce(self.mals / TIME_WINDOW)

        if self.cur_pt % 10 == 0:
            log(mal_avg)
        MAL_THRESH = 4.7
        if mal_avg > MAL_THRESH:
            log("Collision with mal dist:", mal_avg)
            for k in self.cur_mals:
                log(k, self.cur_mals[k])
            return True
        return False

    ##
    # Collision detection using joint probability z-test
    def collision_filtering(self, z):
        # TODO FIX EXTERNAL VARS
        prob = 1.
        TIME_WINDOW = 8
        for k in z:
            if not k in self.z_rsum:
                self.z_rsum[k] = np.copy(z[k])
                self.z_list[k] = [np.array([0.]*len(z[k]))] * TIME_WINDOW
            else:
                self.z_rsum[k] += z[k]
            
            self.z_rsum[k] -= self.z_list[k][self.cur_pt % TIME_WINDOW]
            self.z_list[k][self.cur_pt % TIME_WINDOW] = z[k]

            # log(z[k])
            prob *= np.multiply.reduce(2 * stats.norm.cdf(-self.z_rsum[k] / TIME_WINDOW))
            # log(prob)

        if self.cur_pt < TIME_WINDOW:
            return False

        if prob < self.min_prob:
            self.min_prob = prob

        # CONFIDENCE = 1e-6
        if not self.only_pressure:
            # 4e-7 50/50
            CONFIDENCE = 2e-4 # 2.2e-4 # 1.9e-4
        else:
            # pressure only confidence
            CONFIDENCE = 1.e-1
        if self.cur_pt % 20 == 0:
            log("Current pt:", self.cur_pt, ", probability:", prob)
        if prob < CONFIDENCE:
            log("probability:", prob)
            log("z values")
            for k in z:
                log(k, z[k])
            log("collision returned")
            return True
    #       log("-------------------------")
    #       log(num_percep_col)
    #       for k in sig_counts:
    #           if sig_counts[k] >= PERCEPT_SIG_REQ[k]:
    #               log(k, ":", sig_counts[k])
    #       log("-------------------------")
    #       return True

        return False

    # @TODO add sampling_rate stuff
    def simulate_monitoring(self, monitor_data, models=None, model_zeros=None,
                               std_dev_dict=None, noise_dev_dict=None, 
                               duration=None,
                               std_dev_default=2.0, noise_dev_default=0.25,
                               tol_thresh_dict=None,
                               contingency=None, window_size=70, current_zeros=None,
                               transform_dict=None, verbose=True, collide=True):
        self.setup_monitoring(std_dev_dict=std_dev_dict, noise_dev_dict=noise_dev_dict,
                               duration=duration, std_dev_default=std_dev_default, 
                               tol_thresh_dict=tol_thresh_dict,
                               noise_dev_default=noise_dev_default,                                                         contingency=contingency, window_size=window_size, 
                               current_zeros=current_zeros, transform_dict=transform_dict, 
                               verbose=verbose, collide=collide)
        if models is not None:
            self.models = models
        if model_zeros is not None:
            self.model_zeros = model_zeros
        self.cur_pt = self.window_size
        collision = None
        # i is the number of samples in the monitor data
        for i in range(len(monitor_data[monitor_data.keys()[0]])):
            for k in monitor_data:
                index, diff = self.collision_detect(k, monitor_data[k][i])
                if index != -1:
                    collision = (k, index, diff)
            self.cur_pt += 1
        return collision

    # Reduces the amount of data in the monitor_data dataset by taking only every
    # sample_rate samples.
    def resample_and_thin_data(self, monitor_data, sample_rate):
        ret_data = {}
        for k in monitor_data:
            ret_data[k] = {}
            ret_data[k]["mean"] = monitor_data[k]["mean"][::sample_rate]
            ret_data[k]["variance"] = monitor_data[k]["variance"][::sample_rate]
        return ret_data

##
# Takes data, splits it, finds the variance, and returns the packaged data.
def split_signals(datasets):

    for perception in datasets:
        data_list = datasets[perception]

        # get the minimum model length
        lens = [len(m) for m in data_list]
        max_len = np.max(lens)

        # dynamic finding of parameters
        if smooth_wind_dict is None or smooth_wind_dict[perception] is None:
            smooth_wind = smooth_wind_default
        else:
            smooth_wind = smooth_wind_dict[perception]

        ret_means, ret_vars, ret_mean_models, ret_noise_vars = [], [], [], []
        ret_times, noise_vars = [], []
        # find the number of coordinates from the first element
        num_coords = len(data_list[0][0][1])
        for coord in range(num_coords):
            mean_models, variance_models = [], []
            times = None
            for stream in data_list:
                # extract only the data stream for a single coordinate (all x values)
                stream_coord = np.array(zip(*zip(*stream)[1])[coord])
                cur_mean_model = signal_smooth(stream_coord, smooth_wind)
                mean_models += [cur_mean_model]
            
                # sum up the squared difference over the whole model
                noise_vars += [ ( sum([(x - y) ** 2 for x,y in zip(cur_mean_model,stream_coord)]) /
                                                 len(cur_mean_model) ) ]

            # find the average case over the several runs
            avg_means_model = np.array([0.] * max_len)
            for i in range(max_len):
                n = 0
                for j in range(len(mean_models)):
                    if i < len(mean_models[j]):
                        avg_means_model[i] += mean_models[j][i]
                        n += 1
                avg_means_model[i] /= n

            if var_wind_dict is None or var_wind_dict[perception] is None:
                var_wind = var_wind_default
            else:
                var_wind = var_wind_dict[perception]
            # find the variance of the signal but use var_wind points around the centers
            # to increase the sample size
            vars_model = signal_list_variance(mean_models, avg_means_model, var_wind)
            vars_model = signal_smooth(vars_model, var_smooth_wind)
            vars_model = signal_smooth(vars_model, var_smooth_wind + 23)

            ret_times += [times]
            ret_means += [avg_means_model]
            ret_vars += [vars_model]
            ret_mean_models += [mean_models]
            ret_noise_vars += [np.average(noise_vars)]

        # TODO deal with timestamp data in some way?
        # self.models[perception]["time"] = ret_times
        self.models[perception]["mean"] = np.array(zip(*ret_means))
        self.models[perception]["variance"] = np.array(zip(*ret_vars))
        a = ret_mean_models
        b = []
        for stream in range(len(a[0])):
            t1 = []
            for val in range(len(a[0][0])):
                    t2 = []
                    for coord in range(len(a)):
                            if val < len(a[coord][stream]):
                                t2 += [a[coord][stream][val]]
                    t1 += [np.array(t2)]
            b += [t1]

        self.models[perception]["smoothed_signals"] = b
        self.models[perception]["noise_variance"] = np.array(ret_noise_vars)

    return self.models

##
# Takes a data structure representing the perception signals of a grasp configuration
# over several runs and finds the average signal.  The resulting signals created
# will be as long as the longest input data stream.
def generate_mean_grasp(datasets):

    smooth_wind_default = 80
    means = {}
    for perception in datasets:
        data_list = datasets[perception]

        # get the minimum model length
        lens = [len(m) for m in data_list]
        max_len = np.max(lens)

        ret_means = []
        # find the number of coordinates from the first element
        num_coords = len(data_list[0][0][1])
        for coord in range(num_coords):
            mean_models, variance_models = [], []
            for stream in data_list:
                # extract only the data stream for a single coordinate (all x values)
                stream_coord = np.array(zip(*zip(*stream)[1])[coord])
                cur_mean_model = signal_smooth(stream_coord, smooth_wind_default)
                mean_models += [cur_mean_model]

            # find the average case over the several runs
            avg_means_model = np.array([0.] * max_len)
            for i in range(max_len):
                n = 0
                for j in range(len(mean_models)):
                    if i < len(mean_models[j]):
                        avg_means_model[i] += mean_models[j][i]
                        n += 1
                avg_means_model[i] /= n

            ret_means += [avg_means_model]

        means[perception] = np.array(zip(*ret_means))

    return means
    
# TODO Remove or fix
if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)

    apm = ArmPerceptionMonitor(0, 0.001)
    for x in range(3):
        raw_input("Begin arm")
        apm.start_training()
        rospy.sleep(1.)
        apm.stop_training()
        rospy.sleep(0.1)
    models = apm.generate_models()
    apm.pickle_datasets("pickles//apmtest.pickle")
    means = models["accelerometer"]["mean"]
    vars = models["accelerometer"]["variance"]
    
    xm, ym, zm = zip(*means)
    xv, yv, zv = zip(*vars)
    xv = map(np.sqrt, xv)
    yv = map(np.sqrt, yv)
    zv = map(np.sqrt, zv)
    std_dev = 2.5
    xmmax = [m + np.sqrt(v) * std_dev for m, v in zip(xm, xv)]
    ymmax = [m + np.sqrt(v) * std_dev for m, v in zip(ym, yv)]
    zmmax = [m + np.sqrt(v) * std_dev for m, v in zip(zm, zv)]
    xmmin = [m - np.sqrt(v) * std_dev for m, v in zip(xm, xv)]
    ymmin = [m - np.sqrt(v) * std_dev for m, v in zip(ym, yv)]
    zmmin = [m - np.sqrt(v) * std_dev for m, v in zip(zm, zv)]
    
    import matplotlib.pyplot as plt
    plt.subplot(321)
    plt.plot(xm)
    plt.plot(xmmax)
    plt.plot(xmmin)
    #plt.axis([0, len(xm), 0., 15.])
    plt.title("X mean")
    plt.subplot(323)
    plt.plot(ym)
    plt.plot(ymmax)
    plt.plot(ymmin)
    plt.title("Y mean")
    plt.subplot(325)
    plt.plot(zm)
    plt.plot(zmmax)
    plt.plot(zmmin)
    plt.title("Z mean")

    plt.subplot(322)
    plt.plot(xv)
    plt.title("X variance")
    plt.subplot(324)
    plt.plot(yv)
    plt.title("Y variance")
    plt.subplot(326)
    plt.plot(zv)
    plt.title("Z variance")

    plt.show()
