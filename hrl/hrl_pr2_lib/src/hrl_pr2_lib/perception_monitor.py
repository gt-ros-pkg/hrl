#! /usr/bin/python

import numpy as np, math
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

from hrl_lib.rutils import GenericListener, ros_to_dict
from hrl_lib.data_process import signal_smooth, signal_variance

from pr2_msgs.msg import AccelerometerState

import threading

import time, string

node_name = "arm_perception_monitor" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

def accel_state_processor(msg):
    accel_msg = ros_to_dict(msg)
    x, y, z = 0., 0., 0.
    if msg.samples is None or len(msg.samples) == 0:
        return None
    for samp in msg.samples:
        x += samp.x
        y += samp.y
        z += samp.z
    x /= len(msg.samples)
    y /= len(msg.samples)
    z /= len(msg.samples)
    return (msg.header.stamp.to_nsec(), (x, y, z))

class PeriodicMonitor():
    def __init__(self, callback, rate=0.01, args=None):
        self.ret = []
        self.cb = callback
        self.rate = rate
        self.args = args
        self.is_running = False

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self._run()

    def _run(self):
        if not self.is_running:
            return
        if self.args is None:
            retval = self.cb()
        else:
            retval = self.cb(*self.args)
        self.ret += [retval]
        t = threading.Timer(self.rate, self._run)
        t.start()

    def stop(self):
        if not self.is_running:
            return None
        self.is_running = False
        return self.ret

##
# Monitors perception channels on the robot arms.
class ArmPerceptionMonitor( ):

    def __init__(self, arm, rate):
        log("Initializing arm perception listeners")

        self.rate = rate

        if arm == 0:
            armc = "r"
        else:
            armc = "l"
        self.accel_listener = GenericListener("accel_mon_node", AccelerometerState, 
                                 "accelerometer/" + armc + "_gripper_motor", 0.1, accel_state_processor)
        self.cbs = [self.accel_listener.read]

        self.pmonitors = [None] * len(self.cbs)
        self.datasets = [None] * len(self.cbs)
        
        log("Finished initialization")

    def start(self):
        for i in range(len(self.cbs)):
            self.pmonitors[i] = PeriodicMonitor(self.cbs[i], self.rate)

        for i in range(len(self.cbs)):
            self.pmonitors[i].start()

    def stop(self):
        for i in range(len(self.cbs)):
            if self.datasets[i] is None:
                self.datasets[i] = [self.pmonitors[i].stop()]
            else:
                self.datasets[i] += [self.pmonitors[i].stop()]
    
    ##
    # Returns a model function of the perception over several
    # identical trajectories.
    # 
    # @param i the particular dataset to generate the model for
    def generate_model(self, i, smooth_wind=50, var_wind=30):
        ret_means, ret_vars = [], []
        for coord in range(len(self.datasets[0][0][0][1])):
            mean_models, variance_models = [], []
            for model in self.datasets[i]:
                model_coord = zip(*zip(*model)[1])[coord]
                cur_mean_model = signal_smooth(np.array(model_coord), smooth_wind)
                mean_models += [cur_mean_model]
                variance_models += [signal_variance(model_coord, cur_mean_model, var_wind)]
            
            lens = [len(a) for a in mean_models]
            min_len = np.min(lens)
            num_models = len(mean_models)
            avg_means_model, avg_vars_model = [], []
            for k in range(min_len):
                sum_mean, sum_var = 0., 0.
                for j in range(num_models):
                    sum_mean += mean_models[j][k]
                    sum_var += variance_models[j][k]
                avg_means_model += [sum_mean / num_models]
                avg_vars_model += [sum_var / num_models]
            ret_means += [avg_means_model]
            ret_vars += [avg_vars_model]

        return zip(*ret_means), zip(*ret_vars)

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous=True)

    apm = ArmPerceptionMonitor(0, 0.001)
    apm.start()
    rospy.sleep(5.)
    apm.stop()
    means, vars = apm.generate_model(0, 100, 50)
    
    xm, ym, zm = zip(*means)
    xv, yv, zv = zip(*vars)
    import matplotlib.pyplot as plt
    plt.subplot(321)
    plt.plot(xm)
    #plt.axis([0, len(xm), 0., 15.])
    plt.title("X mean")
    plt.subplot(323)
    plt.plot(ym)
    plt.title("Y mean")
    plt.subplot(325)
    plt.plot(zm)
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
