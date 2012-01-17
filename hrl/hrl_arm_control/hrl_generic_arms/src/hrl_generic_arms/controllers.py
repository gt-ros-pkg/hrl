import numpy as np, math

import roslib; roslib.load_manifest('hrl_generic_arms')
import rospy
from std_msgs.msg import Float64

##
# Classic PID controller with maximum integral thresholding to
# prevent windup.
class PIDController(object):
    def __init__(self, rate=100., k_p=0., k_i=0., k_d=0., i_max=None, saturation=None, name=None):
        self.y = 0.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.i_max = i_max
        self.rate = rate
        self.saturation = saturation
        self.err_last = None
        self.integral = 0
        if name is not None:
            self.has_pub = True
            self.err_pub = rospy.Publisher("/pid_controller/" + name + "_error", Float64)
            self.out_pub = rospy.Publisher("/pid_controller/" + name + "_output", Float64)
        else:
            self.has_pub = False

    ##
    # Updates the controller state and returns the current output.
    def update_state(self, err):
        self.integral += err / self.rate
        if self.i_max is not None:
            self.integral = np.clip(self.integral, -self.i_max, self.i_max)
        if self.err_last is None:
            self.err_last = err
        self.y = (self.k_p * err + 
                  self.k_d * (err - self.err_last) * self.rate +
                  self.k_i * self.integral)
        self.err_last = err
        if self.saturation is not None:
            self.y = np.clip(self.y, -self.saturation, self.saturation)
            if self.y in [-self.saturation, self.saturation]:
                self.integral = 0.
        if self.has_pub:
            self.err_pub.publish(err)
            self.out_pub.publish(self.y)
        return self.y

    def reset_controller(self):
        self.err_last = None
        self.integral = 0
