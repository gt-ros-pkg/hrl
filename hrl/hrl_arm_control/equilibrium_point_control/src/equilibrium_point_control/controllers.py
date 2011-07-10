import numpy as np, math

import roslib; roslib.load_manifest('equilibrium_point_control')
import rospy
from std_msgs.msg import Float64

##
# Classic PID controller with maximum integral thresholding to
# prevent windup.
class PIDController(object):
    def __init__(self, k_p, k_i, k_d, i_max, rate, name=None):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.i_max = i_max
        self.rate = rate
        self.err_last = None
        self.integral = 0
        if name is not None:
            self.err_pub = rospy.Publisher("/pid_controller/" + name + "_error", Float64)
            self.out_pub = rospy.Publisher("/pid_controller/" + name + "_output", Float64)

    ##
    # Updates the controller state and returns the current output.
    def update_state(self, err):
        self.integral += err / self.rate
        self.integral = np.clip(self.integral, -self.i_max, self.i_max)
        if self.err_last is None:
            self.err_last = err
        y = (self.k_p * err + 
             self.k_d * (err - self.err_last) * self.rate +
             self.k_i * self.integral)
        self.err_last = err
        self.err_pub.publish(err)
        self.out_pub.publish(y)
        return y

    def reset_controller(self):
        self.err_last = None
        self.integral = 0
