
import numpy as np

import roslib
roslib.load_manifest('hrl_generic_arms')
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions

def min_jerk_traj(n):
    return [(10 * t**3 - 15 * t**4 + 6 * t**5)
            for t in np.linspace(0, 1, n)]

class EPTrajectoryControl(EPGenerator):
    def __init__(self, control_arm, trajectory, time_step=0.1):
        self.control_arm = control_arm
        self.trajectory = trajectory
        self.time_step = time_step
        self.t_i = 0

    def generate_ep(self):
        ep_new = self.trajectory[self.t_i]
        self.t_i += 1
        return EPStopConditions.CONTINUE, ep_new

    def control_ep(self, ep):
        self.control_arm.set_ep(ep, self.time_step)

    def clamp_ep(self, ep):
        return ep
        
    def terminate_check(self):
        if self.t_i == len(self.trajectory):
            return EPStopConditions.SUCCESSFUL
