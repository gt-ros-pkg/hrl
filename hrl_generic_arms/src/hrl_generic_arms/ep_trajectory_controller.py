
import numpy as np

import roslib
roslib.load_manifest('hrl_generic_arms')
import rospy

from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions

##
# Returns a minimum jerk trajectory from x=0..1 given
# the number of samples in the trajectory.  Assumes
# x_i = 0, x_f = 1, v_i = 0, v_f = 0.
def min_jerk_traj(n):
    return [(10 * t**3 - 15 * t**4 + 6 * t**5)
            for t in np.linspace(0, 1, n)]

##
# Implements a generic equilibirum point generator given a list
# of EPs and the arm object to execute them on.
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
        return EPStopConditions.CONTINUE

##
# Equilbrium Point Control object used to execute a trajectory from 
# the current EP to the end EP using a minimum jerk interpolation.
class EPArmController(EPC):
    def __init__(self, arm, time_step=0.1, epc_name='epc_arm_controller'):
        super(EPArmController, self).__init__(epc_name)
        self.arm = arm
        self.time_step = time_step

    def execute_interpolated_ep(self, end_ep, duration, blocking=True):
        num_samps = duration / self.time_step
        joint_traj = self.arm.interpolate_ep(self.arm.get_ep(), end_ep, min_jerk_traj(num_samps))
        ep_traj_control = EPTrajectoryControl(self.arm, joint_traj, self.time_step)
        def exec_motion(event):
            self.epc_motion(ep_traj_control, self.time_step)
        if blocking:
            exec_motion(None)
        else:
            rospy.Timer(rospy.Duration(0.01), exec_motion, oneshot=True)
