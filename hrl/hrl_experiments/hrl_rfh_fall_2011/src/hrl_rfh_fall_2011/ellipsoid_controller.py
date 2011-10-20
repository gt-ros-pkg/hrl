#! /usr/bin/python
        
import numpy as np

import roslib
roslib.load_manifest('hrl_generic_arms')

from hrl_generic_arms.ep_trajectory_controller import EPTrajectoryControl, min_jerk_traj
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions

class EllipsoidController(object):
    def __init__(self):
        self.time_step = 1. / 20.
        self.ell_ep = [0, 0, 0] # TODO
        self.gripper_rot = 0.
    
    def execute_trajectory(ell_f, duration=5.):
        num_samps = duration / self.time_step
        t_vals = np.mat(min_jerk_traj(num_samps)).T
        ell_init = np.mat(self.ell_ep).T
        ell_final = np.mat(ell_f).T
        ell_traj = ell_init + np.tile(ell_final - ell_init, (1, num_samps)) * t_vals
        ell_pose_traj = [self.ell_space.ellipsoidal_to_pose(*x) 
                         for x in np.vstack((ell_traj, [self.gripper_rot]*num_samps)).T.tolist()]
        ell_traj_behavior.epc_motion(EPTrajectoryControl(arm, ell_pose_traj), self.time_step)
        self.ell_ep = ell_f

def main():
    rospy.init_node("ellipsoid_controller")
    ell_traj_behavior = EPC("ellipsoid_traj")
    

if __name__ == "__main__":
    main()
