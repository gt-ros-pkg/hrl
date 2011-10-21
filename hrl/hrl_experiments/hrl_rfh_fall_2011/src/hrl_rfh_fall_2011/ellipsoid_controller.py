#! /usr/bin/python
        
import numpy as np
import copy

import roslib
roslib.load_manifest('hrl_generic_arms')

from hrl_generic_arms.ep_trajectory_controller import EPTrajectoryControl, min_jerk_traj
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions
from hrl_rfh_fall_2011.ellipsoid_space import EllipsoidSpace

class EllipsoidController(object):
    def __init__(self, ell_space):
        self.time_step = 1. / 20.
        self.ell_steps = [0.1, 0.1, 0.1] # TODO TUNE THESE
        self.gripper_rot = 0.
        self.hybrid_arm = create_pr2_arm('l', PR2ArmHybridForce)
        self.ell_traj_behavior = EPC("ellipsoid_traj")
        self.ell_space = ell_space
        self.reset_ell_ep()
    
    def reset_ell_ep(self):
        pos, quat = PoseConverter.to_pos_quat(self.hybrid_arm.get_end_effector_pose())
        self.ell_ep = list(self.ell_space.pos_to_ellipsoidal(*pos))

    def reset_arm_orientation(self, duration=10.):
        num_samps = duration / self.time_step
        cur_pose = self.get_end_effector_pose()
        ell_pose = self.ell_space.ellipsoidal_to_pose(*(self.ell_ep + [self.gripper_rot]))
        self.hybrid_arm.interpolate_ep(cur_pose, ell_pose, min_jerk_traj(num_samps))

    def command_move(self, direction, duration=5.):
        ell_f = copy.copy(self.ell_ep)
        for i in range(3):
            if direction > 0:
                ell_f[i] += self.ell_steps[i]
            elif direction < 0:
                ell_f[i] -= self.ell_steps[i]
        self.execute_trajectory(ell_f, duration)

    def execute_trajectory(self, ell_f, duration=5.):
        num_samps = duration / self.time_step
        t_vals = np.mat(min_jerk_traj(num_samps)).T
        ell_init = np.mat(self.ell_ep).T
        ell_final = np.mat(ell_f).T
        ell_traj = ell_init + np.tile(ell_final - ell_init, (1, num_samps)) * t_vals
        ell_pose_traj = [self.ell_space.ellipsoidal_to_pose(*x) 
                         for x in np.vstack((ell_traj, [self.gripper_rot]*num_samps)).T.tolist()]
        ep_traj_control = EPTrajectoryControl(self.hybrid_arm, ell_pose_traj)
        self.ell_traj_behavior.epc_motion(ep_traj_control, self.time_step)
        self.ell_ep = ell_f

def main():
    rospy.init_node("ellipsoid_controller")
    E = 1
    ell_space = EllipsoidSpace(E, np.mat([0.78, -0.28, 0.3]).T)
    ell_controller = EllipsoidController(ell_space)
    ell_controller.reset_arm_orientation()
    rospy.sleep(1)
    ell_controller.command_move([0, 1, 0], 10)
    

if __name__ == "__main__":
    main()
