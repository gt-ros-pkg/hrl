#! /usr/bin/python
        
import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_ellipsoidal_control')

import tf.transformations as tf_trans

from hrl_ellipsoidal_control.controller_base import EllipsoidControllerBase, min_jerk_traj

MIN_HEIGHT = 0.2

class EllipsoidController(EllipsoidControllerBase):

    def __init__(self):
        super(EllipsoidController, self).__init__()
        self._lat_bounds = (-np.inf, np.inf)
        self._lon_bounds = (-np.inf, np.inf)
        self._height_bounds = (-np.inf, np.inf)

    def execute_ell_move(self, change_ep, abs_sel, orient_quat=[0., 0., 0., 1.], 
                         velocity=0.001, blocking=True):
        ell_f, rot_mat_f = self._parse_ell_move(change_ep, abs_sel, orient_quat)
        traj = self._create_ell_trajectory(ell_f, rot_mat_f, orient_quat, velocity)
        if traj is None:
            rospy.logerr("[ellipsoid_controller] Bad trajectory.")
            return False
        return self._run_traj(traj, blocking=blocking)

    def set_bounds(self, lat_bounds, lon_bounds, height_bounds):
        assert lon_bounds[1] >= 0
        self._lat_bounds = lat_bounds
        self._lon_bounds = lon_bounds
        self._height_bounds = height_bounds

    def _clip_ell_ep(self, ell_ep):
        lat = np.clip(ell_ep[0], self._lat_bounds[0], self._lat_bounds[1])
        if self._lon_bounds[0] >= 0:
            lon = np.clip(ell_ep[1], self._lon_bounds[0], self._lon_bounds[1])
        else:
            ell_ep_1 = np.mod(ell_ep[1], 2 * np.pi)
            min_lon = np.mod(self._lon_bounds[0], 2 * np.pi)
            if ell_ep_1 >= min_lon or ell_ep_1 <= self._lon_bounds[1]:
                lon = ell_ep[1]
            else:
                if min_lon - ell_ep_1 < ell_ep_1 - self._lon_bounds[1]:
                    lon = min_lon
                else:
                    lon = self._lon_bounds[1]
        height = np.clip(ell_ep[2], self._height_bounds[0], self._height_bounds[1])
        return np.array([lat, lon, height])

    def arm_in_bounds(self):
        ell_ep = self.get_ell_ep()
        equals = ell_ep == self._clip_ell_ep(ell_ep)
        print ell_ep, equals
        if self._lon_bounds[0] >= 0 and ell_ep[1] >= 0:
            return np.all(equals)
        else:
            ell_ep_1 = np.mod(ell_ep[1], 2 * np.pi)
            min_lon = np.mod(self._lon_bounds[0], 2 * np.pi)
            return (equals[0] and equals[2] and 
                    (ell_ep_1 >= min_lon or ell_ep_1 <= self._lon_bounds[1]))


    def _parse_ell_move(self, change_ep, abs_sel, orient_quat):
        with self.cmd_lock:
            change_ell_ep, change_rot_ep = change_ep
            abs_ell_ep_sel, is_abs_rot = abs_sel
            ell_f = np.where(abs_ell_ep_sel, change_ell_ep, 
                                         np.array(self.get_ell_ep()) + np.array(change_ell_ep))
            print "old", ell_f
            if ell_f[0] > np.pi:
                ell_f[0] = 2 * np.pi - ell_f[0]
                ell_f[1] *= -1
            if ell_f[0] < 0.:
                ell_f[0] *= -1
                ell_f[1] *= -1
            ell_f[1] = np.mod(ell_f[1], 2 * np.pi)
            ell_f = self._clip_ell_ep(ell_f)
            print "new", ell_f
            if is_abs_rot:
                rot_change_mat = change_rot_ep
                _, ell_final_rot = self.robot_ellipsoidal_pose(ell_f[0], ell_f[1], ell_f[2],
                                                               orient_quat)
                rot_mat_f = ell_final_rot * rot_change_mat
            else:
                rpy = change_rot_ep
                _, cur_rot = self.arm.get_ep()
                rot_mat = np.mat(tf_trans.euler_matrix(*rpy))[:3,:3]
                rot_mat_f = cur_rot * rot_mat
            return ell_f, rot_mat_f

    def _create_ell_trajectory(self, ell_f, rot_mat_f, orient_quat=[0., 0., 0., 1.], velocity=0.001):
        _, cur_rot = self.arm.get_ep()

        rpy = tf_trans.euler_from_matrix(cur_rot.T * rot_mat_f) # get roll, pitch, yaw of angle diff

        ell_f[1] = np.mod(ell_f[1], 2 * np.pi) # wrap longitude value

        #ell_f[2] = max(ell_f[2], MIN_HEIGHT) # don't want to approach singularity

        ell_init = np.mat(self.get_ell_ep()).T # get the current ellipsoidal location of the end effector
        ell_final = np.mat(ell_f).T

        # find the closest longitude angle to interpolate to
        if np.fabs(2 * np.pi + ell_final[1,0] - ell_init[1,0]) < np.pi:
            ell_final[1,0] += 2 * np.pi
        elif np.fabs(-2 * np.pi + ell_final[1,0] - ell_init[1,0]) < np.pi:
            ell_final[1,0] -= 2 * np.pi
        
        if np.any(np.isnan(ell_init)) or np.any(np.isnan(ell_final)):
            rospy.logerr("[ellipsoid_controller] Nan values in ellipsoid EPs. " +
                         "ell_init: %f, %f, %f; " % (ell_init[0,0], ell_init[1,0], ell_init[2,0]) +
                         "ell_final: %f, %f, %f; " % (ell_final[0,0], ell_final[1,0], ell_final[2,0]))
            return None
        
        num_samps = np.max([2, int(np.linalg.norm(ell_final - ell_init) / velocity), 
                               int(np.linalg.norm(rpy) / velocity)])
        t_vals = min_jerk_traj(num_samps) # makes movement smooth
            
        # smoothly interpolate from init to final
        ell_traj = np.array(ell_init) + np.array(np.tile(ell_final - ell_init, 
                                                         (1, num_samps))) * np.array(t_vals)

        ell_frame_mat = self.get_ell_frame()

        ell_pose_traj = [self.robot_ellipsoidal_pose(ell_traj[0,i], ell_traj[1,i], ell_traj[2,i],
                                                     orient_quat, ell_frame_mat) 
                         for i in range(ell_traj.shape[1])]

        # modify rotation of trajectory
        _, ell_init_rot = self.robot_ellipsoidal_pose(ell_init[0,0], ell_init[1,0], ell_init[2,0],
                                                      orient_quat, ell_frame_mat)
        rot_adjust_traj = self.arm.interpolate_ep([np.mat([0]*3).T, cur_rot], 
                                                  [np.mat([0]*3).T, rot_mat_f], 
                                                  min_jerk_traj(num_samps))
        ell_pose_traj = [(ell_pose_traj[i][0], 
                          ell_pose_traj[i][1] * ell_init_rot.T * rot_adjust_traj[i][1]) 
                         for i in range(num_samps)]

        return ell_pose_traj

def main():
    rospy.init_node("ellipsoid_controller", sys.argv)
    cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                              controller_name='%s_cart_jt_task', 
                              end_link="%s_gripper_shaver45_frame", timeout=0)

    rospy.sleep(1)
    ell_controller = EllipsoidController()
    ell_controller.set_arm(cart_arm)
    rospy.spin()
    

if __name__ == "__main__":
    main()
