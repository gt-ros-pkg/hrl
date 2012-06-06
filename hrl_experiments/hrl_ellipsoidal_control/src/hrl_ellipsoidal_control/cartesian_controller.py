#! /usr/bin/python
        
import sys

import roslib
roslib.load_manifest('hrl_ellipsoidal_control')
from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController

class CartesianController(EllipsoidController):

    def _get_ell_equiv_dist(self, ell_change_ep, ell_abs_sel, orient_quat, velocity):
        ell_f, rot_mat_f = self._parse_ell_move(ell_change_ep, ell_abs_sel, orient_quat)
        traj = self._create_ell_trajectory(ell_f, rot_mat_f, orient_quat, velocity)
        dist = np.linalg.norm(traj[0][0] - traj[-1][0])
        return dist

    def execute_cart_move(self, change_ep, abs_sel, orient_quat=[0., 0., 0., 1.], velocity=0.001,
                          num_samps=None, blocking=True):
        with self.cmd_lock:
            cur_pos, cur_rot = self.arm.get_ep()
            change_pos_ep, change_rot_ep = change_ep
            abs_cart_ep_sel, is_abs_rot = abs_sel
            pos_f = np.where(abs_cart_ep_sel, change_pos_ep, 
                             np.array(cur_pos + cur_rot * np.mat(change_pos_ep).T).T[0])
            if is_abs_rot:
                # TODO FIX THIS
                rot_change_mat = change_rot_ep
                _, ell_final_rot = self.robot_ellipsoidal_pose(ell_f[0], ell_f[1], ell_f[2],
                                                               orient_quat)
                rot_mat_f = ell_final_rot * rot_change_mat
            else:
                rpy = change_rot_ep
                _, cur_rot = self.arm.get_ep()
                rot_mat = np.mat(tf_trans.euler_matrix(*rpy))[:3,:3]
                rot_mat_f = cur_rot * rot_mat
            traj = self._create_cart_trajectory(pos_f, rot_mat_f, orient_quat, velocity, num_samps)
        self._run_traj(traj, blocking=blocking)

        # TODO huh? TODO
        ell_traj = self._parse_ell_move(change_ep, abs_sel, orient_quat, velocity)

    def _create_cart_trajectory(self, pos_f, rot_mat_f, orient_quat=[0., 0., 0., 1.], velocity=0.001,
                                velocity=0.001, num_samps=None):
        cur_pos, cur_rot = self.arm.get_ep()

        rpy = tf_trans.euler_from_matrix(cur_rot.T * rot_mat_f) # get roll, pitch, yaw of angle diff

        if num_samps is None:
            num_samps = np.max([2, int(np.linalg.norm(pos_f - cur_pos) / velocity), 
                                   int(np.linalg.norm(rpy) / velocity)])

        ell_frame_mat = self.get_ell_frame()

        traj = self.arm.interpolate_ep([cur_pos, cur_rot], 
                                       [np.mat(pos_f).T, rot_mat_f], 
                                       min_jerk_traj(num_samps))
        return traj

def main():
    rospy.init_node("cartesian_controller", sys.argv)
    cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                              controller_name='%s_cart_jt_task', 
                              end_link="%s_gripper_shaver45_frame", timeout=0)

    rospy.sleep(1)
    cart_controller = CartesianController(cart_arm)
    rospy.spin()
    

if __name__ == "__main__":
    main()
