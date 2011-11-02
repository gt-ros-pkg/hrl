#! /usr/bin/python
        
import numpy as np
import scipy.stats as stats
import copy
import sys
from threading import Lock

import roslib
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('hrl_pr2_arms')

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import tf.transformations as tf_trans

from hrl_generic_arms.ep_trajectory_controller import EPTrajectoryControl, min_jerk_traj
from equilibrium_point_control.ep_control import EPC, EPStopConditions
from hrl_rfh_fall_2011.ellipsoid_space import EllipsoidSpace
from hrl_rfh_fall_2011.sm_registration_setup import SetupTaskController
from hrl_rfh_fall_2011.msg import EllipsoidMoveAction, EllipsoidMoveResult
from hrl_phri_2011.msg import EllipsoidParams
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def print_trajectory_stats(ep_traj, traj, t_vals):
    diffs_pos = [np.linalg.norm(traj[i+1][0] - traj[i][0]) for i in range(len(traj)-1)]
    diffs_rot = [2 * np.arccos(np.fabs(np.dot(PoseConverter.to_pos_quat(traj[i+1])[1],
                                              PoseConverter.to_pos_quat(traj[i])[1]))) 
                                                           for i in range(len(traj)-1)]
    ptiles = [10, 25, 50, 75, 90, 100]
    print "Diffs pos:", ", ".join(["%d:%f" % (ptile, stats.scoreatpercentile(diffs_pos, ptile)) 
                                   for ptile in ptiles])
    print "Diffs pos:", ", ".join(["%d:%f" % (ptile, stats.scoreatpercentile(diffs_rot, ptile)) 
                                   for ptile in ptiles])
    if max(diffs_pos) > 0.01:
        print traj
        print ep_traj
        print t_vals
        print "ind:", np.argmax(diffs_pos)
        print ep_traj.T[np.argmax(diffs_pos)]
        print traj[np.argmax(diffs_pos)]


class EllipsoidController(object):
    def __init__(self, arm):
        self.time_step = 1. / 20.
        self.gripper_rot = np.pi 
        self.arm = arm
        self.ell_traj_behavior = EPC("ellipsoid_traj")
        self.ell_space = EllipsoidSpace(1)
        self.tf_list = tf.TransformListener()
        self.found_params = False
        self.ell_sub = rospy.Subscriber("/ellipsoid_params", EllipsoidParams, self.read_params)
        self.start_pub = rospy.Publisher("/start_pose", PoseStamped)
        self.end_pub = rospy.Publisher("/end_pose", PoseStamped)
        self.ctrl_switcher = ControllerSwitcher()
        self.ell_cmd_lock = Lock()
        self.params_lock = Lock()
        self.ell_ep = None
        self.action_preempted = False
        self.ell_move_act = actionlib.SimpleActionServer("/ellipsoid_move", EllipsoidMoveAction, 
                                                         self.command_move_exec, False)
        self.ell_move_act.start()

    def read_params(self, e_params):
        with self.params_lock:
            self.ell_space.load_ell_params(e_params)
            self.ell_space.center = np.mat(np.zeros((3, 1)))
            self.ell_space.rot = np.mat(np.eye(3))
            if not self.found_params:
                rospy.loginfo("[ellipsoid_controller] Found params from /ellipsoid_params")
                self.reset_ell_ep()
            self.found_params = True
    
    def reset_ell_ep(self):
        ee_pose = PoseConverter.to_pose_stamped_msg("/torso_lift_link", self.arm.get_end_effector_pose())
        cur_time = rospy.Time.now()
        ee_pose.header.stamp = cur_time
        self.tf_list.waitForTransform("/torso_lift_link", "/ellipse_frame", cur_time, rospy.Duration(3))
        ell_pose = self.tf_list.transformPose("/ellipse_frame", ee_pose)
        pos, quat = PoseConverter.to_pos_quat(ell_pose)
        self.ell_ep = list(self.ell_space.pos_to_ellipsoidal(*pos))

    def update_ellipse_pose(self):
        cur_time = rospy.Time.now()
        self.tf_list.waitForTransform("/torso_lift_link", "/ellipse_frame", cur_time, rospy.Duration(3))
        cur_tf = self.tf_list.lookupTransform("/torso_lift_link", "/ellipse_frame", cur_time, rospy.Duration(3))
        self.ell_tf = PoseConverter.to_homo_mat(cur_tf)

    def robot_ellipsoidal_pose(self, lat, lon, height, gripper_rot, ell_frame_mat):
        """
            Get pose in robot's frame of ellipsoidal coordinates
        """
        pos, quat = self.ell_space.ellipsoidal_to_pose(lat, lon, height)
        quat_gripper_rot = tf_trans.quaternion_from_euler(gripper_rot, 0, 0)
        quat_rotated = tf_trans.quaternion_multiply(quat, quat_gripper_rot)
        ell_pose_mat = PoseConverter.to_homo_mat(pos, quat_rotated)
        return PoseConverter.to_pos_rot(ell_frame_mat * ell_pose_mat)
                                          

#   def reset_arm_orientation(self, duration=10.):
#       if not ell_ep:
#           rospy.logerr("Haven't gotten parameters from /ellipsoid_params")
#           return
#       with self.ell_cmd_lock:
#           num_samps = duration / self.time_step
#           cur_pose = self.arm.get_end_effector_pose()
#           ell_pose = self.robot_ellipsoidal_pose(*(self.ell_ep + [self.gripper_rot]))
#           adjust_traj = self.arm.interpolate_ep(cur_pose, ell_pose, min_jerk_traj(num_samps))
#           ep_traj_control = EPTrajectoryControl(self.arm, adjust_traj)
#           self.start_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cur_pose))
#           self.end_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", ell_pose))
#           self.ell_traj_behavior.epc_motion(ep_traj_control, self.time_step)

    def command_stop(self):
        self.ell_traj_behavior.stop_epc = True

    def command_move_exec(self, req):
        with self.params_lock:
            with self.ell_cmd_lock:
                change_ep = np.array([req.change_latitude, req.change_longitude, req.change_height])
                abs_ep_sel = np.array([req.absolute_latitude, req.absolute_longitude, req.absolute_height])
                rospy.loginfo("Commanding ellipsoidal move: (%f, %f, %f), Abs: (%d, %d, %d)" % 
                               (change_ep[0], change_ep[1], change_ep[2], 
                                abs_ep_sel[0], abs_ep_sel[1], abs_ep_sel[2]))
                self.arm.reset_ep()
                self.reset_ell_ep()
                ell_f = np.where(abs_ep_sel, change_ep, self.ell_ep + change_ep)
                if req.duration == 0:
                    self.execute_trajectory(ell_f, req.gripper_rot)
                else:
                    self.execute_trajectory(ell_f, req.gripper_rot, req.duration)
                self.ell_move_act.set_succeeded(EllipsoidMoveResult(*self.ell_ep))

    def _check_preempt(self, timer_event):
        if self.ell_move_act.is_preempt_requested():
            self.ell_traj_behavior.stop_epc = True
            self.action_preempted = True

    def execute_trajectory(self, ell_f, gripper_rot, duration=5.):
        ell_f[1] = np.mod(ell_f[1], 2 * np.pi)
        num_samps = int(duration / self.time_step)
        t_vals = min_jerk_traj(num_samps)
        self.arm.reset_ep()
        self.reset_ell_ep()
        ell_init = np.mat(self.ell_ep).T
        ell_final = np.mat(ell_f).T
        if np.fabs(2 * np.pi + ell_final[1,0] - ell_init[1,0]) < np.pi:
            ell_final[1,0] += 2 * np.pi
            print "wrapping; ell_f:", ell_f
        elif np.fabs(-2 * np.pi + ell_final[1,0] - ell_init[1,0]) < np.pi:
            ell_final[1,0] -= 2 * np.pi
            print "wrapping; ell_f:", ell_f
        
            
        print "init", ell_init, "final", ell_final
        ell_traj = np.array(ell_init) + np.array(np.tile(ell_final - ell_init, (1, num_samps))) * np.array(t_vals)

        # find the current ellipsoid frame
        cur_time = rospy.Time.now()
        self.tf_list.waitForTransform("/torso_lift_link", "/ellipse_frame", cur_time, rospy.Duration(3))
        ell_frame_mat = PoseConverter.to_homo_mat(
                             self.tf_list.lookupTransform("/torso_lift_link", 
                                                          "/ellipse_frame", cur_time))
        print ell_frame_mat

        print ell_traj.shape
        ell_pose_traj = [self.robot_ellipsoidal_pose(ell_traj[0,i], ell_traj[1,i], ell_traj[2,i],
                                                     gripper_rot, ell_frame_mat) 
                         for i in range(ell_traj.shape[1])]
        # replace the rotation matricies with a simple cartesian slerp
        cart_interp_traj = self.arm.interpolate_ep(self.arm.get_ep(), ell_pose_traj[-1], 
                                                   t_vals)
        fixed_traj = [(ell_pose_traj[i][0], cart_interp_traj[i][1]) for i in range(num_samps)]

        self.start_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cart_interp_traj[0]))
        self.end_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cart_interp_traj[-1]))
#ep_traj_control = EPTrajectoryControl(self.arm, cart_interp_traj)
        print_trajectory_stats(ell_traj, fixed_traj, t_vals)
        ep_traj_control = EPTrajectoryControl(self.arm, fixed_traj)
        self.action_preempted = False
        self.ell_traj_behavior.stop_epc = False
        monitor_timer = rospy.Timer(rospy.Duration(0.1), self._check_preempt)
        self.ell_traj_behavior.epc_motion(ep_traj_control, self.time_step)
        monitor_timer.shutdown()
        if self.action_preempted:
            self.reset_ell_ep()
        else:
            self.ell_ep = ell_f

def main():
    rospy.init_node("ellipsoid_controller", sys.argv)

    if sys.argv[1] == '-s':
        setup_task_controller = SetupTaskController()
        setup_task_controller.execute(None)
        cart_arm = setup_task_controller.arm
    else:
        cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, end_link="%s_gripper_shaver45_frame")

    rospy.sleep(1)
    ell_controller = EllipsoidController(cart_arm)
    rospy.spin()
    

if __name__ == "__main__":
    main()
