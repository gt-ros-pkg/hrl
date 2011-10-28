#! /usr/bin/python
        
import numpy as np
import copy
from threading import Lock

import roslib
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('hrl_pr2_arms')

import rospy
import tf
from geometry_msgs.msg import PoseStamped

from hrl_generic_arms.ep_trajectory_controller import EPTrajectoryControl, min_jerk_traj
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions
from hrl_rfh_fall_2011.ellipsoid_space import EllipsoidSpace
from hrl_rfh_fall_2011.srv import EllipsoidCommand
from hrl_phri_2011.msg import EllipsoidParams
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

class EllipsoidController(object):
    def __init__(self, arm):
        self.time_step = 1. / 20.
        self.ell_steps = [0.1, 0.1, 0.1] # TODO TUNE THESE
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
        self.ell_cmd_srv = rospy.Service("/ellipsoid_command", EllipsoidCommand, self.ellipsoid_command_srv)
        while not rospy.is_shutdown() and not self.found_params:
            rospy.sleep(0.1)

    def read_params(self, e_params):
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

    def ellipsoidal_pose(self, lat, lon, height, gripper_rot, cur_time=None):
        """
            Get pose in robot's frame of ellipsoidal coordinates
        """
        if cur_time is None:
            cur_time = rospy.Time.now()
        ell_pose = PoseConverter.to_pose_stamped_msg("/ellipse_frame", 
                                       self.ell_space.ellipsoidal_to_pose(lat, lon, height, gripper_rot))
        ell_pose.header.stamp = cur_time
        self.tf_list.waitForTransform("/torso_lift_link", "/ellipse_frame", cur_time, rospy.Duration(3))
        tool_pose = self.tf_list.transformPose("/torso_lift_link", ell_pose)
        return PoseConverter.to_pos_rot(tool_pose)
                                          

    def reset_arm_orientation(self, duration=10.):
        with self.ell_cmd_lock:
            num_samps = duration / self.time_step
            cur_pose = self.arm.get_end_effector_pose()
            ell_pose = self.ellipsoidal_pose(*(self.ell_ep + [self.gripper_rot]))
            adjust_traj = self.arm.interpolate_ep(cur_pose, ell_pose, min_jerk_traj(num_samps))
            ep_traj_control = EPTrajectoryControl(self.arm, adjust_traj)
            self.start_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cur_pose))
            self.end_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", ell_pose))
            self.ell_traj_behavior.epc_motion(ep_traj_control, self.time_step)

    def command_move(self, direction, duration=5.):
        ell_f = copy.copy(self.ell_ep)
        for i in range(3):
            if direction[i] > 0:
                ell_f[i] += self.ell_steps[i]
            elif direction[i] < 0:
                ell_f[i] -= self.ell_steps[i]
        self.execute_trajectory(ell_f, duration)

    def ellipsoid_command_srv(self, req):
        with self.ell_cmd_lock:
            direction = [req.change_latitude, req.change_longitude, req.change_height]
            rospy.loginfo("Commanding ellipsoidal move: (%d, %d, %d)" % 
                           (direction[0], direction[1], direction[2]))
            if req.duration == 0:
                self.command_move(direction)
            else:
                self.command_move(direction, req.duration)

    def execute_trajectory(self, ell_f, duration=5.):
#self.ctrl_switcher.carefree_switch('l', '%s_cart', None) #TODO BETTER SOLUTION
#setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
#self.arm.set_gains([250, 600, 600, 40, 40, 40], [15, 15, 15, 1.2, 1.2, 1.2])
#self.arm.set_posture(setup_angles)

        num_samps = int(duration / self.time_step)
        t_vals = min_jerk_traj(num_samps)
        self.reset_ell_ep()
        self.arm.reset_ep()
        ell_init = np.mat(self.ell_ep).T
        ell_final = np.mat(ell_f).T
        ell_traj = np.array(ell_init) + np.array(np.tile(ell_final - ell_init, (1, num_samps))) * np.array(t_vals)
        cur_time = rospy.Time.now()
        ell_pose_traj = [self.ellipsoidal_pose(*x) 
                         for x in np.vstack((ell_traj, [self.gripper_rot]*num_samps, [cur_time]*num_samps)).T.tolist()]
        # replace the rotation matricies with a simple cartesian slerp
        cart_interp_traj = self.arm.interpolate_ep(self.arm.get_ep(), ell_pose_traj[-1], 
                                                   min_jerk_traj(num_samps))
#fixed_traj = [(ell_pose_traj[i][0], cart_interp_traj[i][1]) for i in range(num_samps)]

        self.start_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cart_interp_traj[0]))
        self.end_pub.publish(PoseConverter.to_pose_stamped_msg("/torso_lift_link", cart_interp_traj[-1]))
        ep_traj_control = EPTrajectoryControl(self.arm, cart_interp_traj)
        self.ell_traj_behavior.epc_motion(ep_traj_control, self.time_step)
        self.ell_ep = ell_f

def main():
    rospy.init_node("ellipsoid_controller")

    rospy.sleep(1)

    setup_angles = [0, 0, np.pi/2, -np.pi/2, -np.pi, -np.pi/2, -np.pi/2]
    cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, end_link="%s_gripper_shaver45_frame")
    cart_arm.set_posture(setup_angles)

    cart_arm.set_gains([250, 600, 600, 40, 40, 40], [15, 15, 15, 1.2, 1.2, 1.2])
    ell_controller = EllipsoidController(cart_arm)
#ell_controller.reset_arm_orientation(8)
    rospy.spin()
    

if __name__ == "__main__":
    main()
