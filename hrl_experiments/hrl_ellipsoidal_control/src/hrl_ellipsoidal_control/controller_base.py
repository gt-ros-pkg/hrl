import numpy as np
import scipy.stats as stats
import copy
import sys
from threading import Lock

import roslib
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('hrl_ellipsoidal_control')
roslib.load_manifest('hrl_pr2_arms')

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import tf.transformations as tf_trans

#from hrl_generic_arms.ep_trajectory_controller import EPTrajectoryControl, min_jerk_traj
#from equilibrium_point_control.ep_control import EPC, EPStopConditions
from ellipsoid_space import EllipsoidSpace
from msg import EllipsoidMoveAction, EllipsoidMoveResult
from msg import EllipsoidParams
from srv import LoadEllipsoidParams, LoadEllipsoidParamsResponse
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def min_jerk_traj(n):
    return [(10 * t**3 - 15 * t**4 + 6 * t**5)
            for t in np.linspace(0, 1, n)]

class CartTrajController(object):
    def __init__(self):
        self._moving_lock = Lock()

    def stop_moving(self, wait=False):
        self._stop_moving = True
        if wait:
            self.wait_until_stopped()

    def is_moving(self):
        return self._moving_lock.locked()

    def wait_until_stopped(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if not self.is_moving():
                return
            r.sleep()

    def execute_cart_traj(self, cart_arm, traj, time_step, blocking=True):
        self._moving_lock.acquire(False)
        self._stop_moving = False
        if blocking:
            result = self._execute_cart_traj(cart_arm, traj, time_step)
            self._moving_lock.release()
            return result
        else:
            def execute_cart_traj_cb(event):
                self._execute_cart_traj(cart_arm, traj, time_step)
                self._moving_lock.release()
            rospy.Timer(rospy.Duration(0.01), execute_cart_traj_cb, oneshot=True)
            return True

    def _execute_cart_traj(self, cart_arm, traj, time_step):
        rate = rospy.Rate(1.0/time_step)
        for ep in traj:
            if rospy.is_shutdown() or self._stop_moving:
                return False
            cart_arm.set_ep(ep, time_step)
            rate.sleep()
        return True

class EllipsoidControllerBase(CartTrajController):
    def __init__(self):
        super(EllipsoidControllerBase, self).__init__()

        self.time_step = 1. / 20.
        self.arm = None
        self.cart_traj_ctrl = CartTrajController()
        self.ell_space = None
        self.head_center = None
        self.tf_list = tf.TransformListener()
        self.cmd_lock = Lock()
        self.ell_param_srv = rospy.Service("/load_ellipsoid_params", LoadEllipsoidParams, 
                                           self.load_params)
        self.start_pub = rospy.Publisher("/start_pose", PoseStamped)
        self.end_pub = rospy.Publisher("/end_pose", PoseStamped)
        self.head_center_pub = rospy.Publisher("/head_center", PoseStamped)
        def pub_head_center(te):
            if self.head_center is not None:
                self.head_center_pub.publish(self.head_center)
        rospy.Timer(rospy.Duration(0.2), pub_head_center)

    def set_arm(self, arm):
        self.arm = arm

    def load_params(self, req):
        rospy.loginfo("Loaded ellispoidal parameters.")
        head_center_kinect = PoseConverter.to_pose_stamped_msg(req.params.e_frame)
        cur_time = rospy.Time.now()
        req.params.e_frame.header.stamp = cur_time
        self.tf_list.waitForTransform("/base_link", "/openni_rgb_optical_frame", cur_time, 
                                      rospy.Duration(3))
        self.head_center = self.tf_list.transformPose("/base_link", head_center_kinect)
        # TODO cleanup EllispoidSpace
        self.ell_space = EllipsoidSpace(1)
        self.ell_space.load_ell_params(req.params)
        self.ell_space.center = np.mat(np.zeros((3, 1)))
        self.ell_space.rot = np.mat(np.eye(3))
        return LoadEllipsoidParamsResponse()

    def params_loaded(self):
        return self.ell_space is not None
    
    def get_ell_ep(self):
        ee_pose = PoseConverter.to_pose_stamped_msg("/torso_lift_link", self.arm.get_ep())
        cur_time = rospy.Time.now()
        ee_pose.header.stamp = cur_time
        self.tf_list.waitForTransform("/torso_lift_link", "/openni_rgb_optical_frame", cur_time, 
                                      rospy.Duration(3))
        ep_opti_frame = PoseConverter.to_homo_mat(
                         self.tf_list.transformPose("/openni_rgb_optical_frame", ee_pose))
        pos, quat = PoseConverter.to_pos_quat(self.get_ell_frame("/openni_rgb_optical_frame")**-1 * 
                                              ep_opti_frame)
        return list(self.ell_space.pos_to_ellipsoidal(*pos))

    ##
    # Get pose in robot's frame of ellipsoidal coordinates
    def robot_ellipsoidal_pose(self, lat, lon, height, orient_quat, kinect_frame_mat=None):
        if kinect_frame_mat is None:
            kinect_frame_mat = self.get_ell_frame()
        pos, quat = self.ell_space.ellipsoidal_to_pose(lat, lon, height)
        quat_rotated = tf_trans.quaternion_multiply(quat, orient_quat)
        ell_pose_mat = PoseConverter.to_homo_mat(pos, quat_rotated)
        return PoseConverter.to_pos_rot(kinect_frame_mat * ell_pose_mat)
                                          
    def reset_arm_orientation(self, duration=10., gripper_rot=np.pi, blocking=True):
        with self.cmd_lock:
            num_samps = duration / self.time_step
            cur_pose = self.arm.get_end_effector_pose()
            quat_gripper_rot = tf_trans.quaternion_from_euler(gripper_rot, 0, 0)
            args = self.get_ell_ep() + [quat_gripper_rot]
            ell_pose = self.robot_ellipsoidal_pose(*args)
            adjust_traj = self.arm.interpolate_ep(cur_pose, ell_pose, 
                                                  min_jerk_traj(num_samps))
            return self._run_traj(adjust_traj, blocking=blocking)

    def command_stop(self):
        self.ell_traj_behavior.stop_epc = True

    def get_ell_frame(self, frame="/torso_lift_link"):
        # find the current ellipsoid frame location in this frame
        cur_time = rospy.Time.now()
        self.tf_list.waitForTransform(frame, self.head_center.header.frame_id, 
                                      cur_time, rospy.Duration(3))
        self.head_center.header.stamp = cur_time
        return PoseConverter.to_homo_mat(
                    self.tf_list.transformPose(frame, self.head_center))

    def _run_traj(self, traj, blocking=True):
        self.start_pub.publish(
                PoseConverter.to_pose_stamped_msg("/torso_lift_link", traj[0]))
        self.end_pub.publish(
                PoseConverter.to_pose_stamped_msg("/torso_lift_link", traj[-1]))
        return self.execute_cart_traj(self.arm, traj, self.time_step, blocking=blocking)
