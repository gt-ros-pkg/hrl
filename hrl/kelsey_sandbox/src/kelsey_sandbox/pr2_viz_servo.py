#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('ar_pose')

import rospy
import tf
import tf.transformations as tf_trans

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_generic_arms.controllers import PIDController
from ar_pose.msg import ARMarker

class ServoKalmanFilter(object):
    def __init__(self, delta_t, sigma_z=0.02, P_init=[0.1, 0.05], sigma_a=0.02):
        self.P_init = P_init
        self.delta_t = delta_t
        self.F = np.mat([[1, delta_t], [0, 1]])
        self.Q = np.mat([[delta_t**4/4, delta_t**3/2], [delta_t**3/2, delta_t**2]]) * sigma_a**2
        self.H = np.mat([1, 0])
        self.R = np.mat([sigma_z])
        self.x_cur = None

    def update(self, z_obs):
        if self.x_cur is None:
            self.x_cur = np.mat([z_obs, 0]).T
            self.P_cur = np.mat(np.diag(self.P_init))
        # predict
        x_pred = self.F * self.x_cur # predicted state
        P_pred = self.F * self.P_cur * self.F.T + self.Q # predicted covariance

        # update
        y_resi = z_obs - self.H * x_pred # measurement residual
        S_resi = self.H * P_pred * self.H.T + self.R # residual covariance
        K_gain = P_pred * self.H.T * S_resi**-1 # Kalman gain
        self.x_cur = x_pred + K_gain * y_resi # update state estimate
        self.P_cur = (np.mat(np.eye(2)) - K_gain * self.H) * P_pred

        return self.x_cur, self.P_cur

def homo_mat_from_2d(x, y, rot):
    mat2d = np.mat(tf_trans.euler_matrix(0, 0, rot))
    mat2d[0,3] = x
    mat2d[1,3] = y
    return mat2d

def homo_mat_to_2d(mat):
    rot = tf_trans.euler_from_matrix(mat)[2]
    return mat[0,3], mat[1,3], rot

class PR2VisualServoAR(object):
    def __init__(self, ar_topic="/pr2_test_ar_pose_marker"):
        self.ar_sub = rospy.Subscriber(ar_topic, ARMarker, self.ar_sub)

        self.cur_ar_pose = None
        self.tf_list = tf.TransformListener()

    def ar_sub(self, msg):
        cur_ar_ps = PoseConverter.to_pose_stamped_msg(msg.header.frame_id, msg.pose.pose)
        cur_ar_ps.header.stamp = msg.header.stamp
        try:
            cur_ar_in_base = self.tf_list.transformPose("/base_link", cur_ar_ps)
        except tf.Exception as e:
            return
        self.cur_ar_pose = PoseConverter.to_homo_mat(cur_ar_in_base)

    def save_ar_goal(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cur_ar_pose is not None:
                ar_goal = homo_mat_to_2d(self.cur_ar_pose)
                print ar_goal
            r.sleep()

    def servoing_loop(self, pose_goal):
        goal_ar_pose = homo_mat_from_2d(*pose_goal)
        rate = 20.
        kf_x = ServoKalmanFilter(delta_t=1./rate)
        kf_y = ServoKalmanFilter(delta_t=1./rate)
        kf_r = ServoKalmanFilter(delta_t=1./rate)
        pid_x = PIDController(k_p=10., k_i=10., k_d=10., i_max=10., rate=10., saturation=10., name=None)
        r = rospy.Rate(rate)
        while not rospy.is_shutdown():
            if self.cur_ar_pose is not None:
                cur_ar_pose_2d = homo_mat_from_2d(*homo_mat_to_2d(self.cur_ar_pose))
                ar_err = homo_mat_to_2d(goal_ar_pose**-1 * cur_ar_pose_2d)
                print ar_err
                x_state, x_cov = kf_x.update(ar_err[0])
                y_state, y_cov = kf_y.update(ar_err[1])
                r_state, r_cov = kf_r.update(ar_err[2])
            r.sleep()



def main():
    rospy.init_node("pr2_viz_servo")
    viz_servo = PR2VisualServoAR()
    if False:
        viz_servo.save_ar_goal()
    else:
        viz_servo.servoing_loop((1.1380248332031384, -0.35049196969344076, 1.4807036493249148))

if __name__ == "__main__":
    main()
