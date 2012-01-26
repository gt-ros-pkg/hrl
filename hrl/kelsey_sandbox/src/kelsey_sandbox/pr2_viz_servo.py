#! /usr/bin/python

import numpy as np
from collections import deque

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('ar_pose')
roslib.load_manifest('visualization_msgs')

import rospy
import tf
import tf.transformations as tf_trans
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_generic_arms.controllers import PIDController
from ar_pose.msg import ARMarker
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class ServoKalmanFilter(object):
    # TODO tune these parameters properly
    def __init__(self, delta_t, sigma_z=0.02*0.001, P_init=[0.0, 0.0], sigma_a=0.02):
        self.P_init = P_init
        self.delta_t = delta_t
        self.F = np.mat([[1, delta_t], [0, 1]])
        self.Q = np.mat([[delta_t**4/4, delta_t**3/2], [delta_t**3/2, delta_t**2]]) * sigma_a**2
        self.H = np.mat([1, 0])
        self.R = np.mat([sigma_z])
        self.x_cur = None

        self.resid_q_len = int(1./delta_t)
        self.resid_sigma_reject = 3.
        self.min_reject = 0.1
        self.resid_queue = deque()
        
        self.unreli_q_len = 2 * int(1./delta_t)
        self.unreli_queue = deque()
        self.unreli_weights = np.linspace(2, 0, self.unreli_q_len)

    def update(self, z_obs, new_obs=True):
        is_unreli = False
        if new_obs:
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

            # check residual to be consistent with recent residuals
            if (len(self.resid_queue) == self.resid_q_len and
                np.fabs(y_resi) > max(self.min_reject, 
                                      self.resid_sigma_reject * np.std(self.resid_queue))):
                # we have determined this observation to be unreliable
                is_unreli = True

            else:
                self.x_cur = x_pred + K_gain * y_resi # update state estimate
                self.P_cur = (np.mat(np.eye(2)) - K_gain * self.H) * P_pred

            # record residual
            if len(self.resid_queue) == self.resid_q_len:
                self.resid_queue.popleft()
            self.resid_queue.append(y_resi)
        else:
            is_unreli = True

        # record is_unreli
        if len(self.unreli_queue) == self.unreli_q_len:
            self.unreli_queue.popleft()
        self.unreli_queue.append(is_unreli)

        # find the unreli level
        # this value [0, 1] is a record of the values which have been determined to be unreli
        # in the pase few seconds filtered with linear weights
        # a value of 0 means there are no unreliable estimates, 
        # 1 means there is no reliable state estimate
        if len(self.unreli_queue) == self.unreli_q_len:
            unreli_level = np.sum(self.unreli_weights * self.unreli_queue) / self.unreli_q_len
        else:
            unreli_level = 0.

        return self.x_cur, self.P_cur, unreli_level

def homo_mat_from_2d(x, y, rot):
    mat2d = np.mat(tf_trans.euler_matrix(0, 0, rot))
    mat2d[0,3] = x
    mat2d[1,3] = y
    return mat2d

def homo_mat_to_2d(mat):
    rot = tf_trans.euler_from_matrix(mat)[2]
    return mat[0,3], mat[1,3], rot

def create_base_marker(pose, id, color):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "ar_servo"
    marker.id = id
    marker.pose = PoseConverter.to_pose_msg(pose)
    marker.color = ColorRGBA(*(color + (1.0,)))
    marker.scale.x = 0.7; marker.scale.y = 0.7; marker.scale.z = 0.2
    return marker

class PR2VisualServoAR(object):
    def __init__(self, ar_topic):
        self.ar_sub = rospy.Subscriber(ar_topic, ARMarker, self.ar_sub)
        self.mkr_pub = rospy.Publisher("visualization_marker", Marker)

        self.cur_ar_pose = None
        self.tf_list = tf.TransformListener()
        self.ar_pose_updated = False
        self.base_pub = rospy.Publisher("/base_controller/command", Twist)
        self.preempt_requested = False

    def ar_sub(self, msg):
        cur_ar_ps = PoseConverter.to_pose_stamped_msg(msg.header.frame_id, msg.pose.pose)
        cur_ar_ps.header.stamp = msg.header.stamp
        try:
            cur_ar_in_base = self.tf_list.transformPose("/base_link", cur_ar_ps)
        except tf.Exception as e:
            return
        self.cur_ar_pose = PoseConverter.to_homo_mat(cur_ar_in_base)
        self.ar_pose_updated = True

    def request_preempt(self):
        self.preempt_requested = True

    def save_ar_goal(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cur_ar_pose is not None:
                ar_goal = homo_mat_to_2d(self.cur_ar_pose)
                print ar_goal
            r.sleep()

    def test_move(self):
        rospy.sleep(0)
        base_twist = Twist()
        base_twist.linear.x = 0.0 #x_ctrl
        base_twist.linear.y = 0.0 #y_ctrl
        base_twist.angular.z = 0.4 #r_ctrl
        r = rospy.Rate(20.)
        while not rospy.is_shutdown():
            self.base_pub.publish(base_twist)
            r.sleep()
        self.base_pub.publish(Twist())

    def find_ar_tag(self, timeout=None):
        rate = 20.
        ar_2d_q_len = 10
        sigma_thresh = [0.005, 0.001, 0.01]
        no_mean_thresh = 0.5
        r = rospy.Rate(rate)
        ar_2d_queue = deque()
        new_obs_queue = deque()
        start_time = rospy.get_time()
        while True:
            if timeout is not None and rospy.get_time() - start_time > timeout:
                rospy.logwarn("[pr2_viz_servo] find_ar_tag timed out, current ar_sigma: " + 
                              str(np.std(ar_2d_queue, 0)) +
                              " sigma_thresh: " +
                              str(sigma_thresh))
                return None, 'timeout'
            if self.preempt_requested:
                self.preempt_requested = False
                return None, 'preempted'
            if rospy.is_shutdown():
                return None, 'aborted'

            if self.cur_ar_pose is not None:
                # make sure we have a new observation
                new_obs = self.ar_pose_updated
                self.ar_pose_updated = False

                if new_obs:
                    if len(ar_2d_queue) == ar_2d_q_len:
                        ar_2d_queue.popleft()
                    ar_2d = homo_mat_to_2d(self.cur_ar_pose)
                    ar_2d_queue.append(ar_2d)

                if len(new_obs_queue) == ar_2d_q_len:
                    new_obs_queue.popleft()
                new_obs_queue.append(new_obs)

                # see if we have a low variance tag
                if len(ar_2d_queue) == ar_2d_q_len:
                    ar_sigma = np.std(ar_2d_queue, 0)
                    no_mean = np.mean(new_obs_queue, 0)
                    print ar_sigma, no_mean
                    if np.all(ar_sigma < sigma_thresh) and no_mean >= no_mean_thresh:
                        return np.mean(ar_2d_queue, 0), 'found_tag'
            r.sleep()

    def servo_to_tag(self, pose_goal, goal_error=[0.03, 0.03, 0.1], initial_ar_pose=None):
        lost_tag_thresh = 1.0 #0.4

        # TODO REMOVE
        err_pub = rospy.Publisher("servo_err", Float32MultiArray)
        if False:
            self.test_move()
            return "aborted"
        #######################

        goal_ar_pose = homo_mat_from_2d(*pose_goal)
        rate = 20.
        kf_x = ServoKalmanFilter(delta_t=1./rate)
        kf_y = ServoKalmanFilter(delta_t=1./rate)
        kf_r = ServoKalmanFilter(delta_t=1./rate)
        if initial_ar_pose is not None:
            ar_err = homo_mat_to_2d(homo_mat_from_2d(*initial_ar_pose) * goal_ar_pose**-1)
            kf_x.update(ar_err[0])
            kf_y.update(ar_err[1])
            kf_r.update(ar_err[2])
            
        pid_x = PIDController(k_p=0.5, rate=rate, saturation=0.06)
        pid_y = PIDController(k_p=0.5, rate=rate, saturation=0.06)
        pid_r = PIDController(k_p=0.5, rate=rate, saturation=0.08)
        r = rospy.Rate(rate)
        while True:
            if rospy.is_shutdown():
                self.base_pub.publish(Twist())
                return 'aborted'
            if self.preempt_requested:
                self.preempt_requested = False
                self.base_pub.publish(Twist())
                return 'preempted'
            goal_mkr = create_base_marker(goal_ar_pose, 0, (0., 1., 0.))
            self.mkr_pub.publish(goal_mkr)
            if self.cur_ar_pose is not None:
                # make sure we have a new observation
                new_obs = self.ar_pose_updated
                self.ar_pose_updated = False

                # find the error between the AR tag and goal pose
                cur_ar_pose_2d = homo_mat_from_2d(*homo_mat_to_2d(self.cur_ar_pose))
                ar_mkr = create_base_marker(cur_ar_pose_2d, 1, (1., 0., 0.))
                self.mkr_pub.publish(ar_mkr)
                ar_err = homo_mat_to_2d(cur_ar_pose_2d * goal_ar_pose**-1)

                # filter this error using a Kalman filter
                x_filt_err, x_filt_cov, x_unreli = kf_x.update(ar_err[0], new_obs=new_obs)
                y_filt_err, y_filt_cov, y_unreli = kf_y.update(ar_err[1], new_obs=new_obs)
                r_filt_err, r_filt_cov, r_unreli = kf_r.update(ar_err[2], new_obs=new_obs)

                if np.any(np.array([x_unreli, y_unreli, r_unreli]) > [lost_tag_thresh]*3):
                    self.base_pub.publish(Twist())
                    return 'lost_tag'

                print "Noise:", x_unreli, y_unreli, r_unreli
                # TODO REMOVE
                ma = Float32MultiArray()
                ma.data = [x_filt_err[0,0], x_filt_err[1,0], ar_err[0], x_unreli]
                err_pub.publish(ma)

                print "xerr"
                print x_filt_err
                print x_filt_cov
                print "Cov", x_filt_cov[0,0], y_filt_cov[0,0], r_filt_cov[0,0]
                x_ctrl = pid_x.update_state(x_filt_err[0,0])
                y_ctrl = pid_y.update_state(y_filt_err[0,0])
                r_ctrl = pid_r.update_state(r_filt_err[0,0])
                base_twist = Twist()
                base_twist.linear.x = x_ctrl
                base_twist.linear.y = y_ctrl
                base_twist.angular.z = r_ctrl
                cur_filt_err = np.array([x_filt_err[0,0], y_filt_err[0,0], r_filt_err[0,0]])
                print "err", ar_err
                print "Err filt", cur_filt_err 
                print "Twist:", base_twist
                if np.all(np.fabs(cur_filt_err) < goal_error):
                    self.base_pub.publish(Twist())
                    return 'succeeded'

                self.base_pub.publish(base_twist)

            r.sleep()

def main():
    rospy.init_node("pr2_viz_servo")
    viz_servo = PR2VisualServoAR("/pr2_test_ar_pose_marker")
    if False:
        viz_servo.save_ar_goal()
    elif False:
        viz_servo.servo_to_tag((0.55761498778404717, -0.28816809195738824, 1.5722787397126308))
    else:
        print viz_servo.find_ar_tag(5)

if __name__ == "__main__":
    main()
