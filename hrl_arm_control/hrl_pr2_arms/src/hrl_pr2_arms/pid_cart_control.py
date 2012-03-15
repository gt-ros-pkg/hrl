import numpy as np, math

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

from equilibrium_point_control.controllers import PIDController
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions
from equilibrium_point_control.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import PR2ArmCartesianBase
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics

class PIDCartFactory:
    def __init__(self, arm, err_goal_thresh, err_coll_thresh,
                 p_list, i_list, d_list, i_max_list, 
                 duration=10., time_step=0.1, debug=False):
        self.cart_arm = PR2ArmCartesianBase(arm, PR2ArmKinematics(arm))
        self.cart_arm.reset_ep()
        self.err_goal_thresh = err_goal_thresh
        self.err_coll_thresh = err_coll_thresh
        self.p_list = p_list
        self.i_list = i_list
        self.d_list = d_list
        self.i_max_list = i_max_list
        self.duration = duration
        self.time_step = time_step
        self.debug = debug

    def create_controller(self, cart_goal):
        return PIDCartesianControl(self.cart_arm, cart_goal, self.err_goal_thresh, self.err_coll_thresh,
                               self.p_list, self.i_list, self.d_list, self.i_max_list, 
                               self.duration, self.time_step, self.debug)

class PIDCartesianControl(EPGenerator):
    def __init__(self, cart_control_arm, cart_goal, err_goal_thresh, err_coll_thresh,
                 p_list, i_list, d_list, i_max_list, duration=10., time_step=0.1, debug=False):
        self.cart_arm = cart_control_arm
        self.cart_goal = cart_goal
        self.err_goal_thresh = np.array(err_goal_thresh)
        self.err_coll_thresh = np.array(err_coll_thresh)
        self.controllers = [PIDController(k_p, k_i, k_d, i_max, 1./time_step, 'pid_joint_%d' % i)
                            for k_p, k_i, k_d, i_max, i in 
                            zip(p_list, i_list, d_list, i_max_list, range(len(p_list)))]
        self.time_step = time_step
        self.num_steps = round(duration / time_step)
        self.trajectory = self.cart_arm.interpolate_ep(self.cart_arm.get_ep(), self.cart_goal, self.num_steps)
        self.step_ind = 0
        self.cart_err = np.mat(np.zeros(6)).T
        self.debug = debug
        self.ep_pub = rospy.Publisher("pid_cartesian_control_ep", PoseStamped)
        self.act_pub = rospy.Publisher("pid_cartesian_control_actual", PoseStamped)
        self.goal_pub = rospy.Publisher("pid_cartesian_control_goal", PoseStamped)

    def generate_ep(self):
        cart_cur = self.cart_arm.get_end_effector_pose()
        if self.step_ind < self.num_steps:
            self.cart_cur_goal = self.trajectory[self.step_ind]
            self.step_ind += 1
        else:
            self.cart_cur_goal = self.cart_goal
        self.cart_err = self.cart_arm.ep_error(cart_cur, self.cart_cur_goal)
#self.cart_err[np.where(np.fabs(self.cart_err) < self.err_goal_thresh * 0.6)] = 0.0
        cart_control = np.array([pidc.update_state(cart_err_i) for pidc, cart_err_i in 
                                zip(self.controllers, self.cart_err)])
        cep_pos_cur, cep_rot_cur = self.cart_arm.get_ep()
        cep_pos_new = cep_pos_cur - cart_control[:3,0]
        cart_rot_control = np.mat(tf_trans.euler_matrix(cart_control[3,0], cart_control[4,0], 
                                                 cart_control[5,0]))[:3,:3]
        cep_rot_new = cep_rot_cur * cart_rot_control.T
        ep_new = (cep_pos_new, cep_rot_new)
        if self.debug:
            print "="*50
            print "cart_cur", cart_cur
            print "-"*50
            print "cur goal", self.cart_cur_goal
            print "-"*50
            print "err", self.cart_err
            print "-"*50
            print "cart_control", cart_control
        return EPStopConditions.CONTINUE, ep_new

    def control_ep(self, ep):
        cart_cur = self.cart_arm.get_end_effector_pose()
        self.ep_pub.publish(PoseConverter.to_pose_stamped_msg('torso_lift_link', ep))
        self.act_pub.publish(PoseConverter.to_pose_stamped_msg('torso_lift_link', cart_cur))
        self.goal_pub.publish(PoseConverter.to_pose_stamped_msg('torso_lift_link', self.cart_cur_goal))
        if self.debug:
            print "-"*50
            print "ep:", ep
        self.cart_arm.set_ep(ep, self.time_step*1.5)

    def clamp_ep(self, ep):
        # TODO Add joint limits?
        return ep
        
    def terminate_check(self):
        cart_cur = self.cart_arm.get_end_effector_pose()
        ep_actual_diff = self.cart_arm.ep_error(self.cart_arm.get_ep(), cart_cur)
        if self.debug:
            print "ep_actual_diff", ep_actual_diff, self.err_coll_thresh
        if np.any(np.fabs(ep_actual_diff.T.A[0]) > self.err_coll_thresh):
            return EPStopConditions.COLLISION

        if self.step_ind < self.num_steps:
            return EPStopConditions.CONTINUE
        else:
            if (self.step_ind == self.num_steps and 
                np.all(np.fabs(self.cart_err.T.A[0]) < self.err_goal_thresh)):
                return EPStopConditions.SUCCESSFUL
            else:
                return EPStopConditions.CONTINUE
        

