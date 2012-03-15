import numpy as np, math

import roslib; roslib.load_manifest('hrl_pr2_arms')
import rospy

from equilibrium_point_control.controllers import PIDController
from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions
from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory
from hrl_pr2_arms.pr2_arm_kinematics import PR2ArmKinematics

class PIDJCFactory:
    def __init__(self, arm, err_goal_thresh, err_coll_thresh,
                 p_list, i_list, d_list, i_max_list, 
                 duration=10., time_step=0.1):
        self.jc_arm = PR2ArmJointTrajectory(arm, PR2ArmKinematics(arm))
        self.jc_arm.reset_ep()
        self.err_goal_thresh = err_goal_thresh
        self.err_coll_thresh = err_coll_thresh
        self.p_list = p_list
        self.i_list = i_list
        self.d_list = d_list
        self.i_max_list = i_max_list
        self.duration = duration
        self.time_step = time_step

    def create_pid_jc(self, q_goal):
        return PIDJointControl(self.jc_arm, q_goal, self.err_goal_thresh, self.err_coll_thresh,
                               self.p_list, self.i_list, self.d_list, self.i_max_list, 
                               self.duration, self.time_step)

class PIDJointControl(EPGenerator):
    def __init__(self, joint_control_arm, q_goal, err_goal_thresh, err_coll_thresh,
                 p_list, i_list, d_list, i_max_list, duration=10., time_step=0.1):
        self.jc_arm = joint_control_arm
        self.q_goal = self.jc_arm.wrap_angles(q_goal)
        self.err_goal_thresh = np.array(err_goal_thresh)
        self.err_coll_thresh = np.array(err_coll_thresh)
        self.controllers = [PIDController(k_p, k_i, k_d, i_max, 1./time_step, 'pid_joint_%d' % i)
                            for k_p, k_i, k_d, i_max, i in 
                            zip(p_list, i_list, d_list, i_max_list, range(len(p_list)))]
        self.time_step = time_step
        self.num_steps = round(duration / time_step)
        self.trajectory = self.jc_arm.interpolate_ep(self.jc_arm.get_ep(), self.q_goal, self.num_steps)
        self.step_ind = 0
        self.q_err = np.zeros(self.jc_arm.kinematics.n_jts)

    def generate_ep(self):
        q_cur = self.jc_arm.get_joint_angles(True)
        if self.step_ind < self.num_steps:
            q_cur_goal = self.trajectory[self.step_ind]
            self.step_ind += 1
        else:
            q_cur_goal = self.q_goal
        self.q_err = q_cur_goal - q_cur
        self.q_err[np.where(np.fabs(self.q_err) < self.err_goal_thresh * 0.6)] = 0.0
        q_control = np.array([pidc.update_state(q_err_i) for pidc, q_err_i in 
                              zip(self.controllers, self.q_err)])
        self.q_control = q_control
        jep_cur = np.array(self.jc_arm.get_ep())
        ep = jep_cur + q_control
        return EPStopConditions.CONTINUE, ep

    def control_ep(self, ep):
        wrapped_ep = self.jc_arm.wrap_angles(ep)
        self.jc_arm.set_ep(wrapped_ep, self.time_step*1.5)

    def clamp_ep(self, ep):
        # TODO Add joint limits?
        return ep
        
    def terminate_check(self):
        ep_q_diff = np.array(self.jc_arm.get_ep()) - self.jc_arm.get_joint_angles(True)
        if np.any(np.fabs(ep_q_diff) > self.err_coll_thresh):
            return EPStopConditions.COLLISION

        if self.step_ind < self.num_steps:
            return EPStopConditions.CONTINUE
        else:
            if (self.step_ind == self.num_steps and 
                np.all(np.fabs(self.q_err) < self.err_goal_thresh)):
                return EPStopConditions.SUCCESSFUL
            else:
                return EPStopConditions.CONTINUE
        

