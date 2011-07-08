import numpy as np, math

import roslib; roslib.load_manifest('equilibrium_point_control')
import rospy

from equilibrium_point_control.controllers import PIDController
from equilibrium_point_control.epc import EPGenerator, EPC, EPStopConditions

class PIDJCFactory:
    def __init__(self, arm, err_goal_thresh, p_list, i_list, d_list, i_max_list, rate=10.):
        self.jc_arm = PR2ArmJointTrajectory(arm, PR2ArmKinematics(arm))
        self.err_goal_thresh = err_goal_thresh
        self.p_list = p_list
        self.i_list = i_list
        self.d_list = d_list
        self.i_max_list = i_max_list
        self.rate = rate

    def create_pid_jc(self.q_goal):
        return PIDJointControl(self.jc_arm, q_goal, self.err_goal_thresh,
                               self.p_list, self.i_list, self.d_list, self.i_max_list, self.rate)

class PIDJointControl(EPGenerator):
    def __init__(self, joint_control_arm, q_goal, err_goal_thresh, 
                 p_list, i_list, d_list, i_max_list, rate=10.):
        self.jc_arm = joint_control_arm
        self.q_goal = self.jc_arm.wrap_angles(q_goal)
        self.err_goal_thresh = np.array(err_goal_thresh)
        self.controllers = [PIDController(k_p, k_i, k_d, i_max, rate, 'pid_joint_%d' % i)
                            for k_p, k_i, k_d, i_max, i in 
                            zip(p_list, i_list, d_list, i_max_list, range(len(p_list)))]
        self.rate = rate

    def generate_ep(self):
        q_cur = self.jc_arm.get_joint_angles(True)
        q_err = self.q_goal - q_cur
        q_control = np.array([pidc.update_state(q_err_i) for pidc, q_err_i in 
                              zip(self.controllers, q_err)])
        jep_cur = np.array(self.jc_arm.get_ep())
        return EPStopConditions.CONTINUE, jep_cur + q_control

    def control_ep(self, ep):
        self.set_ep(ep, self.rate)

    def clamp_ep(self, ep):
        # TODO Add joint limits?
        return ep
        
    def terminate_check(self):
        q_cur = self.jc_arm.get_joint_angles(True)
        q_err = self.q_goal - q_cur
        if np.all(np.fabs(q_err) < self.err_goal_thresh):
            return EPStopConditions.SUCCESSFUL
        else:
            return EPStopConditions.CONTINUE
        

