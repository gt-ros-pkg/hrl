#! /usr/bin/python

from khpr2import import *

def main():
    rospy.init_node("test_vel_move")

    possible_controllers = ['%s_joint_controller_low']
    ctrl_switcher = ControllerSwitcher()
    if True:
        ctrl_switcher.carefree_switch('r', '%s_arm_controller', 
                           '$(find hrl_pr2_arms)/params/joint_traj_params_electric.yaml')
        rospy.sleep(0.5)
        ctrl_switcher.carefree_switch('r', '%s_joint_controller_low', 
                           '$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml')
    r_arm_js = create_pr2_arm('r', PR2ArmJointTrajectory, controller_name='%s_joint_controller_low')
    q = [-0.34781704,  0.27341079, -1.75392154, -2.08626393, -3.43756314, -1.82146607, -1.85187734]
    r_arm_js.set_ep(q, 3) 
    rospy.sleep(6)
    ctrl_switcher.carefree_switch('r', '%s_cart',
                                  '$(find kelsey_sandbox)/params/j_transpose_low_rfh.yaml')

    r_arm = create_pr2_arm('r', PR2ArmCartesianPostureBase)
    r_arm.set_posture()
    rospy.sleep(0.2)
    pos_i_des, rot_i_des = r_arm.get_ep()
    pos_i_act, rot_i_act = r_arm.get_end_effector_pose()
    pos_err = pos_i_act - pos_i_des
    rot_err = rot_i_act * rot_i_des.T
    time_start = rospy.get_time()
#pos_cur = pos_start.copy()
#rot_cur = rot_start.copy()
#pos_end = pos_start - np.mat([0.15, 0, 0]).T
#pos_lead = np.mat([0, 0, 0]).T
    test_pub = rospy.Publisher("/test", Float64MultiArray)
    xd_des = 0.015
    xd_integ = 1.0
    vels = deque([0]*40)
    while not rospy.is_shutdown():
        pos, rot = r_arm.get_end_effector_pose()
        xd_act = r_arm.get_controller_state()['xd_act'][0]
        vels.append(xd_act)
        vels.popleft()
        xd_act_filt = np.mean(vels)
        xd_integ += (xd_des - xd_act_filt)
#k_p = 0.14
#k_i = 0.01
#k_c = 0.02
        k_p = 0.014
        k_i = 0.0015
        k_c = 0.003
        xd_ctrl = k_p * (xd_des - xd_act_filt) + k_i * xd_integ + np.sign(xd_des) * k_c
        xd_ctrl_final = np.clip(xd_ctrl, -0.30, 0.30)
        msg = Float64MultiArray()
        msg.data = [xd_ctrl, k_p * (xd_des - xd_act_filt), k_i * xd_integ, np.sign(xd_des) * k_c]
        test_pub.publish(msg)
        print "ctrl", xd_ctrl, "err", (xd_des - xd_act), "integ", xd_integ, "final", xd_ctrl_final
        pos_des = pos + np.mat([xd_ctrl_final, 0, 0]).T - pos_err
        rot_des = rot * rot_err.T
        pos_des[1:,0] = pos_i_des[1:,0]
        rot_des = rot_i_des

        r_arm.set_ep((pos_des, rot_des), 1)
        rospy.sleep(0.01)
    r_arm.set_ep(r_arm.get_ep(), 1)
    pos_f_act, rot_f_act = r_arm.get_end_effector_pose()
    time_end = rospy.get_time()
    print (pos_f_act[0] - pos_i_act[0]) / (time_end - time_start)

if __name__ == "__main__":
    main()
