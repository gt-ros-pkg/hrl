import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
import kinematics_msgs.srv as ks
import hrl_lib.tf_utils as tfu
import tf

class PR2ArmKinematics:

    def __init__(self, arm, listener):
        self.tflistener = listener
        rospy.loginfo('PR2ArmKinematics: waiting for services for %s arm ' % arm)
        if arm == 'right':
            self.tflistener.waitForTransform('r_gripper_tool_frame', 'r_wrist_roll_link', rospy.Time(), rospy.Duration(10))
        else:
            self.tflistener.waitForTransform('l_gripper_tool_frame', 'l_wrist_roll_link', rospy.Time(), rospy.Duration(10))

        rospy.wait_for_service('pr2_' + arm + '_arm_kinematics/get_fk_solver_info')
        rospy.wait_for_service('pr2_' + arm + '_arm_kinematics/get_fk')
        rospy.loginfo('PR2ArmKinematics: Kinematics services online.')

        self._fk_info = rospy.ServiceProxy('pr2_' + arm + '_arm_kinematics/get_fk_solver_info', ks.GetKinematicSolverInfo )
        self._fk      = rospy.ServiceProxy('pr2_' + arm + '_arm_kinematics/get_fk',             ks.GetPositionFK, persistent=True)
        self.fk_info_resp = self._fk_info()
        self.joint_names = self.fk_info_resp.kinematic_solver_info.joint_names
        print 'PR2ArmKinematics: number of joints', len(self.joint_names)

    def fk(self, frame, sol_link, sol_frame, joint_poses_list):
        self.tflistener.waitForTransform(sol_frame, sol_link, rospy.Time(), rospy.Duration(10))

        fk_req = ks.GetPositionFKRequest()
        fk_req.header.frame_id = frame
        fk_req.fk_link_names = [sol_link]
        fk_req.robot_state.joint_state.name = self.fk_info_resp.kinematic_solver_info.joint_names
        fk_req.robot_state.joint_state.position = joint_poses_list 
        fk_resp = self._fk(fk_req)
        
        wr_T_rtip = tfu.transform(sol_link, sol_frame, self.tflistener)
        #rtip_T_wr = tfu.transform(sol_link, sol_frame, self.tflistener)
        #print sol_link, sol_frame, rtip_T_wr
        #right_wr_sol_frame = tfu.pose_as_matrix(fk_resp.pose_stamped[0].pose)
        solframe_T_wr = tfu.pose_as_matrix(fk_resp.pose_stamped[0].pose)
        rtip_pose = solframe_T_wr * wr_T_rtip
        #print tfu.matrix_as_tf(rtip_pose)
        return rtip_pose

class PR2Kinematics:

    def __init__(self, tflistener=None):
        if tflistener == None:
            self.tflistener = tf.TransformListener()
        else:
            self.tflistener = tflistener

        self.left = PR2ArmKinematics('left', self.tflistener)
        self.right = PR2ArmKinematics('right', self.tflistener)
