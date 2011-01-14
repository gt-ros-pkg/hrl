import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
import kinematics_msgs.srv as ks
import hrl_lib.tf_utils as tfu
import tf
import numpy as np
import pdb

class PR2ArmKinematics:

    def __init__(self, arm, listener):
        self.tflistener = listener
        rospy.loginfo('PR2ArmKinematics: waiting for services for %s arm ' % arm)
        if arm == 'right':
            self.tflistener.waitForTransform('r_gripper_tool_frame', 'r_wrist_roll_link', rospy.Time(), rospy.Duration(10))
        else:
            self.tflistener.waitForTransform('l_gripper_tool_frame', 'l_wrist_roll_link', rospy.Time(), rospy.Duration(10))

        # Forward kinematics
        rospy.wait_for_service('pr2_' + arm + '_arm_kinematics/get_fk_solver_info')
        rospy.wait_for_service('pr2_' + arm + '_arm_kinematics/get_fk')
        rospy.loginfo('PR2ArmKinematics: forward kinematics services online.')

        self._fk_info = rospy.ServiceProxy('pr2_' + arm + '_arm_kinematics/get_fk_solver_info', ks.GetKinematicSolverInfo )
        self._fk      = rospy.ServiceProxy('pr2_' + arm + '_arm_kinematics/get_fk',             ks.GetPositionFK, persistent=True)
        self.fk_info_resp = self._fk_info()
        self.joint_names = self.fk_info_resp.kinematic_solver_info.joint_names
        print 'PR2ArmKinematics: number of joints', len(self.joint_names)
        
        # Inverse kinematics
        rospy.wait_for_service("pr2_" + arm + "_arm_kinematics/get_ik_solver_info")
        rospy.wait_for_service("pr2_" + arm + "_arm_kinematics/get_ik")
        rospy.loginfo('PR2ArmKinematics: inverse kinematics services online.')
        self._ik_info = rospy.ServiceProxy('pr2_' + arm +'_arm_kinematics/get_ik_solver_info', ks.GetKinematicSolverInfo)
        self._ik      = rospy.ServiceProxy('pr2_' + arm +'_arm_kinematics/get_ik', ks.GetPositionIK, persistent=True)
        self.ik_info_resp = self._ik_info()
        self.ik_joint_names = self.ik_info_resp.kinematic_solver_info.joint_names
        if arm == 'left':
            self.ik_frame = 'l_wrist_roll_link'
            self.tool_frame = 'l_gripper_tool_frame'
        else:
            self.ik_frame = 'r_wrist_roll_link'
            self.tool_frame = 'r_gripper_tool_frame'

    ##
    # Inverse kinematics
    # @param cart_pose a 4x4 SE(3) pose
    # @param frame_of_pose frame cart_pose is given in, if None we assume that self.tool_frame is being used
    # @param seed starting solution for IK solver (list of floats or column np.matrix of floats)
    def ik(self, cart_pose, frame_of_pose='torso_lift_link', seed=None):
        #if frame_of_pose == self.tool_frame or frame_of_pose == None:

        self.tflistener.waitForTransform(self.ik_frame, self.tool_frame, rospy.Time(), rospy.Duration(10))
        #wr_T_toolframe = tfu.transform(sol_link, self.tool_frame, self.tflistener)
        #solframe_T_wr * wr_T_toolframe
        #print 'undoing'
        toolframe_T_ikframe = tfu.transform(self.tool_frame, self.ik_frame, self.tflistener)
        cart_pose = cart_pose * toolframe_T_ikframe 
        #frame_of_pose = self.tool_frame

        trans, rot = tfu.matrix_as_tf(cart_pose)

        ik_req = ks.GetPositionIKRequest()
        ik_req.timeout = rospy.Duration(5.0)
        ik_req.ik_request.ik_link_name = self.ik_frame

        #set pose
        ik_req.ik_request.pose_stamped.header.frame_id = frame_of_pose
        ik_req.ik_request.pose_stamped.pose.position.x = trans[0]#cart_pose[0][0,0]
        ik_req.ik_request.pose_stamped.pose.position.y = trans[1]#cart_pose[0][1,0]
        ik_req.ik_request.pose_stamped.pose.position.z = trans[2]#cart_pose[0][2,0]

        ik_req.ik_request.pose_stamped.pose.orientation.x = rot[0]#cart_pose[1][0,0];
        ik_req.ik_request.pose_stamped.pose.orientation.y = rot[1]#cart_pose[1][1,0];
        ik_req.ik_request.pose_stamped.pose.orientation.z = rot[2]#cart_pose[1][2,0];
        ik_req.ik_request.pose_stamped.pose.orientation.w = rot[3]#cart_pose[1][3,0];

        #seed solver
        ik_req.ik_request.ik_seed_state.joint_state.name = self.ik_joint_names
        if seed == None:
            p = []
            for i in range(len(self.ik_joint_names)):
                minp = self.ik_info_resp.kinematic_solver_info.limits[i].min_position
                maxp = self.ik_info_resp.kinematic_solver_info.limits[i].max_position
                p.append((minp + maxp) / 2.0)
            ik_req.ik_request.ik_seed_state.joint_state.position = p
        else:
            if seed.__class__ == np.matrix:
                seed = seed.T.A1.tolist()
            ik_req.ik_request.ik_seed_state.joint_state.position = seed

        response = self._ik(ik_req)
        if response.error_code.val == response.error_code.SUCCESS:
            #print 'success'
            return np.matrix(response.solution.joint_state.position).T
        else:
            #print 'fail', response.__class__, response
            print response
            return None

    ##
    # Forward Kinematics
    # @param joint_poses_mat nx1 matrix of joint positions
    # @param frame frame to give solution in
    # @param sol_link link to solve FK for
    # @param use_tool_frame FK doesn't account for length of tool frame (PR2 gripper), 
    #                       if sol_link is the wrist link then will return the
    #                       gripper's FK.
    # @return a 4x4 SE(3) matrix
    def fk(self, joint_poses_mat, frame='torso_lift_link', sol_link=None, use_tool_frame=True):
        if sol_link == None:
            sol_link = self.ik_frame

        fk_req = ks.GetPositionFKRequest()
        fk_req.header.frame_id = frame
        fk_req.fk_link_names = [sol_link]
        fk_req.robot_state.joint_state.name = self.fk_info_resp.kinematic_solver_info.joint_names
        fk_req.robot_state.joint_state.position = joint_poses_mat.T.A1.tolist() 
        fk_resp = self._fk(fk_req)
        
        solframe_T_wr = tfu.pose_as_matrix(fk_resp.pose_stamped[0].pose)
        if not use_tool_frame:
            return solframe_T_wr
        else:
            #print 'redoing'
            self.tflistener.waitForTransform(self.tool_frame, sol_link, rospy.Time(), rospy.Duration(10))
            wr_T_toolframe = tfu.transform(sol_link, self.tool_frame, self.tflistener)
            return solframe_T_wr * wr_T_toolframe

class PR2Kinematics:

    def __init__(self, tflistener=None):
        try:
            rospy.init_node('kinematics', anonymous=True)
        except rospy.exceptions.ROSException, e:
            pass
        if tflistener == None:
            self.tflistener = tf.TransformListener()
        else:
            self.tflistener = tflistener

        self.left = PR2ArmKinematics('left', self.tflistener)
        self.right = PR2ArmKinematics('right', self.tflistener)


if __name__ == '__main__':
    import pr2
    import pdb
    robot = pr2.PR2()
    pose = robot.left.pose()

    k = PR2Kinematics(robot.tf_listener)
    cart = k.left.fk(pose)
    print 'cart pose\n', cart
    print 'real pose', pose.T
    ik_sol = k.left.ik(cart, 'torso_lift_link').T
    print 'ik pose', ik_sol
    print 'ik cart\n', k.left.fk(ik_sol)
