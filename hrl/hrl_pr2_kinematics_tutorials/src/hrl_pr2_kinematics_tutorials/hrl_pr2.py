
import numpy as np, math

import roslib; roslib.load_manifest('hrl_pr2_kinematics_tutorials')
import rospy

import actionlib

from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint

import hrl_lib.transforms as tr

class HRL_PR2():

    def __init__(self):

        self.joint_names_list = ['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint']

        rospy.wait_for_service('pr2_right_arm_kinematics/get_fk');
        rospy.wait_for_service('pr2_right_arm_kinematics/get_ik');

        self.fk_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk',
                                         GetPositionFK)
        self.ik_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik',
                                         GetPositionIK)

        self.joint_action_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.gripper_action_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.joint_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()

    ## go to a joint configuration.
    # @param q - list of 7 joint angles in RADIANS.
    # @param duration - how long (SECONDS) before reaching the joint angles.
    def set_jointangles(self, arm, q, duration=0.1):
        rospy.logwarn('Currently ignoring the arm parameter.')
        jtg = JointTrajectoryGoal()
        jtg.trajectory.joint_names = self.joint_names_list
        jtp = JointTrajectoryPoint()
        jtp.positions = q
        jtp.velocities = [0 for i in range(len(q))]
        jtp.accelerations = [0 for i in range(len(q))]
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)

    def FK(self, arm, q):
        rospy.logwarn('Currently ignoring the arm parameter.')
        fk_req = GetPositionFKRequest()
        fk_req.header.frame_id = 'torso_lift_link'
        fk_req.fk_link_names.append('r_wrist_roll_link')
        fk_req.robot_state.joint_state.name = self.joint_names_list
        fk_req.robot_state.joint_state.position = q

        fk_resp = GetPositionFKResponse()
        fk_resp = self.fk_srv.call(fk_req)
        if fk_resp.error_code.val == fk_resp.error_code.SUCCESS:
            x = fk_resp.pose_stamped[0].pose.position.x
            y = fk_resp.pose_stamped[0].pose.position.y
            z = fk_resp.pose_stamped[0].pose.position.z
            ret = np.matrix([x,y,z]).T
        else:
            rospy.logerr('Forward kinematics failed')
            ret = None

        return ret
    
    def IK(self, arm, p, rot, q_guess):
        rospy.logwarn('Currently ignoring the arm parameter.')
        ik_req = GetPositionIKRequest()
        ik_req.timeout = rospy.Duration(5.)
        ik_req.ik_request.ik_link_name = 'r_wrist_roll_link'
        ik_req.ik_request.pose_stamped.header.frame_id = 'torso_lift_link'

        ik_req.ik_request.pose_stamped.pose.position.x = p[0,0]
        ik_req.ik_request.pose_stamped.pose.position.y = p[1,0]
        ik_req.ik_request.pose_stamped.pose.position.z = p[2,0]

        quat = tr.matrix_to_quaternion(rot)
        ik_req.ik_request.pose_stamped.pose.orientation.x = quat[0]
        ik_req.ik_request.pose_stamped.pose.orientation.y = quat[1]
        ik_req.ik_request.pose_stamped.pose.orientation.z = quat[2]
        ik_req.ik_request.pose_stamped.pose.orientation.w = quat[3]

        ik_req.ik_request.ik_seed_state.joint_state.position = q_guess
        ik_req.ik_request.ik_seed_state.joint_state.name = self.joint_names_list

        ik_resp = self.ik_srv.call(ik_req)
        if ik_resp.error_code.val == ik_resp.error_code.SUCCESS:
            ret = ik_resp.solution.joint_state.position
        else:
            rospy.logerr('Inverse kinematics failed')
            ret = None

        return ret

    # for compatibility with Meka arms on Cody. Need to figure out a
    # good way to have a common interface to different arms.
    def step(self):
        return
    
    def end_effector_pos(self, arm):
        q = self.get_joint_angles(arm)
        return self.FK(arm, q)

    def get_joint_angles(self, arm):
        rospy.logerr('Need to implement this function.')
        raise RuntimeError('Unimplemented function')

    # need for search and hook
    def go_cartesian(self, arm):
        rospy.logerr('Need to implement this function.')
        raise RuntimeError('Unimplemented function')

    def get_wrist_force(self, arm, bias = True, base_frame = False):
        rospy.logerr('Need to implement this function.')
        raise RuntimeError('Unimplemented function')


if __name__ == '__main__':

    rospy.init_node('hrl_pr2', anonymous = True)
    rospy.logout('hrl_pr2: ready')

    hrl_pr2 = HRL_PR2()
    q = [0, 0, 0, 0, 0, 0, 0]
    hrl_pr2.set_jointangles('right_arm', q)
    ee_pos = hrl_pr2.FK('right_arm', q)
    print 'FK result:', ee_pos.A1

    ee_pos[0,0] -= 0.1
    q_ik = hrl_pr2.IK('right_arm', ee_pos, tr.Rx(0.), q)
    print 'q_ik:', [math.degrees(a) for a in q_ik]

    rospy.spin()




