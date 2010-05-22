
import numpy as np, math

import roslib; roslib.load_manifest('hrl_pr2_kinematics_tutorials')
import rospy

import actionlib

from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint

class HRL_PR2():

    def __init__(self):

        self.joint_names_list = ['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint']

        rospy.wait_for_service('pr2_right_arm_kinematics/get_fk_solver_info');
        rospy.wait_for_service('pr2_right_arm_kinematics/get_fk');
        self.fk_info = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk_solver_info',
                                          GetKinematicSolverInfo)
        self.fk_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk',
                                         GetPositionFK)

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



if __name__ == '__main__':

    rospy.init_node('hrl_pr2', anonymous = True)
    rospy.logout('hrl_pr2: ready')

    hrl_pr2 = HRL_PR2()
    q = [0, 0, 0, 0, 0, 0, 0]
    hrl_pr2.set_jointangles('right_arm', q)
    ee_pos = hrl_pr2.FK('right_arm', q)
    print 'FK result:', ee_pos.A1

    rospy.spin()




