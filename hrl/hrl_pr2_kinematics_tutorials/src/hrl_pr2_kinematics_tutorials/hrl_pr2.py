
import numpy as np, math
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_kinematics_tutorials')
import rospy

import actionlib

from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import hrl_lib.transforms as tr
import time

class HRL_PR2():
    def __init__(self):
        self.joint_names_list = ['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint']

        rospy.wait_for_service('pr2_right_arm_kinematics/get_fk');
        rospy.wait_for_service('pr2_right_arm_kinematics/get_ik');

        self.fk_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk', GetPositionFK)
        self.ik_srv = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)

        self.joint_action_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.gripper_action_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.joint_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()

        self.arm_state_lock = RLock()
        #rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.r_arm_state_cb)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.r_arm_cart_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)

        self.r_arm_pub_l = []
        self.joint_nm_list = ['shoulder_pan', 'shoulder_lift', 'upper_arm_roll',
                              'elbow_flex', 'forearm_roll', 'wrist_flex',
                              'wrist_roll']

        self.r_arm_angles = None
        self.r_arm_efforts = None

        for nm in self.joint_nm_list:
            self.r_arm_pub_l.append(rospy.Publisher('r_'+nm+'_controller/command', Float64))

        rospy.sleep(1.)

    def joint_states_cb(self, data):
        r_arm_angles = []
        r_arm_efforts = []
        r_jt_idx_list = [17, 18, 16, 20, 19, 21, 22]
        for i,nm in enumerate(self.joint_nm_list):
            idx = r_jt_idx_list[i]
            if data.name[idx] != 'r_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            r_arm_angles.append(data.position[idx])
            r_arm_efforts.append(data.effort[idx])

        self.arm_state_lock.acquire()
        self.r_arm_angles = r_arm_angles
        self.r_arm_efforts = r_arm_efforts
        self.arm_state_lock.release()

    ## go to a joint configuration.
    # @param q - list of 7 joint angles in RADIANS.
    # @param duration - how long (SECONDS) before reaching the joint angles.
    def set_jointangles(self, arm, q, duration=0.15):
        rospy.logwarn('Currently ignoring the arm parameter.')
#        for i,p in enumerate(self.r_arm_pub_l):
#            p.publish(q[i])
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
        rospy.logwarn('Currently ignoring the arm parameter.')
        self.arm_state_lock.acquire()
        q = self.r_arm_angles
        self.arm_state_lock.release()
        return q

    # need for search and hook
    def go_cartesian(self, arm):
        rospy.logerr('Need to implement this function.')
        raise RuntimeError('Unimplemented function')

    def get_wrist_force(self, arm, bias = True, base_frame = False):
        rospy.logerr('Need to implement this function.')
        raise RuntimeError('Unimplemented function')

    def set_cartesian(self, arm, p, rot):
        rospy.logwarn('Currently ignoring the arm parameter.')
        ps = PoseStamped()
        ps.header.stamp = rospy.rostime.get_rostime()
        ps.header.frame_id = 'torso_lift_link'

        ps.pose.position.x = p[0,0]
        ps.pose.position.y = p[1,0]
        ps.pose.position.z = p[2,0]

        quat = tr.matrix_to_quaternion(rot)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        self.r_arm_cart_pub.publish(ps)

    def open_gripper(self, arm):
        self.gripper_action_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=0.08, max_effort = -1)))

    ## close the gripper
    # @param effort - supposed to be in Newtons. (-ve number => max effort)
    def close_gripper(self, arm, effort = 15):
        self.gripper_action_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=0.0, max_effort = effort)))

    def get_wrist_force(self, arm):
        pass


if __name__ == '__main__':
    rospy.init_node('hrl_pr2', anonymous = True)
    rospy.logout('hrl_pr2: ready')

    hrl_pr2 = HRL_PR2()

    if False:
        q = [0, 0, 0, 0, 0, 0, 0]
        hrl_pr2.set_jointangles('right_arm', q)
        ee_pos = hrl_pr2.FK('right_arm', q)
        print 'FK result:', ee_pos.A1

        ee_pos[0,0] -= 0.1
        q_ik = hrl_pr2.IK('right_arm', ee_pos, tr.Rx(0.), q)
        print 'q_ik:', [math.degrees(a) for a in q_ik]

        rospy.spin()

    if False:
        p = np.matrix([0.9, -0.3, -0.15]).T
        #rot = tr.Rx(0.)
        rot = tr.Rx(math.radians(90.))
        hrl_pr2.set_cartesian('right_arm', p, rot)

    hrl_pr2.open_gripper('right_arm')
    raw_input('Hit ENTER to close')
    hrl_pr2.close_gripper('right_arm', effort = 15)



