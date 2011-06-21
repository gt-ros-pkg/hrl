import numpy as np

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
import tf.transformations as tf_trans
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from sensor_msgs.msg import JointState

JOINT_NAMES_LIST = ['_shoulder_pan_joint',
                    '_shoulder_lift_joint', '_upper_arm_roll_joint',
                    '_elbow_flex_joint', '_forearm_roll_joint',
                    '_wrist_flex_joint', '_wrist_roll_joint']
JOINT_STATE_INDS_R = [17, 18, 16, 20, 19, 21, 22]
JOINT_STATE_INDS_L = [29, 30, 28, 32, 31, 33, 34]

##
# Very basic base class for a PR2 arm. Contains functionality for IK, FK, simple
# joint commands, angle wrapping, angle biasing, getting the current angle positions.
# All functionality takes place in the torso_lift_link.
class PR2ArmBase(object):
    ##
    # Initializes subscribers
    # @param arm 'r' for right, 'l' for left
    def __init__(self, arm='r'):
        self.arm = arm
        if arm == 'r':
            arm_name = "right"
        else:
            arm_name = "left"
        self.ik_srv = rospy.ServiceProxy('pr2_' + arm_name + '_arm_kinematics/get_ik', 
                                         GetPositionIK)
        self.fk_srv = rospy.ServiceProxy('pr2_' + arm_name + '_arm_kinematics/get_fk', 
                                         GetPositionFK)
        self.joint_action_client = actionlib.SimpleActionClient(
                                       arm + '_arm_controller/joint_trajectory_action',
                                       JointTrajectoryAction)
        rospy.Subscriber('joint_states', JointState, 
                         self.joint_state_cb)
        self.joint_names_list = []
        for s in JOINT_NAMES_LIST:
            self.joint_names_list.append(arm + s)
        if arm == 'r':
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_R
        else:
            self.JOINT_STATE_INDS = JOINT_STATE_INDS_L
        self.joint_angles = None

    ##
    # Joint angles listener callback
    def joint_state_cb(self, msg):
        self.joint_angles = [msg.position[i] for i in self.JOINT_STATE_INDS]

    ##
    # Inverse kinematics
    # @param pose Homogeneous matrix specifying the desired pose in the
    #             torso_lift_link
    # @param q_guess Seed for IK, tries to find solutions near this one
    # @return Joint angles for given pose or None if none exists
    def IK(self, pose, q_guess=[0]*7):
        ik_req = GetPositionIKRequest()
        ik_req.timeout = rospy.Duration(5.)
        ik_req.ik_request.ik_link_name = self.arm + '_wrist_roll_link'
        pos = pose[:3, 3].T.A[0].tolist()
        quat = tf_trans.quaternion_from_matrix(pose)
        ik_req.ik_request.pose_stamped.header.frame_id = 'torso_lift_link'
        ik_req.ik_request.pose_stamped.pose = Pose(Point(*pos), Quaternion(*quat))
        ik_req.ik_request.ik_seed_state.joint_state.position = q_guess
        ik_req.ik_request.ik_seed_state.joint_state.name = self.joint_names_list
        ik_resp = self.ik_srv.call(ik_req)
        if ik_resp.error_code.val == ik_resp.error_code.SUCCESS:
            return ik_resp.solution.joint_state.position
        return None

    ##
    # Forward kinematics
    # @param list of 7 angles for desired position
    def FK(self, q):
        fk_req = GetPositionFKRequest()
        fk_req.header.frame_id = 'torso_lift_link'
        fk_req.fk_link_names.append(self.arm + '_wrist_roll_link') 
        fk_req.robot_state.joint_state.name = self.joint_names_list
        fk_req.robot_state.joint_state.position = q

        fk_resp = self.fk_srv.call(fk_req)
        if fk_resp.error_code.val == fk_resp.error_code.SUCCESS:
            x = fk_resp.pose_stamped[0].pose.position.x
            y = fk_resp.pose_stamped[0].pose.position.y
            z = fk_resp.pose_stamped[0].pose.position.z
            pos = np.mat([x, y, z]).T
            q1 = fk_resp.pose_stamped[0].pose.orientation.x
            q2 = fk_resp.pose_stamped[0].pose.orientation.y
            q3 = fk_resp.pose_stamped[0].pose.orientation.z
            q4 = fk_resp.pose_stamped[0].pose.orientation.w
            quat = [q1,q2,q3,q4]
            mat = np.mat(tf_trans.quaternion_matrix(quat))
            mat[:3,3] = pos
            return mat

        rospy.logerr('Forward kinematics failed')
        return None

    ##
    # Biases the given angles towards a given configuration and clips them to
    # joint limits
    # @param q Joint angles to bias
    # @param joints_bias Increment to add
    def bias_angles(self, q, joints_bias):
        if self.arm == 'r':
            max_angs = np.array([.69, 1.33, 0.79, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-2.27, -.54, -3.9, -2.34, -1000000.0, -2.15, -1000000.0])
        else:
            max_angs = np.array([2.27, 1.33, 3.9, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-.69, -.54, -0.79, -2.34, -1000000.0, -2.15, -1000000.0])
        q_off = np.array(joints_bias)
        angs = np.array(q) + q_off
        angs = np.clip(angs, min_angs, max_angs)
        return angs.tolist()

    ##
    # Runs IK but tries to bias it towards the given bias configuration by
    # calculating IK over several iterations and adding increments to the
    # solutions.
    # @param pose Pose for IK
    # @param init_angs Start angles for biasing
    # @param joints_bias Angles to increment the IK solutions at each iteration
    # @param num_iters Number of times to add the bias
    def biased_IK(self, pose, init_angs=None, joints_bias = [0.]*7, num_iters=6):
        if init_angs is None:
            angs = self.get_joint_angles(wrapped=True)
        else:
            angs = init_angs
        has_solution = False
        for i in range(num_iters):
            ik_solution = self.IK(pose, angs)
            if ik_solution:
                angs = ik_solution
                has_solution = True
            if i < num_iters - 1:
                angs = self.bias_angles(angs, joints_bias)
        if has_solution:
            return angs
        else:
            return None

    def search_IK(self, pose, q_guess, sigma=0.6, sample_size=10,
                  search_penalties=[10, 10, 8, 5, 2, 2, 1]):
        q_samples = []
        q_diffs = []
        for i in range(sample_size):
            if i != 0:
                rot_vals = np.random.normal(0, sigma, 3)
            else:
                rot_vals = [0]*3
            rot_vals = np.clip(rot_vals, -np.pi, np.pi)
            rand_rot = tf_trans.euler_matrix(*rot_vals)
            new_pose = pose.copy()
            new_pose[:3,:3] = rand_rot[:3,:3] * new_pose[:3,:3]
            cur_time = rospy.Time.now().to_sec()
            q_sample = self.IK(new_pose, q_guess)
            if q_sample is None:
                continue
            q_samples.append(q_sample)
            q_diffs.append(np.sum(np.array(search_penalties) * 
                                  self.angle_difference(q_guess, q_sample)))
        if len(q_samples) == 0:
            return None
        return q_samples[np.argmin(q_diffs)]

    ##
    # Commands joint angles to a single position
    # @param q Joint angles
    # @param time 
    def command_joint_angles(self, q, duration=1.0, delay=0.0):
        if q is None or len(q) != 7:
            return False
        jtg = JointTrajectoryGoal()
        jtg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        jtg.trajectory.joint_names = self.joint_names_list
        jtp = JointTrajectoryPoint()
        jtp.positions = list(q)
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)
        return True

    ##
    # Commands the arm to move to it's current joint angles, effectively stopping
    # the arm in place.
    def freeze_arm(self):
        self.command_joint_angles(self.get_joint_angles(wrapped=False))

    ##
    # Returns the current joint angle positions
    # @param wrapped If False returns the raw encoded positions, if True returns
    #                the angles with the forearm and wrist roll in the range -pi to pi
    def get_joint_angles(self, wrapped=False):
        if self.joint_angles is None:
            rospy.logerr("[pr2_arm_base] Joint angles haven't been filled yet")
            return None
        if wrapped:
            return self.wrap_angles(self.joint_angles)
        else:
            return np.array(self.joint_angles)

    ##
    # Returns the same angles with the forearm and wrist roll wrapped to the 
    # range -pi to pi
    # @param q Joint angles
    # @return Wrapped joint angles
    def wrap_angles(self, q):
        q = list(q)
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return np.array(q)

    ##
    # Returns the difference between the two joint positions, taking into account
    # wrapping
    def angle_difference(self, q_a, q_b):
        diff = np.array(self.wrap_angles(q_a)) - np.array(self.wrap_angles(q_b))
        for ind in [4, 6]:
            if abs(diff[ind] + 2*np.pi) < abs(diff[ind]):
                diff[ind] += 2*np.pi
            if abs(diff[ind] - 2*np.pi) < abs(diff[ind]):
                diff[ind] -= 2*np.pi
        return diff



