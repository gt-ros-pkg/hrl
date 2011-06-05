#! /usr/bin/python

import numpy as np
import copy

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_srvs.srv
import tf.transformations as tf_trans
import tf

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from sensor_msgs.msg import JointState

#import pr2_arms.pr2_arms as epc_pr2
#import epc_core.epc as epc_core
from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from hrl_arm_move_behaviors.msg import LinearMoveRelativeGoal, LinearMoveRelativeAction
from hrl_arm_move_behaviors.msg import LinearMoveRelativeResult, LinearMoveRelativeSetupGoal
from hrl_arm_move_behaviors.msg import LinearMoveRelativeSetupAction, LinearMoveRelativeSetupResult
from pr2_collision_monitor.srv import JointDetectionStart, ArmMovingWait

##
# Transforms the given position by the offset position in the given quaternion
# rotation frame
def transform_in_frame(pos, quat, off_point):
    quatmat = np.mat(tf_trans.quaternion_matrix(quat))
    quatmat[0:3,3] = np.mat(pos).T
    trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
    transpos = quatmat * trans
    return transpos.T.A[0,0:3]

def extract_pose_stamped(ps):
    seq = ps.header.seq; stamp = ps.header.stamp; frame_id = ps.header.frame_id
    px = ps.pose.position.x; py = ps.pose.position.y; pz = ps.pose.position.z
    ox = ps.pose.orientation.x; oy = ps.pose.orientation.y
    oz = ps.pose.orientation.z; ow = ps.pose.orientation.w
    return [seq, stamp, frame_id], [px, py, pz], [ox, oy, oz, ow]

def pose_msg_to_mat(ps):
    header, pos, quat = extract_pose_stamped(ps)
    return pose_pq_to_mat(pos, quat)

def pose_pq_to_mat(pos, quat):
    B = np.mat(tf_trans.quaternion_matrix(quat))
    B[0:3,3] = np.mat([pos]).T
    return B

def pose_mat_to_msg(B):
    pos = B[0:3,3].T.tolist()
    quat = tf_trans.quaternion_from_matrix(B)
    return Pose(Point(*pos[0]), Quaternion(*quat))

def pose_mat_to_stamped_msg(frame_id, B, time=None):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    if time is None:
        ps.header.stamp = rospy.Time.now()
    else:
        ps.header.stamp = time
    ps.pose = pose_mat_to_msg(B)
    return ps

def quaternion_dist(B_a, B_b):
    quat_a = tf_trans.quaternion_from_matrix(B_a)
    quat_b = tf_trans.quaternion_from_matrix(B_b)
    quat_a_norm = quat_a / np.linalg.norm(quat_a)
    quat_b_norm = quat_b / np.linalg.norm(quat_b)
    dot = np.dot(quat_a_norm, quat_b_norm)
    if dot > 0.99999:
        dist = 0
    else:
        dist = np.arccos(dot)
    return dist

def interpolate_cartesian(start_pose, end_pose, num_steps):
    diff_pos = end_pose[:3,3] - start_pose[:3,3]
    start_quat = tf_trans.quaternion_from_matrix(start_pose)
    end_quat = tf_trans.quaternion_from_matrix(end_pose)
    mat_list = []
    for fraction in np.linspace(0, 1, num_steps):
        cur_quat = tf_trans.quaternion_slerp(start_quat, end_quat, fraction)
        cur_mat = np.mat(tf_trans.quaternion_matrix(cur_quat))
        cur_mat[:3,3] = start_pose[:3,3] + fraction * diff_pos
        mat_list.append(cur_mat)
    return mat_list
    

JOINT_NAMES_LIST = ['_shoulder_pan_joint',
                    '_shoulder_lift_joint', '_upper_arm_roll_joint',
                    '_elbow_flex_joint', '_forearm_roll_joint',
                    '_wrist_flex_joint', '_wrist_roll_joint']

class LinearMove(object):
    def __init__(self, arm, tf_listener=None):
        self.arm = arm
        if tf_listener is None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
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
        rospy.Subscriber(arm + '_arm_controller/state', JointTrajectoryControllerState, 
                         self.joint_controller_cb)
        self.joint_names_list = []
        for s in JOINT_NAMES_LIST:
            self.joint_names_list.append(arm + s)

    def joint_controller_cb(self, msg):
        self.joint_angles = msg.actual.positions

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

    def get_transform(self, from_frame, to_frame, time=rospy.Time()):
        pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame, time)
        return pose_pq_to_mat(pos, quat)

    def command_angles(self, q, duration=1.0):
        if q is None or len(q) != 7:
            return False
        jtg = JointTrajectoryGoal()
        jtg.trajectory.joint_names = self.joint_names_list
        jtp = JointTrajectoryPoint()
        jtp.positions = q
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)
        return True

    def get_angles(self, wrapped=False):
        if wrapped:
            return self.wrap_angles(self.joint_angles)
        else:
            return self.joint_angles

    def wrap_angles(self, q):
        q = list(q)
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2*np.pi
            while q[ind] > np.pi:
                q[ind] -= 2*np.pi
        return q

    def angle_diff(self, q_a, q_b):
        diff = np.array(q_a) - np.array(q_b)
        for ind in [4, 6]:
            if abs(diff[ind] + 2*np.pi) < abs(diff[ind]):
                diff[ind] += 2*np.pi
            if abs(diff[ind] - 2*np.pi) < abs(diff[ind]):
                diff[ind] -= 2*np.pi
        return diff

    ##
    # Added function which biases the given angles towards a given configuration
    def bias_angle_guess(self, q, joints_bias):
        if self.arm == 'r':
            max_angs = np.array([.69, 1.33, 0.79, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-2.27, -.54, -3.9, -2.34, -1000000.0, -2.15, -1000000.0])
        else:
            max_angs = np.array([2.27, 1.33, 3.9, 0.0, 1000000.0, 0.0, 1000000.0])
            min_angs = np.array([-.69, -.54, -0.79, -2.34, -1000000.0, -2.15, -1000000.0])
        q_off = np.array(joints_bias)
        angs = np.array(q) + q_off
        for i in range(7):
            if angs[i] > max_angs[i]:
                angs[i] = max_angs[i]
            elif angs[i] < min_angs[i]:
                angs[i] = min_angs[i]
        return angs.tolist()

    ##
    # runs ik but tries to bias it towards the given bias configuration
    def biased_IK(self, pose, init_angs=[0.]*7, joints_bias = [0.]*7, num_iters=6):
        angs = init_angs
        has_solution = False
        for i in range(num_iters):
            ik_solution = self.IK(pose, angs)
            if ik_solution:
                angs = ik_solution
                has_solution = True
            if i < num_iters - 1:
                angs = self.bias_angle_guess(angs, joints_bias)
        if has_solution:
            return angs
        else:
            return None

    def move_to_pose(self, end_pose, tool_frame, velocity=0.03, rate=5):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        # magic numbers
        k_p_pos, k_p_ang = 0.3, 10.1
        k_d_pos, k_d_ang = 0.02, 0.1
        k_i_pos, k_i_ang = 1.500, 0.0
        integral_pos_max, integral_ang_max = 0.1, 1.0
        max_angles = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # advertise informative topics
        cmd_pub = rospy.Publisher("/commanded_pose", PoseStamped)
        cur_pub = rospy.Publisher("/current_pose", PoseStamped)
        goal_pub = rospy.Publisher("/goal_pose", PoseStamped)
        u_pub = rospy.Publisher("/u_signal", Float64)
        p_pub = rospy.Publisher("/p_signal", Float64)
        i_pub = rospy.Publisher("/i_signal", Float64)
        d_pub = rospy.Publisher("/d_signal", Float64)

        # initialze variables
        err_pos_mag_last, err_ang_last = 0, 0
        steady_count, num_ik_not_found = 0, 0
        integral_pos, integral_ang = 0, 0

        while not rospy.is_shutdown():
            # find current tool location
            t_B_c = self.get_transform("torso_lift_link", tool_frame)

            # find error in position
            err_pos = end_pose[:3,3] - t_B_c[:3,3]
            err_pos_mag = np.linalg.norm(err_pos)

            # find error in angle
            err_ang = quaternion_dist(end_pose, t_B_c)

            # compute integrals
            integral_pos += err_pos_mag / rate
            integral_ang += err_ang / rate
            integral_pos = np.clip(integral_pos, -integral_pos_max, integral_pos_max)
            integral_ang = np.clip(integral_ang, -integral_ang_max, integral_ang_max)

            print "t_B_c, end", t_B_c, end_pose
            print "err", err_pos, err_ang, err_pos_mag
            print "deriv", (err_pos_mag - err_pos_mag_last) * rate, 
            print "integral", integral_pos
            
            # check to see if we have settled
            if err_pos_mag < 0.02 and err_ang < 0.08:
                steady_count += 1
                if steady_count > 4:
                    return True
            else:
                steady_count = 0

            # find control input
            u_pos_mag = (k_p_pos * err_pos_mag + 
                         k_d_pos * (err_pos_mag - err_pos_mag_last) * rate +
                         k_i_pos * integral_pos)
            slerp_t = k_p_ang * err_ang + k_d_ang * (err_ang - err_ang_last) * rate 
            slerp_t = np.clip(slerp_t, 0, 1)

            # find commanded frame of tool

            # rotation
            ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                                   tf_trans.quaternion_from_matrix(end_pose.copy()),
                                                   slerp_t)
            t_B_new = np.mat(tf_trans.quaternion_matrix(ei_q_slerp))

            # position
            u_pos_clamped = np.clip(u_pos_mag, -0.20, 0.20)
            t_B_new[:3,3] = t_B_c[:3,3] + u_pos_clamped * err_pos / err_pos_mag

            print "u_pos_mag", u_pos_mag, k_p_pos * err_pos_mag, \
                         k_d_pos * (err_pos_mag - err_pos_mag_last) * rate, \
                         k_i_pos * integral_pos
            print "slerp_t", slerp_t
            print "u_pos_mag_clamped", u_pos_clamped
            print "add",  u_pos_clamped * err_pos / err_pos_mag
            print "new", t_B_new

            # transform commanded frame into wrist
            l_B_w = self.get_transform(tool_frame, self.arm + '_wrist_roll_link')
            t_B_new_wrist = t_B_new * l_B_w

            # find IK solution
            q_guess = self.get_angles(wrapped=True)
            q_cmd = self.IK(t_B_new_wrist, q_guess)

            # publish informative topics
            cmd_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_new))
            cur_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_c))
            goal_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", end_pose))
            u_pub.publish(Float64(u_pos_mag))
            p_pub.publish(Float64(k_p_pos * err_pos_mag))
            i_pub.publish(Float64(k_i_pos * integral_pos))
            d_pub.publish(Float64(k_d_pos * (err_pos_mag - err_pos_mag_last) * rate))

            # ignore if no IK solution found
            if q_cmd is None:
                rospy.logerr("No IK solution found")
                num_ik_not_found += 1
                if num_ik_not_found > 4:
                    rospy.logerr("Controller has hit a rut, no IK solutions in area")
                    return False
                rospy.sleep(1.0/rate)
                continue
            else:
                num_ik_not_found = 0

            # safety checking
            cur_angles = self.get_angles(wrapped=True)
            angle_diff = self.angle_diff(q_cmd, cur_angles)
            q_cmd_clamped = np.clip(q_cmd, cur_angles - max_angles, cur_angles + max_angles)

            print "Goal dist:", np.linalg.norm(err_pos)
            print "q_cmd" , q_cmd
            print "q_cmd_clamped", q_cmd_clamped
            #print "t_B_c", t_B_c
            #print "end_pose", end_pose
            #print "t_B_new_wrist", t_B_new_wrist
            #print "FK", self.FK(self.get_angles())

            # command joints
            self.command_angles(q_cmd_clamped, 1.2/rate)
    
            # cleanup
            err_pos_mag_last = err_pos_mag
            err_ang_last = err_ang
            rospy.sleep(1.0/rate)

    def move_to_pose2(self, end_pose, tool_frame, velocity=0.03, rate=5):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        # magic numbers
        k_p_pos, k_p_ang = 0.5, 10.1
        k_d_pos, k_d_ang = 0.09, 0.1
        k_i_pos, k_i_ang = 0.300, 0.0
        integral_pos_max, integral_ang_max = 0.4, 1.0
        max_angles = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # advertise informative topics
        cmd_pub = rospy.Publisher("/commanded_pose", PoseStamped)
        cur_pub = rospy.Publisher("/current_pose", PoseStamped)
        goal_pub = rospy.Publisher("/goal_pose", PoseStamped)
        u_pub = rospy.Publisher("/u_signal", Float64)
        p_pub = rospy.Publisher("/p_signal", Float64)
        i_pub = rospy.Publisher("/i_signal", Float64)
        d_pub = rospy.Publisher("/d_signal", Float64)

        # initialze variables
        err_pos_last, err_ang_last = np.mat([0]*3).T, 0
        steady_count, num_ik_not_found = 0, 0
        integral_pos, integral_ang = np.mat([0]*3).T, 0

        while not rospy.is_shutdown():

            # find current tool location
            t_B_c = self.get_transform("torso_lift_link", tool_frame)

            # find error in position
            err_pos = end_pose[:3,3] - t_B_c[:3,3]
            err_pos_mag = np.linalg.norm(err_pos)

            # find error in angle
            err_ang = quaternion_dist(end_pose, t_B_c)

            # compute integrals
            #integral_pos += err_pos_mag / rate
            integral_pos += err_pos / rate
            integral_ang += err_ang / rate
            integral_pos = np.clip(integral_pos, -integral_pos_max, integral_pos_max)
            integral_ang = np.clip(integral_ang, -integral_ang_max, integral_ang_max)

            print "t_B_c, end", t_B_c, end_pose
            print "err", err_pos, err_ang, err_pos_mag
            print "deriv", (err_pos - err_pos_last) * rate, 
            print "integral", integral_pos
            
            # check to see if we have settled
            if err_pos_mag < 0.02 and err_ang < 0.15:
                steady_count += 1
                if steady_count > 4:
                    return True
            else:
                steady_count = 0

            # find control input
            #u_pos_mag = (k_p_pos * err_pos_mag + 
            #             k_d_pos * (err_pos_mag - err_pos_mag_last) * rate +
            #             k_i_pos * integral_pos)
            u_pos_mag = (k_p_pos * err_pos + 
                         k_d_pos * (err_pos - err_pos_last) * rate +
                         k_i_pos * integral_pos)
            slerp_t = k_p_ang * err_ang + k_d_ang * (err_ang - err_ang_last) * rate 
            slerp_t = np.clip(slerp_t, 0, 1)

            # find commanded frame of tool

            # rotation
            ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                                   tf_trans.quaternion_from_matrix(end_pose.copy()),
                                                   slerp_t)
            t_B_new = np.mat(tf_trans.quaternion_matrix(ei_q_slerp))

            # position
            u_pos_clamped = np.clip(u_pos_mag, -0.20, 0.20)
            t_B_new[:3,3] = t_B_c[:3,3] + u_pos_clamped 

            print "u_pos_mag", u_pos_mag, k_p_pos * err_pos_mag, \
                         k_d_pos * (err_pos - err_pos_last) * rate, \
                         k_i_pos * integral_pos
            print "slerp_t", slerp_t
            print "u_pos_mag_clamped", u_pos_clamped
            print "add",  u_pos_clamped
            print "new", t_B_new

            # transform commanded frame into wrist
            l_B_w = self.get_transform(tool_frame, self.arm + '_wrist_roll_link')
            t_B_new_wrist = t_B_new * l_B_w

            # find IK solution
            q_guess = self.get_angles(wrapped=True)
            q_cmd = self.IK(t_B_new_wrist, q_guess)

            # publish informative topics
            cmd_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_new))
            cur_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_c))
            goal_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", end_pose))
            u_pub.publish(Float64(np.linalg.norm(u_pos_mag)))
            p_pub.publish(Float64(k_p_pos * np.linalg.norm(err_pos)))
            i_pub.publish(Float64(k_i_pos * np.linalg.norm(integral_pos)))
            d_pub.publish(Float64(k_d_pos * np.linalg.norm(err_pos - err_pos_last) * rate))

            # ignore if no IK solution found
            if q_cmd is None:
                rospy.logerr("No IK solution found")
                num_ik_not_found += 1
                if num_ik_not_found > 4:
                    rospy.logerr("Controller has hit a rut, no IK solutions in area")
                    return False
                rospy.sleep(1.0/rate)
                continue
            else:
                num_ik_not_found = 0

            # safety checking
            cur_angles = self.get_angles(wrapped=True)
            angle_diff = self.angle_diff(q_cmd, cur_angles)
            max_angles = np.array([100]*7)
            q_cmd_clamped = np.clip(q_cmd, cur_angles - max_angles, cur_angles + max_angles)

            print "Goal dist:", np.linalg.norm(err_pos)
            print "q_cmd" , q_cmd
            print "q_cmd_clamped", q_cmd_clamped
            #print "t_B_c", t_B_c
            #print "end_pose", end_pose
            #print "t_B_new_wrist", t_B_new_wrist
            #print "FK", self.FK(self.get_angles())

            # command joints
            self.command_angles(q_cmd_clamped, 1.2/rate)
    
            # cleanup
            err_pos_last = err_pos
            err_ang_last = err_ang
            rospy.sleep(1.0/rate)

            self.err_pos = err_pos
            self.integral_pos = integral_pos

    def linear_trajectory3(self, start_pose, end_pose, tool_frame, velocity=0.03, rate=5):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        # magic numbers
        k_p_pos, k_p_ang = 0.5, 10.1
        k_d_pos, k_d_ang = 0.09, 0.1
        k_i_pos, k_i_ang = 0.300, 0.0
        integral_pos_max, integral_ang_max = 0.4, 1.0
        max_angles = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # create trajectory
        dist = np.linalg.norm(start_pose[:3,3] - end_pose[:3,3])
        traj_time = dist / velocity
        num_steps = traj_time * rate
        trajectory = interpolate_cartesian(start_pose, end_pose, num_steps)

        # TODO FIX HACK
        trajectory_setup = interpolate_cartesian(start_pose, start_pose, 5)
        trajectory = trajectory_setup + trajectory

        # advertise informative topics
        cmd_pub = rospy.Publisher("/commanded_pose", PoseStamped)
        cur_pub = rospy.Publisher("/current_pose", PoseStamped)
        goal_pub = rospy.Publisher("/goal_pose", PoseStamped)
        u_pub = rospy.Publisher("/u_signal", Float64)
        p_pub = rospy.Publisher("/p_signal", Float64)
        i_pub = rospy.Publisher("/i_signal", Float64)
        d_pub = rospy.Publisher("/d_signal", Float64)

        # initialze variables
        err_pos_last, err_ang_last = np.mat([0]*3).T, 0
        steady_count, num_ik_not_found = 0, 0
        integral_pos, integral_ang = np.mat([0]*3).T, 0

        err_pos_last = self.err_pos
        integral_pos = self.integral_pos
        for target_pose in trajectory:

            # find current tool location
            t_B_c = self.get_transform("torso_lift_link", tool_frame)

            # find error in position
            err_pos = target_pose[:3,3] - t_B_c[:3,3]
            err_pos_mag = np.linalg.norm(err_pos)

            # find error in angle
            err_ang = quaternion_dist(target_pose, t_B_c)

            # compute integrals
            #integral_pos += err_pos_mag / rate
            integral_pos += err_pos / rate
            integral_ang += err_ang / rate
            integral_pos = np.clip(integral_pos, -integral_pos_max, integral_pos_max)
            integral_ang = np.clip(integral_ang, -integral_ang_max, integral_ang_max)

            print "t_B_c, end", t_B_c, target_pose
            print "err", err_pos, err_ang, err_pos_mag
            print "deriv", (err_pos - err_pos_last) * rate, 
            print "integral", integral_pos

            # find control input
            #u_pos_mag = (k_p_pos * err_pos_mag + 
            #             k_d_pos * (err_pos_mag - err_pos_mag_last) * rate +
            #             k_i_pos * integral_pos)
            u_pos_mag = (k_p_pos * err_pos + 
                         k_d_pos * (err_pos - err_pos_last) * rate +
                         k_i_pos * integral_pos)
            slerp_t = k_p_ang * err_ang + k_d_ang * (err_ang - err_ang_last) * rate 
            slerp_t = np.clip(slerp_t, 0, 1)

            # find commanded frame of tool

            # rotation
            ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                                   tf_trans.quaternion_from_matrix(target_pose.copy()),
                                                   slerp_t)
            t_B_new = np.mat(tf_trans.quaternion_matrix(ei_q_slerp))

            # position
            u_pos_clamped = np.clip(u_pos_mag, -0.20, 0.20)
            t_B_new[:3,3] = t_B_c[:3,3] + u_pos_clamped 

            print "u_pos_mag", u_pos_mag, k_p_pos * err_pos_mag, \
                         k_d_pos * (err_pos - err_pos_last) * rate, \
                         k_i_pos * integral_pos
            print "slerp_t", slerp_t
            print "u_pos_mag_clamped", u_pos_clamped
            print "add",  u_pos_clamped
            print "new", t_B_new

            # transform commanded frame into wrist
            l_B_w = self.get_transform(tool_frame, self.arm + '_wrist_roll_link')
            t_B_new_wrist = t_B_new * l_B_w

            # find IK solution
            q_guess = self.get_angles(wrapped=True)
            q_cmd = self.IK(t_B_new_wrist, q_guess)

            # publish informative topics
            cmd_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_new))
            cur_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_c))
            goal_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", target_pose))
            u_pub.publish(Float64(np.linalg.norm(u_pos_mag)))
            p_pub.publish(Float64(k_p_pos * np.linalg.norm(err_pos)))
            i_pub.publish(Float64(k_i_pos * np.linalg.norm(integral_pos)))
            d_pub.publish(Float64(k_d_pos * np.linalg.norm(err_pos - err_pos_last) * rate))

            # ignore if no IK solution found
            if q_cmd is None:
                rospy.logerr("No IK solution found")
                num_ik_not_found += 1
                if num_ik_not_found > 4:
                    rospy.logerr("Controller has hit a rut, no IK solutions in area")
                    return False
                rospy.sleep(1.0/rate)
                continue
            else:
                num_ik_not_found = 0

            # safety checking
            cur_angles = self.get_angles(wrapped=True)
            angle_diff = self.angle_diff(q_cmd, cur_angles)
            max_angles = np.array([100]*7)
            q_cmd_clamped = np.clip(q_cmd, cur_angles - max_angles, cur_angles + max_angles)

            print "Goal dist:", np.linalg.norm(err_pos)
            print "q_cmd" , q_cmd
            print "q_cmd_clamped", q_cmd_clamped
            #print "t_B_c", t_B_c
            #print "target_pose", target_pose
            #print "t_B_new_wrist", t_B_new_wrist
            #print "FK", self.FK(self.get_angles())

            # command joints
            self.command_angles(q_cmd_clamped, 1.2/rate)
    
            # cleanup
            err_pos_last = err_pos
            err_ang_last = err_ang
            rospy.sleep(1.0/rate)

    def linear_trajectory2(self, start_pose, end_pose, tool_frame, velocity=0.03, rate=5):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        # magic numbers
        k_p_pos, k_p_ang = 0.7, 10.1
        k_d_pos, k_d_ang = 0.10, 0.1
        k_i_pos, k_i_ang = 0.500, 0.0
        integral_pos_max, integral_ang_max = 0.3, 1.0
        max_angles = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # create trajectory
        dist = np.linalg.norm(start_pose[:3,3] - end_pose[:3,3])
        traj_time = dist / velocity
        num_steps = traj_time * rate
        trajectory = interpolate_cartesian(start_pose, end_pose, num_steps)

        # advertise informative topics
        cmd_pub = rospy.Publisher("/commanded_pose", PoseStamped)
        cur_pub = rospy.Publisher("/current_pose", PoseStamped)
        goal_pub = rospy.Publisher("/goal_pose", PoseStamped)
        u_pub = rospy.Publisher("/u_signal", Float64)

        # initialze variables
        err_pos_mag_last, err_ang_last = 0, 0
        steady_count, num_ik_not_found = 0, 0
        integral_pos, integral_ang = 0, 0

        for target_pose in trajectory:

            # find current tool location
            t_B_c = self.get_transform("torso_lift_link", tool_frame)

            # find error in position
            err_pos = target_pose[:3,3] - t_B_c[:3,3]
            err_pos_mag = np.linalg.norm(err_pos)

            # find error in angle
            err_ang = quaternion_dist(target_pose, t_B_c)

            # compute integrals
            integral_pos += err_pos_mag / rate
            integral_ang += err_ang / rate
            integral_pos = np.clip(integral_pos, -integral_pos_max, integral_pos_max)
            integral_ang = np.clip(integral_ang, -integral_ang_max, integral_ang_max)

            print "t_B_c, end", t_B_c, target_pose
            print "err", err_pos, err_ang, err_pos_mag
            print "deriv", (err_pos_mag - err_pos_mag_last) * rate, 
            print "integral", integral_pos

            # find control input
            u_pos_mag = (k_p_pos * err_pos_mag + 
                         k_d_pos * (err_pos_mag - err_pos_mag_last) * rate +
                         k_i_pos * integral_pos)
            slerp_t = k_p_ang * err_ang + k_d_ang * (err_ang - err_ang_last) * rate 
            slerp_t = np.clip(slerp_t, 0, 1)

            # find commanded frame of tool

            # rotation
            ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                                   tf_trans.quaternion_from_matrix(target_pose.copy()),
                                                   slerp_t)
            t_B_new = np.mat(tf_trans.quaternion_matrix(ei_q_slerp))

            # position
            u_pos_clamped = np.clip(u_pos_mag, -0.20, 0.20)
            t_B_new[:3,3] = t_B_c[:3,3] + u_pos_clamped * err_pos / err_pos_mag

            print "u_pos_mag", u_pos_mag, k_p_pos * err_pos_mag, \
                         k_d_pos * (err_pos_mag - err_pos_mag_last) * rate, \
                         k_i_pos * integral_pos
            print "slerp_t", slerp_t
            print "u_pos_mag_clamped", u_pos_clamped
            print "add",  u_pos_clamped * err_pos / err_pos_mag
            print "new", t_B_new

            # transform commanded frame into wrist
            l_B_w = self.get_transform(tool_frame, self.arm + '_wrist_roll_link')
            t_B_new_wrist = t_B_new * l_B_w

            # find IK solution
            q_guess = self.get_angles(wrapped=True)
            q_cmd = self.IK(t_B_new_wrist, q_guess)

            # publish informative topics
            cmd_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_new))
            cur_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", t_B_c))
            goal_pub.publish(pose_mat_to_stamped_msg("torso_lift_link", target_pose))
            u_pub.publish(Float64(u_pos_mag))

            # ignore if no IK solution found
            if q_cmd is None:
                rospy.logerr("No IK solution found")
                num_ik_not_found += 1
                if num_ik_not_found > 4:
                    rospy.logerr("Controller has hit a rut, no IK solutions in area")
                    return False
                rospy.sleep(1.0/rate)
                continue
            else:
                num_ik_not_found = 0

            # safety checking
            cur_angles = self.get_angles(wrapped=True)
            angle_diff = self.angle_diff(q_cmd, cur_angles)
            q_cmd_clamped = np.clip(q_cmd, cur_angles - max_angles, cur_angles + max_angles)

            print "Goal dist:", np.linalg.norm(err_pos)
            print "q_cmd" , q_cmd
            print "q_cmd_clamped", q_cmd_clamped
            #print "t_B_c", t_B_c
            #print "target_pose", target_pose
            #print "t_B_new_wrist", t_B_new_wrist
            #print "FK", self.FK(self.get_angles())

            # command joints
            self.command_angles(q_cmd_clamped, 1.2/rate)
    
            # cleanup
            err_pos_mag_last = err_pos_mag
            err_ang_last = err_ang
            rospy.sleep(1.0/rate)


    def linear_trajectory(self, start_pose, end_pose, tool_frame, velocity=0.01, rate=1):
        """Commands arm to follow a linear trajectory, controlling the tool's frame
           and using PD equilibrium point control to keep the tool fixed on this
           trajectory
           @param start_pose PoseStamped containing start position
           @param end_pose PoseStamped containing end position
           @param tool_frame string indicating tool's frame
        """
        k_p_pos, k_p_ang = 0.3, 0.1
        k_d_pos, k_d_ang = 0, 0
        max_angles = np.array([0.2, 0.2, 0.2, 0.2, 0.6, 0.6, 0.6])
        dist = np.linalg.norm(start_pose[:3,3] - end_pose[:3,3])
        traj_time = dist / velocity
        num_steps = traj_time * rate
        #print dist, traj_time, num_steps
        # form desired trajectory
        trajectory = interpolate_cartesian(start_pose, end_pose, num_steps)
        #print trajectory
        err_pos_last, err_ang_last = np.mat([0]*3).T, 0
        for t_B_ei in trajectory:
            # find current tool location
            t_B_c = self.get_transform("torso_lift_link", tool_frame)
            # find error in position
            err_pos = t_B_ei[:3,3] - t_B_c[:3,3]
            # find error in angle
            err_ang = quaternion_dist(t_B_ei, t_B_c)
            #print "err", err_pos, err_ang
            # find control input
            u_pos = k_p_pos * err_pos + k_d_pos * (err_pos - err_pos_last) * rate 
            slerp_t = k_p_ang * err_ang + k_d_ang * (err_ang - err_ang_last) * rate 
            #print "u", u_pos, slerp_t
            slerp_t = 0

            # find commanded frame of tool

            # rotation
            ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                                   tf_trans.quaternion_from_matrix(t_B_ei),
                                                   slerp_t)
            #print "slerp", np.mat(tf_trans.quaternion_matrix(ei_q_slerp))
            # TODO FIX TODO slerp bad?
            t_B_new = t_B_ei * np.mat(tf_trans.quaternion_matrix(ei_q_slerp))
            t_B_new = t_B_ei

            # position
            t_B_new[:3,3] = u_pos + t_B_ei[:3,3]

            # transform commanded frame into wrist
            l_B_w = self.get_transform(tool_frame, self.arm + '_wrist_roll_link')
            #print "l_B_W", l_B_w
            t_B_new_wrist = t_B_new * l_B_w
            # find IK solution
            q_guess = self.get_angles(wrapped=True)
            q_cmd = self.IK(t_B_new_wrist, q_guess)
            if q_cmd is None:
                rospy.logerr("No IK solution found")
                continue
            #print "t_B_c", t_B_c
            #print "t_B_ei", t_B_ei
            #print "t_B_new_wrist", t_B_new_wrist
            #print "FK", self.FK(self.get_angles())

            # safety checking
            cur_angles = self.get_angles(wrapped=True)
            #print "cur_angles", cur_angles
            #print "q_cmd", q_cmd
            angle_diff = self.angle_diff(q_cmd, cur_angles)
            #print "angle_diff", angle_diff
            print "Goal dist:", np.linalg.norm(err_pos)
            if np.any( np.fabs(angle_diff) > max_angles):
                rospy.logerr("Commanded angles greater than max allowed")
                continue

            # command joints
            self.command_angles(q_cmd, 1.2/rate)
    
            # cleanup
            err_pos_last = err_pos
            err_ang_last = err_ang
            rospy.sleep(1.0/rate)

class LinearMoveRelative(object):
    def __init__(self):
        self.MOVE_VELOCITY = 0.05
        self.SETUP_VELOCITY = 0.15
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        #self.BIAS_RADIUS = 0.012
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        #self.INIT_ANGS = [0.31224765812286392, 0.67376890754823038, -0.033887965389851837, -2.0722385314846998, -3.1341338126914398, -1.3392539168745463, -0.04655005168062587]

        self.arm = rospy.get_param("~arm", default="r")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

#self.cm = ControllerManager(self.arm)
#self.tf_listener = self.cm.tf_listener
#self.pr2_arms = epc_pr2.PR2Arms()
#self.epc = epc_core.EPC(self.pr2_arms)
#self.tf_listener = self.pr2_arms.tf_lstnr
        self.tf_listener = tf.TransformListener()
        self.lin_move = LinearMove(self.arm, self.tf_listener)
        self.arm_num = self.arm == "l"

        rospy.loginfo("Waiting for start_monitoring")
        rospy.wait_for_service(self.arm + '_collision_monitor/start_monitoring')
        self.start_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/start_monitoring', 
                                                  JointDetectionStart, persistent=False)
        rospy.loginfo("Waiting for stop_monitoring")
        rospy.wait_for_service(self.arm + '_collision_monitor/stop_monitoring')
        self.stop_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/stop_monitoring', 
                                                 std_srvs.srv.Empty, persistent=False)
        self.stop_detection()
        rospy.loginfo("Waiting for arm_moving_wait")
        rospy.wait_for_service(self.arm + '_arm_moving_server/arm_moving_wait')
        self.arm_moving_wait = rospy.ServiceProxy(self.arm + '_arm_moving_server/arm_moving_wait', 
                                                  ArmMovingWait, persistent=False)

        class CollisionState:
            def __init__(self, lmr):
                self.collided = False
                self.lmr = lmr
            def callback(self, msg):
                if msg.data:
                    self.collided = True
#self.lmr.cm.joint_action_client.cancel_all_goals()
        self.coll_state = CollisionState(self)
        rospy.Subscriber(self.arm + "_collision_monitor/collision_detected", 
                         Bool, self.coll_state.callback)

        self.move_arm_act = actionlib.SimpleActionServer("~move_relative", 
                                                         LinearMoveRelativeAction, 
                                                         self.execute_move, False)
        self.move_arm_act.start()

        self.move_arm_setup_act = actionlib.SimpleActionServer("~move_relative_setup", 
                                                               LinearMoveRelativeSetupAction, 
                                                               self.execute_setup, False)
        self.move_arm_setup_act.start()

        self.pix3_pub = rospy.Publisher("/lin_pose3", PoseStamped)
        rospy.loginfo("[linear_move_relative] LinearMoveRelative loaded.")

    #############################################################################
    # Transform goal's desired location to a pose in the wrist frame such that
    # the offset frame will be located at the goal
    def transform_pose_for_wrist(self, goal, offset_frame):
        # get goal in torso frame D_torso
        self.tf_listener.waitForTransform("torso_lift_link", goal.goal_pose.header.frame_id,
                                             rospy.Time(0), rospy.Duration(5))
        goal.goal_pose.header.stamp = rospy.Time()
        t_goal_f = self.tf_listener.transformPose("torso_lift_link", goal.goal_pose)
        # get toolframe_B_wristframe
        f_pos_w, f_quat_w = self.tf_listener.lookupTransform(offset_frame, 
                                                   self.arm + "_wrist_roll_link", rospy.Time())
        # find the wrist goal transformed from the tool frame goal
        t_goal_w_mat = (pose_msg_to_mat(t_goal_f) * pose_pq_to_mat(f_pos_w, f_quat_w))
        t_goal_w = PoseStamped()
        t_goal_w.header.frame_id = "torso_lift_link"
        t_goal_w.header.stamp = rospy.Time.now() + rospy.Duration(0.3)
        t_goal_w.pose = pose_mat_to_msg(t_goal_w_mat)
        return t_goal_w

    def wait_arm_moving(self, stop_wait=True):
        """ Wait for the arm to start/stop moving 
        """
        if stop_wait:
            rospy.loginfo("[linear_move_relative] Waiting for arm to stop moving.")
        else:
            rospy.loginfo("[linear_move_relative] Waiting for arm to start moving.")
        while not rospy.is_shutdown():
            if stop_wait:
                if not self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                    break
            else:
                if self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                    break
            rospy.sleep(0.05)
        if stop_wait:
            rospy.loginfo("[linear_move_relative] Arm has stopped moving.")
        else:
            rospy.loginfo("[linear_move_relative] Arm has started moving.")

    def execute_move(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement.")
        result = LinearMoveRelativeResult()

        torso_B_frame = self.lin_move.get_transform("torso_lift_link", 
                                                    goal.goal_pose.header.frame_id)
        frame_B_goal = pose_msg_to_mat(goal.goal_pose)
        appr_B_tool = self.lin_move.get_transform(self.tool_approach_frame, self.tool_frame) 
        torso_B_tool_appr = torso_B_frame * frame_B_goal * appr_B_tool
        torso_B_tool_goal = torso_B_frame * frame_B_goal

        self.wait_arm_moving(stop_wait=True)

        move_success = self.lin_move.linear_trajectory3(torso_B_tool_appr, torso_B_tool_goal, 
                                                        self.tool_frame, velocity=0.005, rate=5)
        self.move_arm_act.set_succeeded(result)
        return
        
        if not move_success:
            rospy.loginfo("[linear_move_relative] Relative arm movement failed.")
            self.move_arm_act.set_aborted(move_success)
            return

        self.wait_arm_moving(stop_wait=False)

        self.start_detection("overhead_grasp", 1.0)

        rospy.sleep(2)
        self.wait_arm_moving(stop_wait=True)
        self.stop_detection()
        rospy.loginfo("[linear_move_relative] Relative arm movement complete.")

        result.collided = self.coll_state.collided
        self.move_arm_act.set_succeeded(result)
        self.coll_state.collided = False

    def execute_setup(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement setup.")
        result = LinearMoveRelativeSetupResult()

        torso_B_frame = self.lin_move.get_transform("torso_lift_link", 
                                                    goal.goal_pose.header.frame_id)
        frame_B_goal = pose_msg_to_mat(goal.goal_pose)
        appr_B_tool = self.lin_move.get_transform(self.tool_approach_frame, self.tool_frame) 
        torso_B_tool_appr = torso_B_frame * frame_B_goal * appr_B_tool
        appr_B_wrist = self.lin_move.get_transform(self.tool_approach_frame, 
                                                  self.arm + "_wrist_roll_link") 
        torso_B_wrist = torso_B_frame * frame_B_goal * appr_B_wrist

        move_goal = PoseStamped()
        move_goal.header.frame_id = "torso_lift_link"
        move_goal.header.stamp = rospy.Time.now()
        move_goal.pose = pose_mat_to_msg(torso_B_tool_appr)
        self.pix3_pub.publish(move_goal)

        self.wait_arm_moving(stop_wait=True)

        angles = self.lin_move.biased_IK(torso_B_wrist, self.INIT_ANGS, 
                                         self.JOINTS_BIAS, num_iters=6)
        self.lin_move.command_angles(angles, 3.0)
        rospy.sleep(2)
        self.wait_arm_moving(stop_wait=True)

        move_success = self.lin_move.move_to_pose2(torso_B_tool_appr, self.tool_frame, 
                                                  velocity=0.03, rate=5)

        self.move_arm_setup_act.set_succeeded(result)
        return

        # TODO THIS STUFF ADDED BACK
        self.wait_arm_moving(stop_wait=False)

        self.start_detection("overhead_grasp", 0.99999)

        rospy.sleep(2)
        self.wait_arm_moving(stop_wait=True)
        self.stop_detection()
        rospy.loginfo("[linear_move_relative] Relative arm movement setup complete.")

        result.collided = self.coll_state.collided
        self.move_arm_setup_act.set_succeeded(result)
        self.coll_state.collided = False


def main():
    rospy.init_node("linear_arm_move", anonymous=True)
    lmr = LinearMoveRelative()
    rospy.spin()

if __name__ == "__main__":
    main()
