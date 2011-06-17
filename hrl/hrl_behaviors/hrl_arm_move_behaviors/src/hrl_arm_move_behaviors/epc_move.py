import numpy as np
import copy

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans
import tf

from hrl_arm_move_behaviors.pr2_arm_base import PR2ArmBase
import hrl_arm_move_behaviors.util as util

##
# Classic PID controller with maximum integral thresholding to
# prevent windup.
class PIDController(object):
    def __init__(self, k_p, k_i, k_d, i_max, rate, name=None):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.i_max = i_max
        self.rate = rate
        self.err_last = 0
        self.integral = 0
        if name is not None:
            self.err_pub = rospy.Publisher("/pid_controller/" + name + "_error", Float64)
            self.out_pub = rospy.Publisher("/pid_controller/" + name + "_output", Float64)

    ##
    # Updates the controller state and returns the current output.
    def update_state(self, err):
        self.integral += err / self.rate
        self.integral = np.clip(self.integral, -self.i_max, self.i_max)
        y = (self.k_p * err + 
             self.k_d * (err - self.err_last) * self.rate +
             self.k_i * self.integral)
        self.err_last = err
        self.err_pub.publish(err)
        self.out_pub.publish(y)
        return y

    def reset_controller(self):
        self.err_last = 0
        self.integral = 0

class EPCMove(PR2ArmBase):
    ##
    # Initializes subscribers
    # @param arm 'r' for right, 'l' for left
    # @param tf_listener tf.TransformListener object if one already exists in the node
    def __init__(self, arm, tf_listener=None, rate=5):
        super(EPCMove, self).__init__(arm)
        if tf_listener is None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        self.rate = rate

        # magic numbers
        self.max_angles = 5*np.array([0.06, 0.08, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.u_pos_max = 0.2
        self.grav_comp = 0.02
        pos_p, pos_i, pos_d, pos_i_max = 0.24, 0.18, 0.07, 0.6
        ang_p, ang_i, ang_d, ang_i_max = 5.0, 0.1, 0.50, 6.0
        self.x_pid = PIDController(pos_p, pos_i, pos_d, pos_i_max, rate, "x")
        self.y_pid = PIDController(pos_p, pos_i, pos_d, pos_i_max, rate, "y")
        self.z_pid = PIDController(pos_p, pos_i, pos_d, pos_i_max, rate, "z")
        self.a_pid = PIDController(ang_p, ang_i, ang_d, ang_i_max, rate, "angle")

        # advertise informative topics
        self.cmd_pub = rospy.Publisher("/commanded_pose", PoseStamped)
        self.cur_pub = rospy.Publisher("/current_pose", PoseStamped)
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped)
        #u_pub = rospy.Publisher("/u_signal", Float64)
        #p_pub = rospy.Publisher("/p_signal", Float64)
        #i_pub = rospy.Publisher("/i_signal", Float64)
        #d_pub = rospy.Publisher("/d_signal", Float64)

        self.cur_angles = None

    def reset_controllers(self):
        self.x_pid.reset_controller()
        self.y_pid.reset_controller()
        self.z_pid.reset_controller()
        self.a_pid.reset_controller()
#       self.qx_pid.reset_controller()
#       self.qy_pid.reset_controller()
#       self.qz_pid.reset_controller()
#       self.qw_pid.reset_controller()

    ##
    # Returns the homogeneous matrix from_B_to
    def get_transform(self, from_frame, to_frame, time=rospy.Time()):
        pos, quat = self.tf_listener.lookupTransform(from_frame, to_frame, time)
        return util.pose_pq_to_mat(pos, quat)

    ##
    # Updates the controllers and returns a pose to command the tool next
    # @param Homogeneous matrix specifying the current goal
    # @return Next commanded pose, the position error column matrix, the angle error float
    def find_controlled_tool_pose(self, target_pose):
        # find current tool location
        t_B_c = self.get_transform("torso_lift_link", self.tool_frame)
        # find error in position
        err_pos = target_pose[:3,3] - t_B_c[:3,3]
        # find error in angle
        err_ang = util.quaternion_dist(target_pose, t_B_c)

        # find control values
        u_x = self.x_pid.update_state(err_pos[0,0])
        u_y = self.y_pid.update_state(err_pos[1,0])
        u_z = self.z_pid.update_state(err_pos[2,0]) + self.grav_comp
        u_pos = np.mat([u_x, u_y, u_z]).T
        u_a = self.a_pid.update_state(err_ang)
#       quat_diff = (np.array(tf_trans.quaternion_from_matrix(target_pose)) -
#                    np.array(tf_trans.quaternion_from_matrix(t_B_c)))
#       u_qx = self.qx_pid.update_state(quat_diff[0])
#       u_qy = self.qy_pid.update_state(quat_diff[1])
#       u_qz = self.qz_pid.update_state(quat_diff[2])
#       u_qw = self.qw_pid.update_state(quat_diff[3])
#       u_quat = np.array([u_qx, u_qy, u_qz, u_qw])
#       u_quat = u_quat / np.linalg.norm(u_quat)

        # find commanded frame of tool
        # rotation
        u_a = np.clip(u_a, 0, 1)
        ei_q_slerp = tf_trans.quaternion_slerp(tf_trans.quaternion_from_matrix(t_B_c),
                                               tf_trans.quaternion_from_matrix(target_pose),
                                               u_a)
        new_quat = ei_q_slerp
#       new_quat = tf_trans.quaternion_multiply(tf_trans.quaternion_from_matrix(t_B_c),
#                                               u_quat)
        t_B_new = np.mat(tf_trans.quaternion_matrix(new_quat))

        # position
        u_pos_clipped = np.clip(u_pos, -self.u_pos_max, self.u_pos_max)
        t_B_new[:3,3] = t_B_c[:3,3] + u_pos_clipped 

        # publish informative topics
        self.goal_pub.publish(util.pose_mat_to_stamped_msg("torso_lift_link", target_pose))
        self.cur_pub.publish(util.pose_mat_to_stamped_msg("torso_lift_link", t_B_c))
        self.cmd_pub.publish(util.pose_mat_to_stamped_msg("torso_lift_link", t_B_new))
        
        return t_B_new, err_pos, err_ang

    def ik_tool_pose(self, t_B_tool, use_search=False):
        # transform commanded frame into wrist
        l_B_w = self.get_transform(self.tool_frame, self.arm + '_wrist_roll_link')
        t_B_wrist = t_B_tool * l_B_w

        # find IK solution
        q_guess = self.get_joint_angles(wrapped=True)
        # TODO add bias option?
        cur_time = rospy.Time.now().to_sec()
        if use_search:
            q_sol = self.search_IK(t_B_wrist, q_guess, sigma=0.02, sample_size=10)
        else:
            q_sol = self.IK(t_B_wrist, q_guess)

        # ignore if no IK solution found
        if q_sol is None:
            # TODO better solution?
            rospy.logerr("No IK solution found")
            self.num_ik_not_found += 1
            return None
        else:
            self.num_ik_not_found = 0
        return q_sol

    def command_joints_safely(self, q_cmd):
        # Clamp angles so the won't exceed max_angles in this command
#if self.cur_angles is None:
#    self.cur_angles = self.get_joint_angles(wrapped=True)
        cur_angles = self.get_joint_angles(wrapped=True)
        if True:
            if np.any(np.fabs(self.angle_difference(cur_angles, q_cmd)) > self.max_angles):
                return False
            else:
                self.command_joint_angles(q_cmd, 1.2/self.rate)
                return True

        else:
            q_cmd_clamped = np.clip(q_cmd, cur_angles - self.max_angles, 
                                           cur_angles + self.max_angles)

        #diff = self.angle_difference(q_cmd, self.cur_angles)
        #max_diff_arg = np.argmax(np.fabs(diff) - np.array(self.max_angles))
        #diff_scaled = diff / diff[max_diff_arg] * self.max_angles[max_diff_arg]
        #print "CA", self.cur_angles
        #print "QC", q_cmd
        #print "D", diff
        #print "DS", diff_scaled
        #q_cmd_clamped = self.cur_angles + diff_scaled
        # command joints
#self.command_joint_angles(q_cmd_clamped, 1.2/self.rate)

    def direct_move(self, target_pose, tool_frame=None, 
                    err_pos_goal=0.02, err_ang_goal=0.35, 
                    err_pos_max=0.10, err_ang_max=1.0, steady_steps=5,
                    coll_detect_cb=None):
        if tool_frame is None or tool_frame == "":
            self.tool_frame = self.arm + "_gripper_tool_frame"
        else:
            self.tool_frame = tool_frame
        self.num_ik_not_found = 0
        steady_count = 0
        self.cur_angles = None
        while not rospy.is_shutdown():
            # find where the next commanded tool position should be
            t_B_new, err_pos, err_ang = self.find_controlled_tool_pose(target_pose)

            # run the collision detection callback to see if we should stop
            if coll_detect_cb is not None:
                result = coll_detect_cb(err_pos, err_ang)
                if result is not None:
                    rospy.loginfo("[epc_move] Callback function reported collision: '%s'"
                                                                                        % result)
                    return result

            # check to see if we have strayed too far from the goal
            err_pos_mag = np.linalg.norm(err_pos)
            if err_pos_mag > err_pos_max or err_ang > err_ang_max:
                rospy.loginfo("[epc_move] Controller error exceeded thresholds (pos: %f ang %f)" % (err_pos_mag, err_ang))
                return "error_high"

            # check to see if we have settled
            if err_pos_mag < err_pos_goal and err_ang < err_ang_goal:
                # we are close to the target, wait a few steps to make sure we're steady
                steady_count += 1
                if steady_count >= steady_steps:
                    return "succeeded"
            else:
                steady_count = 0

            # get the ik for the wrist
            q_cmd = self.ik_tool_pose(t_B_new, use_search=True)
            if q_cmd is None:
                if self.num_ik_not_found > 4:
                    rospy.logwarn("[epc_move] Controller has hit a rut, no IK solutions in area")
                    return "ik_failure"
                continue

            # command the joints to their positions (with safety checking)
            command_successful = self.command_joints_safely(q_cmd)
            if not command_successful:
                rospy.loginfo("[epc move] Max angles exceeded")
                continue

            print "Current error: Pos:", err_pos_mag, "Angle:", err_ang
            rospy.sleep(1.0/self.rate)

        return "shutdown"

    def linear_move(self, start_pose, end_pose, tool_frame=None, 
                    velocity=0.03, err_pos_max=0.10, err_ang_max=1.0, setup_steps=3, 
                    coll_detect_cb=None):
        if tool_frame is None or tool_frame == "":
            self.tool_frame = self.arm + "_gripper_tool_frame"
        else:
            self.tool_frame = tool_frame

        # create trajectory
        dist = np.linalg.norm(start_pose[:3,3] - end_pose[:3,3])
        traj_time = dist / velocity
        num_steps = traj_time * self.rate
        trajectory = util.interpolate_cartesian(start_pose, end_pose, num_steps)
        # add several steps of setup to the start position before continuing
        trajectory_setup = util.interpolate_cartesian(start_pose, start_pose, setup_steps)
        trajectory = trajectory_setup + trajectory

        self.num_ik_not_found = 0
        self.cur_angles = None
        for target_pose in trajectory:
            if rospy.is_shutdown():
                return "shutdown"

            # find where the next commanded tool position should be
            t_B_new, err_pos, err_ang = self.find_controlled_tool_pose(target_pose)

            # run the collision detection callback to see if we should stop
            if coll_detect_cb is not None:
                result = coll_detect_cb(err_pos, err_ang)
                if result is not None:
                    rospy.loginfo("[epc_move] Callback function reported collision: '%s'"
                                                                                        % result)
                    return result

            # check to see if we have strayed too far from the trajectory
            err_pos_mag = np.linalg.norm(err_pos)
            if err_pos_mag > err_pos_max or err_ang > err_ang_max:
                rospy.loginfo("[epc_move] Controller error exceeded thresholds (pos: %f ang %f)" % (err_pos_mag, err_ang))
                return "error_high"

            # get the ik for the wrist
            q_cmd = self.ik_tool_pose(t_B_new, use_search=False)
            if q_cmd is None:
                if self.num_ik_not_found > 4:
                    rospy.logwarn("[epc_move] Controller has hit a rut, no IK solutions in area")
                    return "ik_failure"
                continue

            # command the joints to their positions (with safety checking)
            command_successful = self.command_joints_safely(q_cmd)
            if not command_successful:
                rospy.loginfo("[epc move] Max angles exceeded")
                continue

            print "Current error: Pos:", err_pos_mag, "Angle:", err_ang
            rospy.sleep(1.0/self.rate)

        return "succeeded"
