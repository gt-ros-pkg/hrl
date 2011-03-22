#######################################################################
#
#   USE pr2_object_manipulation/pr2_gripper_reactive_approach/controller_manager.py
#   That code has much of the ideas at the bottom, with more.
#
#######################################################################






# TODO Update code to throw points one at a time.  Sections are labled: "Hack"

import numpy as np, math
from threading import RLock, Timer
import sys

import roslib; roslib.load_manifest('hrl_pr2_lib')
roslib.load_manifest('force_torque') # hack by Advait
import force_torque.FTClient as ftc
import tf

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus

from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from teleop_controllers.msg import JTTeleopControllerState

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import hrl_lib.transforms as tr
import time
import functools as ft

import tf.transformations as tftrans
import operator as op
import types

from visualization_msgs.msg import Marker


node_name = "pr2_arms" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

##
# Convert arrays, lists, matricies to column format.
#
# @param x the unknown format
# @return a column matrix
def make_column(x):
    if (type(x) == type([]) 
        or (type(x) == np.ndarray and x.ndim == 1)
        or type(x) == type(())):
        return np.matrix(x).T
    if type(x) == np.ndarray:
        x = np.matrix(x)
    if x.shape[0] == 1:
        return x.T
    return x

##
# Convert column matrix to list
#
# @param col the column matrix
# @return the list
def make_list(col):
    if type(col) == type([]) or type(col) == type(()):
        return col
    return [col[i,0] for i in range(col.shape[0])]

##
# Class for simple management of the arms and grippers.
# Provides functionality for moving the arms, opening and closing
# the grippers, performing IK, and other functionality as it is
# developed.
class PR2Arms(object):

    ##
    # Initializes all of the servers, clients, and variables
    #
    # @param send_delay send trajectory points send_delay nanoseconds into the future
    # @param gripper_point given the frame of the wrist_roll_link, this point offsets
    #                      the location used in FK and IK, preferably to the tip of the
    #                      gripper
    def __init__(self, send_delay=50000000, gripper_point=(0.23,0.0,0.0),
                 force_torque = False):
        log("Loading PR2Arms")

        self.send_delay = send_delay
        self.off_point = gripper_point
        self.joint_names_list = [['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint'],
                           ['l_shoulder_pan_joint',
                           'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                           'l_elbow_flex_joint', 'l_forearm_roll_joint',
                           'l_wrist_flex_joint', 'l_wrist_roll_joint']]

        rospy.wait_for_service('pr2_right_arm_kinematics/get_fk');
        self.fk_srv = [rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk', GetPositionFK),
                       rospy.ServiceProxy('pr2_left_arm_kinematics/get_fk', GetPositionFK)] 

        rospy.wait_for_service('pr2_right_arm_kinematics/get_ik');
        self.ik_srv = [rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK),
                       rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik', GetPositionIK)]

        self.joint_action_client = [actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction),
                actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)]

        self.gripper_action_client = [actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction),actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)]

        self.joint_action_client[0].wait_for_server()
        self.joint_action_client[1].wait_for_server()
        self.gripper_action_client[0].wait_for_server()
        self.gripper_action_client[1].wait_for_server()

        self.arm_state_lock = [RLock(), RLock()]
        self.r_arm_cart_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
        self.l_arm_cart_pub = rospy.Publisher('/l_cart/command_pose', PoseStamped)

        rospy.Subscriber('/r_cart/state', JTTeleopControllerState, self.r_cart_state_cb)
        rospy.Subscriber('/l_cart/state', JTTeleopControllerState, self.l_cart_state_cb)

        if force_torque:
            self.r_arm_ftc = ftc.FTClient('force_torque_ft2')
        self.tf_lstnr = tf.TransformListener()

        self.arm_angles = [None, None]
        self.arm_efforts = [None, None]
        self.jtg = [None, None]
        self.cur_traj = [None, None]
        self.cur_traj_timer = [None, None]
        self.cur_traj_pos = [None, None]

        self.marker_pub = rospy.Publisher('/pr2_arms/viz_marker', Marker)
        rospy.Subscriber('/joint_states', JointState,
                         self.joint_states_cb, queue_size=2)

        rospy.sleep(1.)

        log("Finished loading SimpleArmManger")

    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    #
    # @param data JointState message recieved from the /joint_states topic
    def joint_states_cb(self, data):
        arm_angles = [[], []]
        arm_efforts = [[], []]
        r_jt_idx_list = [0]*7
        l_jt_idx_list = [0]*7
        for i, jt_nm in enumerate(self.joint_names_list[0]):
            r_jt_idx_list[i] = data.name.index(jt_nm)
        for i, jt_nm in enumerate(self.joint_names_list[1]):
            l_jt_idx_list[i] = data.name.index(jt_nm)

        for i in range(7):
            idx = r_jt_idx_list[i]
            if data.name[idx] != self.joint_names_list[0][i]:
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            ang = self.normalize_ang(data.position[idx])
            arm_angles[0] += [ang]
            arm_efforts[0] += [data.effort[idx]]

            idx = l_jt_idx_list[i]
            if data.name[idx] != self.joint_names_list[1][i]:
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            ang = self.normalize_ang(data.position[idx])
            arm_angles[1] += [ang]
            arm_efforts[1] += [data.effort[idx]]

        self.arm_state_lock[0].acquire()
        self.arm_angles[0] = arm_angles[0]
        self.arm_efforts[0] = arm_efforts[0]
        self.arm_state_lock[0].release()

        self.arm_state_lock[1].acquire()
        self.arm_angles[1] = arm_angles[1]
        self.arm_efforts[1] = arm_efforts[1]
        self.arm_state_lock[1].release()

    def r_cart_state_cb(self, msg):
        trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                 'r_gripper_tool_frame', rospy.Time(0))
        rot = tr.quaternion_to_matrix(quat)
        tip = np.matrix([0.12, 0., 0.]).T
        self.r_ee_pos = rot*tip + np.matrix(trans).T
        self.r_ee_rot = rot


        marker = Marker()
        marker.header.frame_id = 'torso_lift_link'
        time_stamp = rospy.Time.now()
        marker.header.stamp = time_stamp
        marker.ns = 'aloha land'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.r_ee_pos[0,0]
        marker.pose.position.y = self.r_ee_pos[1,0]
        marker.pose.position.z = self.r_ee_pos[2,0]
        size = 0.02
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.lifetime = rospy.Duration()

        marker.id = 71
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        color = (0.5, 0., 0.0)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.
        self.marker_pub.publish(marker)



        
        ros_pt = msg.x_desi_filtered.pose.position
        x, y, z = ros_pt.x, ros_pt.y, ros_pt.z
        self.r_cep_pos = np.matrix([x, y, z]).T
        pt = rot.T * (np.matrix([x,y,z]).T - np.matrix(trans).T)
        pt = pt + tip
        self.r_cep_pos_hooktip = rot*pt + np.matrix(trans).T
        ros_quat = msg.x_desi_filtered.pose.orientation
        quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
        self.r_cep_rot = tr.quaternion_to_matrix(quat)

    def l_cart_state_cb(self, msg):
        ros_pt = msg.x_desi_filtered.pose.position
        self.l_cep_pos = np.matrix([ros_pt.x, ros_pt.y, ros_pt.z]).T
        ros_quat = msg.x_desi_filtered.pose.orientation
        quat = (ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w)
        self.l_cep_rot = tr.quaternion_to_matrix(quat)

    def normalize_ang(self, ang):
        while ang >= 2 * np.pi:
            ang -= 2 * np.pi
        while ang < 0.:
            ang += 2 * np.pi
        return ang

    ##
    # Create a joint configuration trajectory goal.
    #
    # @param arm 0 for right, 1 for left
    # @param pos_arr list of lists of 7 joint angles in RADIANS.
    # @param dur_arr list of how long (SECONDS) from the beginning of the trajectory
    #                before reaching the joint angles.
    # @param stamp header (rospy.Duration) stamp to give the trajectory 
    # @param vel_arr list of lists of 7 joint velocities in RADIANS/sec.
    # @param acc_arr list of lists of 7 joint accelerations in RADIANS/sec^2.
    def create_JTG(self, arm, pos_arr, dur_arr, stamp=None, vel_arr=None, acc_arr=None):
        # Hack
        vel_arr = [[0.]*7]*len(pos_arr)
        acc_arr = [[0.]*7]*len(pos_arr)

        ##
        # Compute joint velocities and acclereations.
        def get_vel_acc(q_arr, d_arr):
            vel_arr = [[0.]*7]
            acc_arr = [[0.]*7]
            for i in range(1, len(q_arr)):
                vel, acc = [], []
                for j in range(7):
                    vel += [(q_arr[i][j] - q_arr[i-1][j]) / d_arr[i]]
                    acc += [(vel[j] - vel_arr[i-1][j]) / d_arr[i]]
                vel_arr += [vel]
                acc_arr += [acc]
                print vel, acc
            return vel_arr, acc_arr

        if arm != 1:
            arm = 0
        jtg = JointTrajectoryGoal()
        if stamp is None:
            stamp = rospy.Time.now()
        else:
            jtg.trajectory.header.stamp = stamp

        if len(pos_arr) > 1 and (vel_arr is None or acc_arr is None):
            v_arr, a_arr = get_vel_acc(pos_arr, dur_arr)
            if vel_arr is None:
                vel_arr = v_arr
            if acc_arr is None:
                acc_arr = a_arr

        jtg.trajectory.joint_names = self.joint_names_list[arm]
        for i in range(len(pos_arr)):
            if pos_arr[i] is None or type(pos_arr[i]) is types.NoneType:
                continue
            jtp = JointTrajectoryPoint()
            jtp.positions = pos_arr[i]
            if vel_arr is None:
                vel = [0.] * 7
            else:
                vel = vel_arr[i]
            jtp.velocities = vel
            if acc_arr is None:
                acc = [0.] * 7
            else:
                acc = acc_arr[i]
            jtp.accelerations = acc
            jtp.time_from_start = rospy.Duration(dur_arr[i])
            jtg.trajectory.points.append(jtp)
        return jtg

    ##
    # Executes a joint trajectory goal. This is the only function through which
    # arm motion is performed.
    # 
    # @param arm 0 for right, 1 for left
    # @param jtg the joint trajectory goal to execute
    def execute_trajectory(self, arm, jtg):
        if self.cur_traj[arm] is not None or self.cur_traj_timer[arm] is not None:
            log("Arm is currently executing trajectory")
            if rospy.is_shutdown():
                sys.exit()
            return
        print "Yo 2"

        # Hack
        # self.cur_traj[arm] = jtg 
        self.cur_traj_pos[arm] = 0

        # if too far in past, shift forward the time the trajectory starts
        min_init_time = rospy.Time.now().to_sec() + 2 * self.send_delay
        if jtg.trajectory.header.stamp.to_nsec() < min_init_time:
            jtg.trajectory.header.stamp = rospy.Time(rospy.Time.now().to_sec(), 
                                                         2 * self.send_delay)

        print "dfsdfsfd", jtg
        jtg.trajectory.header.stamp = rospy.Time.now()
        self.joint_action_client[arm].send_goal(jtg) # Hack
        return

        # setup first point throw
        call_time = ((jtg.trajectory.header.stamp.to_nsec() - self.send_delay -
                      rospy.Time.now().to_nsec()) / 1000000000.)
        self.cur_traj_timer[arm] = Timer(call_time, self._exec_traj, [arm])
        self.cur_traj_timer[arm].start()

	
    ##
    # Callback for periodic joint trajectory point throwing
    def _exec_traj(self, arm):
        jtg = self.cur_traj[arm]
        i = self.cur_traj_pos[arm]
        beg_time = jtg.trajectory.header.stamp.to_nsec()

        # time to execute current point
        cur_exec_time = rospy.Time(jtg.trajectory.header.stamp.to_sec() +
                                   jtg.trajectory.points[i].time_from_start.to_sec())

        # create a one point joint trajectory and send it
        if i == 0:
            last_time_from = 0
        else:
            last_time_from = jtg.trajectory.points[i-1].time_from_start.to_sec()
        cur_dur = jtg.trajectory.points[i].time_from_start.to_sec() - last_time_from
        cur_jtg = self.create_JTG(arm, [jtg.trajectory.points[i].positions],
                                       [cur_dur],
                                       cur_exec_time,
                                       [jtg.trajectory.points[i].velocities],
                                       [jtg.trajectory.points[i].accelerations])
        # send trajectory goal to node
        print "cur_jtg", cur_jtg
        self.joint_action_client[arm].send_goal(cur_jtg)

        self.cur_traj_pos[arm] += 1
        if self.cur_traj_pos[arm] == len(jtg.trajectory.points):
            # end trajectory
            self.cur_traj[arm] = None
            self.cur_traj_timer[arm] = None
        else:
            # setup next point throw
            next_exec_time = beg_time + jtg.trajectory.points[i+1].time_from_start.to_nsec()
            print "diff times", next_exec_time / 1000000000. - cur_exec_time.to_sec() - cur_dur
            call_time = ((next_exec_time - self.send_delay -
                          rospy.Time.now().to_nsec()) / 1000000000.)
            self.cur_traj_timer[arm] = Timer(call_time, self._exec_traj, [arm])
            self.cur_traj_timer[arm].start()

    ##
    # Stop the current arm trajectory.
    #
    # @param arm 0 for right, 1 for left
    def stop_trajectory(self, arm):
        self.cur_traj_timer[arm].cancel()
        self.cur_traj[arm] = None
        self.cur_traj_timer[arm] = None

    ##
    # Move the arm to a joint configuration.
    #
    # @param arm 0 for right, 1 for left
    # @param q list of 7 joint angles in RADIANS.
    # @param start_time time (in secs) from function call to start action
    # @param duration how long (SECONDS) before reaching the joint angles.
    def set_joint_angles(self, arm, q, duration=1., start_time=0.):
        self.set_joint_angles_traj(arm, [q], [duration], start_time)

    ##
    # Move the arm through a joint configuration trajectory goal.
    #
    # @param arm 0 for right, 1 for left
    # @param q_arr list of lists of 7 joint angles in RADIANS.
    # @param dur_arr list of how long (SECONDS) from the beginning of the trajectory
    #                before reaching the joint angles.
    # @param start_time time (in secs) from function call to start action
    def set_joint_angles_traj(self, arm, q_arr, dur_arr, start_time=0.):
        if arm != 1:
            arm = 0
        jtg = self.create_JTG(arm, q_arr, dur_arr)
        cur_time = rospy.Time.now().to_sec() + 2 * self.send_delay / 1000000000.
        jtg.trajectory.header.stamp = rospy.Duration(start_time + cur_time)
        self.execute_trajectory(arm, jtg)

    ##
    # Is the arm currently moving?
    #
    # @param arm 0 for right, 1 for left
    # @return True if moving, else False
    def is_arm_in_motion(self, arm):
        # Hack
        # if self.cur_traj[arm] is not None:
        #     return True
        state = self.joint_action_client[arm].get_state()
        return state == GoalStatus.PENDING or state == GoalStatus.ACTIVE

    ##
    # Block until the arm has finished moving
    # 
    # @param arm 0 for right, 1 for left
    # @param hz how often to check
    def wait_for_arm_completion(self, arm, hz=0.01):
        while self.is_arm_in_motion(arm) and not rospy.is_shutdown():
            rospy.sleep(hz)

    ##
    # Transforms the given position by the offset position in the given quaternion
    # rotation frame
    #
    # @param pos the current positions
    # @param quat quaternion representing the rotation of the frame
    # @param off_point offset to move the position inside the quat's frame
    # @return the new position as a matrix column
    def transform_in_frame(self, pos, quat, off_point):
        pos = make_column(pos)    
        quat = make_list(quat)
        invquatmat = np.mat(tftrans.quaternion_matrix(quat))
        invquatmat[0:3,3] = pos 
        trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
        transpos = invquatmat * trans
        return np.resize(transpos, (3, 1))

    ##
    # Performs Forward Kinematics on the given joint angles
    #
    # @param arm 0 for right, 1 for left
    # @param q list of 7 joint angles in radians
    # @return column matrix of cartesian position, column matrix of quaternion
    def FK(self, arm, q):
        if rospy.is_shutdown():
            sys.exit()
        if arm != 1:
            arm = 0
        fk_req = GetPositionFKRequest()
        fk_req.header.frame_id = 'torso_lift_link'
        if arm == 0:
            fk_req.fk_link_names.append('r_wrist_roll_link') 
        else:
            fk_req.fk_link_names.append('l_wrist_roll_link')
        fk_req.robot_state.joint_state.name = self.joint_names_list[arm]
        fk_req.robot_state.joint_state.position = q

        fk_resp = GetPositionFKResponse()
        fk_resp = self.fk_srv[arm].call(fk_req)
        if fk_resp.error_code.val == fk_resp.error_code.SUCCESS:
            x = fk_resp.pose_stamped[0].pose.position.x
            y = fk_resp.pose_stamped[0].pose.position.y
            z = fk_resp.pose_stamped[0].pose.position.z
            q1 = fk_resp.pose_stamped[0].pose.orientation.x
            q2 = fk_resp.pose_stamped[0].pose.orientation.y
            q3 = fk_resp.pose_stamped[0].pose.orientation.z
            q4 = fk_resp.pose_stamped[0].pose.orientation.w
            quat = [q1,q2,q3,q4]
            
            # Transform point from wrist roll link to actuator
            ret1 = self.transform_in_frame([x,y,z], quat, self.off_point)
            ret2 = np.matrix(quat).T 
        else:
            rospy.logerr('Forward kinematics failed')
            ret1, ret2 = None, None

        return ret1, ret2
    
    ##
    # Performs Inverse Kinematics on the given position and rotation
    #
    # @param arm 0 for right, 1 for left
    # @param p cartesian position in torso_lift_link frame
    # @param rot quaternion rotation column or rotation matrix 
    #                                            of wrist in torso_lift_link frame
    # @param q_guess initial joint angles to use for finding IK
    def IK(self, arm, p, rot, q_guess):
        if arm != 1:
            arm = 0

        p = make_column(p)

        if rot.shape == (3, 3):
            quat = np.matrix(tr.matrix_to_quaternion(rot)).T
        elif rot.shape == (4, 1):
            quat = make_column(rot)
        else:
            rospy.logerr('Inverse kinematics failed (bad rotation)')
            return None

        # Transform point back to wrist roll link
        neg_off = [-self.off_point[0],-self.off_point[1],-self.off_point[2]]
        transpos = self.transform_in_frame(p, quat, neg_off)
        
        ik_req = GetPositionIKRequest()
        ik_req.timeout = rospy.Duration(5.)
        if arm == 0:
            ik_req.ik_request.ik_link_name = 'r_wrist_roll_link'
        else:
            ik_req.ik_request.ik_link_name = 'l_wrist_roll_link'
        ik_req.ik_request.pose_stamped.header.frame_id = 'torso_lift_link'

        ik_req.ik_request.pose_stamped.pose.position.x = transpos[0] 
        ik_req.ik_request.pose_stamped.pose.position.y = transpos[1]
        ik_req.ik_request.pose_stamped.pose.position.z = transpos[2]

        ik_req.ik_request.pose_stamped.pose.orientation.x = quat[0]
        ik_req.ik_request.pose_stamped.pose.orientation.y = quat[1]
        ik_req.ik_request.pose_stamped.pose.orientation.z = quat[2]
        ik_req.ik_request.pose_stamped.pose.orientation.w = quat[3]

        ik_req.ik_request.ik_seed_state.joint_state.position = q_guess
        ik_req.ik_request.ik_seed_state.joint_state.name = self.joint_names_list[arm]

        ik_resp = self.ik_srv[arm].call(ik_req)
        if ik_resp.error_code.val == ik_resp.error_code.SUCCESS:
            ret = list(ik_resp.solution.joint_state.position)
        else:
            rospy.logerr('Inverse kinematics failed')
            ret = None

        return ret

    ##
    # Evaluates IK for the given position and rotation to see if the arm could move
    # @param arm 0 for right, 1 for left
    # @param p cartesian position in torso_lift_link frame
    # @param rot quaternion rotation column or rotation matrix 
    #                                            of wrist in torso_lift_link frame
    def can_move_arm(self, arm, pos, rot):
        begq = self.get_joint_angles(arm)
        endq = self.IK(arm, pos, rot, begq)
        if endq is None:
            return False
        else:
            return True

    ##
    # Moves the arm to the given position and rotation
    #
    # @param arm 0 for right, 1 for left
    # @param pos cartesian position of end effector
    # @param rot quaterninon or rotation matrix of the end effector
    # @param dur length of time to do the motion in
    def move_arm(self, arm, pos, begq=None , rot=None, dur=4.0):
        if begq is None:
			begq = self.get_joint_angles(arm)
        if rot is None:
            temp, rot = self.FK(arm, begq)
        endq = self.IK(arm, pos, rot, begq)
        if endq is None:
            return False
        self.set_joint_angles(arm, endq, dur)
        return True
	
    def cancel_trajectory(self, arm):
        self.joint_action_client[arm].cancel_all_goals()

    ##
    # Move the arm through a trajectory defined by a series of positions and rotations
    #
    # @param arm 0 for right, 1 for left
    # @param pos_arr list of positions to achieve during trajectory
    # @param dur length of time to execute all trajectories in, all positions will
    #            be given the same amount of time to reach the next point
    #            (ignore if dur_arr is not None)
    # @param rot_arr achieve these rotations along with those positions
    #                if None, maintain original rotation
    # @param dur_arr use these times for the time positions
    # @param freeze_wrist if True, keeps the rotation of the wrist constant
    def move_arm_trajectory(self, arm, pos_arr, dur=1.,
                            rot_arr=None, dur_arr=None, freeze_wrist=False):
        if dur_arr is None:
            dur_arr = np.linspace(0., dur, len(pos_arr) + 1)[1:]
        
        initq = self.get_joint_angles(arm)
        curq = initq
        q_arr = []
        fails, trys = 0, 0
        if rot_arr is None:
            temp, rot = self.FK(arm, curq)
        
        for i in range(len(pos_arr)):
            if rot_arr is None:
                cur_rot = rot
            else:
                cur_rot = rot_arr[i]
            nextq = self.IK(arm, pos_arr[i], cur_rot, curq)
            q_arr += [nextq]
            if nextq is None:
                fails += 1
            trys += 1
            if not nextq is None:
                curq = nextq
        log("IK Accuracy: %d/%d (%f)" % (trys-fails, trys, (trys-fails)/float(trys)))
        
        if freeze_wrist:
            for i in range(len(q_arr)):
                if not q_arr[i] is None:
                    q_arr[i] = (q_arr[i][0], q_arr[i][1], q_arr[i][2], q_arr[i][3],
                                q_arr[i][4], q_arr[i][5], initq[6])

        self.set_joint_angles_traj(arm, q_arr, dur_arr)
    
    ##
    # Returns the current position, rotation of the arm.
    #
    # @param arm 0 for right, 1 for left
    # @return rotation, position
    def end_effector_pos(self, arm):
        q = self.get_joint_angles(arm)
        return self.FK(arm, q)

    ##
    # Returns the list of 7 joint angles
    #
    # @param arm 0 for right, 1 for left
    # @return list of 7 joint angles
    def get_joint_angles(self, arm):
        if arm != 1:
            arm = 0
        self.arm_state_lock[arm].acquire()
        q = self.arm_angles[arm]
        self.arm_state_lock[arm].release()
        return q


    # force that is being applied on the wrist.
    def get_wrist_force(self, arm, bias = True, base_frame = False):
        if arm != 0:
            rospy.logerr('Unsupported arm: %d'%arm)
            raise RuntimeError('Unimplemented function')
 
        f = self.r_arm_ftc.read(without_bias = not bias)
        f = f[0:3, :]
        if base_frame:
            trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                '/ft2', rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            f = rot * f
        return -f # the negative is intentional (Advait, Nov 24. 2010.)

    def bias_wrist_ft(self, arm):
       if arm != 0:
           rospy.logerr('Unsupported arm: %d'%arm)
           raise RuntimeError('Unimplemented function')
       self.r_arm_ftc.bias()


    def get_ee_jtt(self, arm):
        if arm == 0:
            return self.r_ee_pos, self.r_ee_rot
        else:
            return self.l_ee_pos, self.l_ee_rot

    def get_cep_jtt(self, arm, hook_tip = False):
        if arm == 0:
            if hook_tip:
                return self.r_cep_pos_hooktip, self.r_cep_rot
            else:
                return self.r_cep_pos, self.r_cep_rot
        else:
            return self.l_cep_pos, self.l_cep_rot

    # set a cep using the Jacobian Transpose controller.
    def set_cep_jtt(self, arm, p, rot=None):
        if arm != 1:
            arm = 0
        ps = PoseStamped()
        ps.header.stamp = rospy.rostime.get_rostime()
        ps.header.frame_id = 'torso_lift_link'
 
        ps.pose.position.x = p[0,0]
        ps.pose.position.y = p[1,0]
        ps.pose.position.z = p[2,0]
 
        if rot == None:
            if arm == 0:
                rot = self.r_cep_rot
            else:
                rot = self.l_cep_rot

        quat = tr.matrix_to_quaternion(rot)
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        if arm == 0:
            self.r_arm_cart_pub.publish(ps)
        else:
            self.l_arm_cart_pub.publish(ps)

    # rotational interpolation unimplemented.
    def go_cep_jtt(self, arm, p):
        step_size = 0.01
        sleep_time = 0.1
        cep_p, cep_rot = self.get_cep_jtt(arm)
        unit_vec = (p-cep_p)
        unit_vec = unit_vec / np.linalg.norm(unit_vec)
        while np.linalg.norm(p-cep_p) > step_size:
            cep_p += unit_vec * step_size
            self.set_cep_jtt(arm, cep_p)
            rospy.sleep(sleep_time)
        self.set_cep_jtt(arm, p)
        rospy.sleep(sleep_time)



# TODO Evaluate gripper functions and parameters

    ##
    # Move the gripper the given amount with given amount of effort
    #
    # @param arm 0 for right, 1 for left
    # @param amount the amount the gripper should be opened
    # @param effort - supposed to be in Newtons. (-ve number => max effort)
    def move_gripper(self, arm, amount=0.08, effort = 15):
        self.gripper_action_client[arm].send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=amount, max_effort = effort)))

    ##
    # Open the gripper
    #
    # @param arm 0 for right, 1 for left
    def open_gripper(self, arm):
        self.move_gripper(arm, 0.08, -1)

    ##
    # Close the gripper
    #
    # @param arm 0 for right, 1 for left
    def close_gripper(self, arm, effort = 15):
        self.move_gripper(arm, 0.0, effort)

    # def get_wrist_force(self, arm):
    #     pass

    ######################################################
    # More specific functionality
    ######################################################

    ##
    # Moves arm smoothly through a linear trajectory.
    #
    # @param arm
    def smooth_linear_arm_trajectory(self, arm, dist, dir=(0.,0.,-1.), max_jerk=0.25, 
                                     delta=0.005, dur=None,
                                     freeze_wrist=True, is_grasp_biased=True):

        ####################################################
        # Smooth trajectory functions
        def _smooth_traj_pos(t, k, T):
            return -k / T**3 * np.sin(T * t) + k / T**2 * t

        def _smooth_traj_vel(t, k, T):
            return -k / T**2 * np.cos(T * t) + k / T**2 
        
        def _smooth_traj_acc(t, k, T):
            return k / T * np.sin(T * t) 

        # length of time of the trajectory
        def _smooth_traj_time(l, k):
            return np.power(4 * np.pi**2 * l / k, 1./3.)

        def _interpolate_traj(traj, k, T, num=10, begt=0., endt=1.):
            return [traj(t,k,T) for t in np.linspace(begt, endt, num)]

        ####################################################

        # Vector representing full transform of end effector
        traj_vec = [x/np.sqrt(np.vdot(dir,dir)) for x in dir]
        # number of steps to interpolate trajectory over
        num_steps = dist / delta
        # period of the trajectory 
        trajt = _smooth_traj_time(dist, max_jerk)
        period = 2. * np.pi / trajt

        # break the trajectory into interpolated parameterized function
        # from 0 to length of the trajectory
        traj_pos_mult = _interpolate_traj(_smooth_traj_pos, max_jerk, period, 
                                          num_steps, 0., trajt) 
#       traj_vel_mult = _interpolate_traj(_smooth_traj_vel, max_jerk, period, 
#                                         num_steps, 0., trajt) 
#       traj_acc_mult = _interpolate_traj(_smooth_traj_acc, max_jerk, period, 
#                                         num_steps, 0., trajt) 

        # cartesian offset points of trajectory
        cart_pos_traj = [(a*traj_vec[0],a*traj_vec[1],a*traj_vec[2]) for a in traj_pos_mult]
#       cart_vel_traj = [(a*traj_vec[0],a*traj_vec[1],a*traj_vec[2]) for a in traj_vel_mult]
#       cart_acc_traj = [(a*traj_vec[0],a*traj_vec[1],a*traj_vec[2]) for a in traj_acc_mult]

        cur_pos, cur_rot = self.FK(arm, self.get_joint_angles(arm))
        # get actual positions of the arm by transforming the offsets
        arm_traj = [(ct[0] + cur_pos[0], ct[1] + cur_pos[1],
                                         ct[2] + cur_pos[2]) for ct in cart_pos_traj]
        if dur is None:
            dur = trajt

        if is_grasp_biased:
            self.grasp_biased_move_arm_trajectory(arm, arm_traj, dur, 
                                                  freeze_wrist=freeze_wrist)
        else:
            self.move_arm_trajectory(arm, arm_traj, dur, freeze_wrist=freeze_wrist)

        # return the expected time of the trajectory
        return dur

    def bias_guess(self, q, joints_bias, bias_radius):
        if bias_radius == 0.0:
            return q
        max_angs = np.array([.69, 1.33, 0.79, 0.0, 1000000.0, 0.0, 1000000.0])
        min_angs = np.array([-2.27, -.54, -3.9, -2.34, -1000000.0, -2.15, -1000000.0])
        q_off = bias_radius * np.array(joints_bias) / np.linalg.norm(joints_bias)
        angs = np.array(q) + q_off
        for i in range(7):
            if angs[i] > max_angs[i]:
                angs[i] = max_angs[i]
            elif angs[i] < min_angs[i]:
                angs[i] = min_angs[i]
        return angs.tolist()

    ##
    # Same as move_arm but tries to keep the elbow up.
    def grasp_biased_IK(self, arm, pos, rot, q_guess, joints_bias=[0.]*7, bias_radius=0., num_iters=5):
        angs = q_guess
        for i in range(num_iters):
            angs = self.IK(arm, pos, rot, angs)
            angs = self.bias_guess(angs, joints_bias, bias_radius)
        return self.IK(arm, pos, rot, angs)

    ##
    # Same as move_arm but tries to keep the elbow up.
    def grasp_biased_move_arm(self, arm, pos, rot=None, dur=4.0, num_iters=20):
        if rot is None:
            temp, rot = self.FK(arm, begq)
        angs = self.get_joint_angles(arm)
        angs = self.grasp_biased_IK(arm, pos, rot, angs)
        self.set_joint_angles(arm, angs, dur)

    ##
    # Move the arm through a trajectory defined by a series of positions and rotations
    #
    # @param arm 0 for right, 1 for left
    # @param pos_arr list of positions to achieve during trajectory
    # @param dur length of time to execute all trajectories in, all positions will
    #            be given the same amount of time to reach the next point
    #            (ignore if dur_arr is not None)
    # @param rot_arr achieve these rotations along with those positions
    #                if None, maintain original rotation
    # @param dur_arr use these times for the time positions
    # @param freeze_wrist if True, keeps the rotation of the wrist constant
    def grasp_biased_move_arm_trajectory(self, arm, pos_arr, dur=1.,
                                         rot_arr=None, dur_arr=None, freeze_wrist=False):
        bias_vec = np.array([0., -1., -1., 0., 0., 1., 0.])
        q_radius = 0.012 # Calculated as q_radius = arctan( delta / wrist_flex_length )
        q_off = q_radius * bias_vec / np.linalg.norm(bias_vec)

        if dur_arr is None:
            dur_arr = np.linspace(0., dur, len(pos_arr) + 1)[1:]
        
        initq = self.get_joint_angles(arm)
        curq = initq
        q_arr = []
        fails, trys = 0, 0
        if rot_arr is None:
            temp, rot = self.FK(arm, curq)
        
        for i in range(len(pos_arr)):
            if rot_arr is None:
                cur_rot = rot
            else:
                cur_rot = rot_arr[i]
            q_guess = (np.array(curq) + q_off).tolist()
            nextq = self.IK(arm, pos_arr[i], cur_rot, q_guess)
            q_arr += [nextq]
            if nextq is None:
                fails += 1
            trys += 1
            if not nextq is None:
                curq = nextq
        log("IK Accuracy: %d/%d (%f)" % (trys-fails, trys, (trys-fails)/float(trys)))
        
        if freeze_wrist:
            for i in range(len(q_arr)):
                if not q_arr[i] is None:
                    q_arr[i] = (q_arr[i][0], q_arr[i][1], q_arr[i][2], q_arr[i][3],
                                q_arr[i][4], q_arr[i][5], initq[6])

        self.set_joint_angles_traj(arm, q_arr, dur_arr)

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

    pr2_arm = PR2Arms()

    if False:
        q = [0, 0, 0, 0, 0, 0, 0]
        pr2_arm.set_jointangles('right_arm', q)
        ee_pos = simparm.FK('right_arm', q)
        log('FK result:' + ee_pos.A1)

        ee_pos[0,0] -= 0.1
        q_ik = pr2_arm.IK('right_arm', ee_pos, tr.Rx(0.), q)
        log('q_ik:' + [math.degrees(a) for a in q_ik])

        rospy.spin()

    if False:
        p = np.matrix([0.9, -0.3, -0.15]).T
        #rot = tr.Rx(0.)
        rot = tr.Rx(math.radians(90.))
        pr2_arm.set_cartesian('right_arm', p, rot)

#    #------ testing gripper opening and closing ---------
#    raw_input('Hit ENTER to begin')
#    pr2_arm.open_gripper(0)
#    raw_input('Hit ENTER to close')
#    pr2_arm.close_gripper(0, effort = 15)


#    #------- testing set JEP ---------------
#    raw_input('Hit ENTER to begin')
    r_arm, l_arm = 0, 1
#    cep_p, cep_rot = pr2_arm.get_cep_jtt(r_arm)
#    print 'cep_p:', cep_p.A1
#
#    for i in range(5):
#        cep_p[0,0] += 0.01
#        raw_input('Hit ENTER to move')
#        pr2_arm.set_cep_jtt(r_arm, cep_p)

    raw_input('Hit ENTER to move')
    p1 = np.matrix([0.91, -0.22, -0.05]).T
    pr2_arm.go_cep_jtt(r_arm, p1)

    #rospy.sleep(10)
    #pr2_arm.close_gripper(r_arm, effort = -1)
    raw_input('Hit ENTER to move')
    p2 = np.matrix([0.70, -0.22, -0.05]).T
    pr2_arm.go_cep_jtt(r_arm, p2)







