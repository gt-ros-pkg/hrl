
import numpy as np, math
from threading import RLock, Timer
import sys

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus

from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from kinematics_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, JointTrajectoryControllerState
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import hrl_lib.transforms as tr
from hrl_lib.util import RateCaller
import time
import functools as ft

import tf.transformations as tftrans
import operator as op
import types

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
    def __init__(self, send_delay=50000000, gripper_point=(0.23, 0.0, 0.0)):
        log("Loading SimpleArmTrajectory")

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

        self.joint_nm_list = ['shoulder_pan', 'shoulder_lift', 'upper_arm_roll',
                              'elbow_flex', 'forearm_roll', 'wrist_flex',
                              'wrist_roll']

        self.arm_angles = [None, None]
        self.arm_efforts = [None, None]
        self.jtg = [None, None]
        self.cur_traj = [None, None]
        self.cur_traj_timer = [None, None]
        self.cur_traj_pos = [None, None]

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

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
        r_jt_idx_list = [17, 18, 16, 20, 19, 21, 22]
        l_jt_idx_list = [31, 32, 30, 34, 33, 35, 36]
        for i,nm in enumerate(self.joint_nm_list):
            idx = r_jt_idx_list[i]
            if data.name[idx] != 'r_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            arm_angles[0] += [data.position[idx]]
            arm_efforts[0] += [data.effort[idx]]

            idx = l_jt_idx_list[i]
            if data.name[idx] != 'l_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            arm_angles[1] += [data.position[idx]]
            arm_efforts[1] += [data.effort[idx]]

        self.arm_state_lock[0].acquire()
        self.arm_angles[0] = arm_angles[0]
        self.arm_efforts[0] = arm_efforts[0]
        self.arm_state_lock[0].release()

        self.arm_state_lock[1].acquire()
        self.arm_angles[1] = arm_angles[1]
        self.arm_efforts[1] = arm_efforts[1]
        self.arm_state_lock[1].release()

    ##
    # Create a joint configuration trajectory goal.
    #
    # @param arm 0 for right, 1 for left
    # @param q_arr list of lists of 7 joint angles in RADIANS.
    # @param dur_arr list of how long (SECONDS) from the beginning of the trajectory
    #                before reaching the joint angles.
    # @param stamp header (rospy.Duration) stamp to give the trajectory 
    def create_JTG(self, arm, q_arr, dur_arr, stamp=None):
        if arm != 1:
            arm = 0
        jtg = JointTrajectoryGoal()
        if stamp is None:
            stamp = rospy.Time.now()
        else:
            jtg.trajectory.header.stamp = stamp
        jtg.trajectory.joint_names = self.joint_names_list[arm]
        for i in range(len(q_arr)):
            if q_arr[i] is None or type(q_arr[i]) is types.NoneType:
                continue
            jtp = JointTrajectoryPoint()
            jtp.positions = q_arr[i]
            jtp.velocities = [0.] * 7
            jtp.accelerations = [0.] * 7
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

        self.cur_traj[arm] = jtg
        self.cur_traj_pos[arm] = 0

        # if too far in past, shift forward the time the trajectory starts
        min_init_time = rospy.Time.now().to_sec() + 2 * self.send_delay
        if jtg.trajectory.header.stamp.to_nsec() < min_init_time:
            jtg.trajectory.header.stamp = rospy.Duration(rospy.Time.now().to_sec(), 
                                                         2 * self.send_delay)

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
                                       cur_exec_time)
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
        if self.cur_traj[arm] is not None:
            return True
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
            ret = ik_resp.solution.joint_state.position
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
    def move_arm(self, arm, pos, rot=None, dur=4.0):
        begq = self.get_joint_angles(arm)
        if rot is None:
            temp, rot = self.FK(arm, begq)
        endq = self.IK(arm, pos, rot, begq)
        if endq is None:
            return False
        self.set_joint_angles(arm, endq, dur)
        return True

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
    def move_arm_trajectory(self, arm, pos_arr, dur=1., rot_arr=None, dur_arr=None):
        if dur_arr is None:
            dur_arr = np.linspace(0., dur, len(pos_arr) + 1)[1:]
        
        curq = self.get_joint_angles(arm)
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
            if not (nextq is None):
                curq = nextq
        log("IK Accuracy: %d/%d (%f)" % (trys-fails, trys, (trys-fails)/float(trys)))
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

    # need for search and hook
#   def go_cartesian(self, arm):
#       rospy.logerr('Need to implement this function.')
#       raise RuntimeError('Unimplemented function')

#   def get_wrist_force(self, arm, bias = True, base_frame = False):
#       rospy.logerr('Need to implement this function.')
#       raise RuntimeError('Unimplemented function')

#   def set_cartesian(self, arm, p, rot):
#       if arm != 1:
#           arm = 0
#       ps = PoseStamped()
#       ps.header.stamp = rospy.rostime.get_rostime()
#       ps.header.frame_id = 'torso_lift_link'

#       ps.pose.position.x = p[0,0]
#       ps.pose.position.y = p[1,0]
#       ps.pose.position.z = p[2,0]

#       quat = tr.matrix_to_quaternion(rot)
#       ps.pose.orientation.x = quat[0]
#       ps.pose.orientation.y = quat[1]
#       ps.pose.orientation.z = quat[2]
#       ps.pose.orientation.w = quat[3]
#       if arm == 0:
#           self.r_arm_cart_pub.publish(ps)
#       else:
#           self.l_arm_cart_pub.publish(ps)

# TODO Evaluate gripper functions and parameters

    ##
    # Move the gripper the given amount with given amount of effort
    #
    # @param arm 0 for right, 1 for left
    # @param amount the amount the gripper should be opened
    # @param effort - supposed to be in Newtons. (-ve number => max effort)
    def move_gripper(self, arm, amount=0.08, effort = 15):
        self.gripper_action_client[arm].send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=amount, max_effort = -1)))

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
    def close_gripper(self, arm):
        self.move_gripper(arm, 0.0, 15)

    # def get_wrist_force(self, arm):
    #     pass

    ######################################################
    # More specific functionality
    ######################################################

    ##
    # Moves arm smoothly through a linear trajectory.
    #
    # @param arm
    def smooth_linear_arm_trajectory(self, arm, dist,
                                     dir=(0.,0.,-1.), max_jerk=0.5, delta=0.09, dur=None):

        ####################################################
        # Smooth trajectory functions
        def _smooth_traj_pos(t, k, T):
            return -k / T**3 * np.sin(T * t) + k / T**2 * t

        # def _smooth_traj_vel(t, k, T):
        #     return -k / T**2 * np.cos(T * t) + k / T**2 
        # 
        # def _smooth_traj_acc(t, k, T):
        #     return k / T * np.sin(T * t) 

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
        traj_mult = _interpolate_traj(_smooth_traj_pos, max_jerk, period, 
                                      num_steps, 0., trajt) 
        # cartesian offset points of trajectory
        cart_traj = [(a*traj_vec[0],a*traj_vec[1],a*traj_vec[2]) for a in traj_mult]
        cur_pos, cur_rot = self.FK(arm, self.get_joint_angles(arm))
        # get actual positions of the arm by transforming the offsets
        arm_traj = [(ct[0] + cur_pos[0], ct[1] + cur_pos[1],
                                         ct[2] + cur_pos[2]) for ct in cart_traj]
        if dur is None:
            dur = trajt
        self.move_arm_trajectory(arm, arm_traj, dur)

        # return the expected time of the trajectory
        return dur

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

    simparm = SimpleArmTrajectory()

    if False:
        q = [0, 0, 0, 0, 0, 0, 0]
        simparm.set_jointangles('right_arm', q)
        ee_pos = simparm.FK('right_arm', q)
        log('FK result:' + ee_pos.A1)

        ee_pos[0,0] -= 0.1
        q_ik = simparm.IK('right_arm', ee_pos, tr.Rx(0.), q)
        log('q_ik:' + [math.degrees(a) for a in q_ik])

        rospy.spin()

    if False:
        p = np.matrix([0.9, -0.3, -0.15]).T
        #rot = tr.Rx(0.)
        rot = tr.Rx(math.radians(90.))
        simparm.set_cartesian('right_arm', p, rot)

    simparm.open_gripper('right_arm')
    raw_input('Hit ENTER to close')
    simparm.close_gripper('right_arm', effort = 15)



