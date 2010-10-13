
import numpy as np, math
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_lib')
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

import tf.transformations as tftrans
import operator as op
import types

node_name = "simple_arm_manager" 

def log(str):
    rospy.loginfo(node_name + ": " + str)

##
# Class for simple management of the arms and grippers.
# Provides functionality for moving the arms, opening and closing
# the grippers, performing IK, and other functionality as it is
# developed.
class SimpleArmManager(object):

    ##
    # Initializes all of the servers, clients, and variables
    #
    # @param gripper_point given the frame of the wrist_roll_link, this point offsets
    #                      the location used in FK and IK, preferably to the tip of the
    #                      gripper
    def __init__(self, gripper_point=(0.23, 0.0, 0.0)):
        log("Loading SimpleArmManager")
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
        #rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.r_arm_state_cb)
        self.r_arm_cart_pub = rospy.Publisher('/r_cart/command_pose', PoseStamped)
        self.l_arm_cart_pub = rospy.Publisher('/l_cart/command_pose', PoseStamped)

        self.r_arm_pub_l = []
        self.l_arm_pub_l = []
        self.joint_nm_list = ['shoulder_pan', 'shoulder_lift', 'upper_arm_roll',
                              'elbow_flex', 'forearm_roll', 'wrist_flex',
                              'wrist_roll']

        self.r_arm_angles = None
        self.r_arm_efforts = None
        self.jtg = None

        for nm in self.joint_nm_list:
            self.r_arm_pub_l.append(rospy.Publisher('r_'+nm+'_controller/command', Float64))

        self.l_arm_angles = None
        self.l_arm_efforts = None

        for nm in self.joint_nm_list:
            self.l_arm_pub_l.append(rospy.Publisher('l_'+nm+'_controller/command', Float64))

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

        rospy.sleep(1.)

        log("Finished loading SimpleArmManger")

    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    #
    # @param data JointState message recieved from the /joint_states topic
    def joint_states_cb(self, data):
        r_arm_angles = []
        r_arm_efforts = []
        l_arm_angles = []
        l_arm_efforts = []
        r_jt_idx_list = [17, 18, 16, 20, 19, 21, 22]
        l_jt_idx_list = [31, 32, 30, 34, 33, 35, 36]
        for i,nm in enumerate(self.joint_nm_list):
            idx = r_jt_idx_list[i]
            if data.name[idx] != 'r_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            r_arm_angles.append(data.position[idx])
            r_arm_efforts.append(data.effort[idx])

            idx = l_jt_idx_list[i]
            if data.name[idx] != 'l_'+nm+'_joint':
                raise RuntimeError('joint angle name does not match. Expected: %s, Actual: %s i: %d'%('r_'+nm+'_joint', data.name[idx], i))
            l_arm_angles.append(data.position[idx])
            l_arm_efforts.append(data.effort[idx])

        self.arm_state_lock[0].acquire()
        self.r_arm_angles = r_arm_angles
        self.r_arm_efforts = r_arm_efforts
        self.arm_state_lock[0].release()

        self.arm_state_lock[1].acquire()
        self.l_arm_angles = l_arm_angles
        self.l_arm_efforts = l_arm_efforts
        self.arm_state_lock[1].release()

    ##
    # Create a joint configuration trajectory goal.
    #
    # @param arm 0 for right, 1 for left
    # @param q_arr list of lists of 7 joint angles in RADIANS.
    # @param dur_arr list of how long (SECONDS) from the beginning of the trajectory
    #                before reaching the joint angles.
    def create_joint_angles_traj(self, arm, q_arr, dur_arr):
        if arm != 1:
            arm = 0
        jtg = JointTrajectoryGoal()
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
    # Move the arm to a joint configuration.
    #
    # @param arm 0 for right, 1 for left
    # @param q list of 7 joint angles in RADIANS.
    # @param duration how long (SECONDS) before reaching the joint angles.
    def set_joint_angles(self, arm, q, duration=1.):
        if arm != 1:
            arm = 0
        self.jtg = create_joint_angles_traj(arm, [q], [duration])
        self.joint_action_client[arm].send_goal(self.jtg)

    ##
    # Move the arm through a joint configuration trajectory goal.
    #
    # @param arm 0 for right, 1 for left
    # @param q_arr list of lists of 7 joint angles in RADIANS.
    # @param dur_arr list of how long (SECONDS) from the beginning of the trajectory
    #                before reaching the joint angles.
    def set_joint_angles_traj(self, arm, q_arr, dur_arr):
        if arm != 1:
            arm = 0
        self.jtg = create_joint_angles_traj(arm, q_arr, dur_arr)
        self.joint_action_client[arm].send_goal(self.jtg)

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
            fk_req.fk_link_names.append('r_wrist_roll_link') # gripper_tool_frame
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
            
            # Transform point from wrist roll link
            invquatmat = np.mat(tftrans.quaternion_matrix(tftrans.quaternion_inverse(quat)))
            invquatmat[0:3,3] = np.matrix([x,y,z]).T
            trans = np.matrix([self.off_point[0],self.off_point[1],self.off_point[2],1.]).T
            transpos = invquatmat * trans
            ret1 = np.resize(transpos, (3, 1))
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

        if rot[:].shape == (3, 3):
            quat = tr.matrix_to_quaternion(rot)
        elif rot[:].shape == (4, 1):
            quat = rot
        else:
            rospy.logerr('Inverse kinematics failed (bad rotation)')
            return None

        # Transform point back to wrist roll link
        invquatmat = np.mat(tftrans.quaternion_matrix(tftrans.quaternion_inverse(quat.T.A[0])))
        invquatmat[0:3,3] = p
        trans = np.matrix([-self.off_point[0],-self.off_point[1],-self.off_point[2],1.]).T
        transpos = invquatmat * trans
        
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
    def move_arm(self, arm, pos, rot=None, dur=1.0):
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
        log("IK Accuracy: %d/%d (%1.2f)" % (trys-fails, trys, (trys-fails)/float(trys)))
        self.set_joint_angles_traj(arm, q_arr, dur_arr)

    ##
    # Returns the current position, rotation of the arm.
    #
    # @param arm 0 for right, 1 for left
    # @return rotation, position
    def end_effector_pos_rot(self, arm):
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
        if arm == 0:
            q = self.r_arm_angles
        else:
            q = self.l_arm_angles
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

#   def get_wrist_force(self, arm):
#       pass

# TODO Rename classes
    
# Trajectory functions
def smooth_traj_pos(t, k, T):
    return -k / T**3 * np.sin(T * t) + k / T**2 * t

def smooth_traj_vel(t, k, T):
    return -k / T**2 * np.cos(T * t) + k / T**2 

def smooth_traj_acc(t, k, T):
    return k / T * np.sin(T * t) 

def interpolate_traj(traj, k, T, num=10, begt=0., endt=1.):
    return [traj(t,k,T) for t in np.linspace(begt, endt, num)]

# length of time of the trajectory
def traj_time(l, k):
    return np.power(4 * np.pi**2 * l / k, 1./3.)

class SmoothMoveArmTrajectory(SimpleArmManager):
    def __init__(self):
        super(SmoothMoveArmTrajectory,self).__init__()

    # @param arm 0 for right, 1 for left
    def beg_arm_traj(self, arm, dist,
                             dir=(0.,0.,-1.), max_jerk=0.5, delta=0.01, dur=1.0):
        # Vector representing full transform of end effector
        traj_vec = [x/np.sqrt(np.vdot(dir,dir)) for x in dir]
        # number of steps to interpolate trajectory over
        num_steps = dist / delta
        # period of the trajectory 
        trajt = traj_time(dist, max_jerk)
        period = 2. * np.pi / trajt
        # break the trajectory into interpolated parameterized function
        # from 0 to length of the trajectory
        traj_mult = interpolate_traj(smooth_traj_pos, max_jerk, period, num_steps, 0., trajt) 
        # cartesian offset points of trajectory
        cart_traj = [(a*traj_vec[0],a*traj_vec[1],a*traj_vec[2]) for a in traj_mult]
        cur_pos, cur_rot = self.FK(0, self.get_joint_angles(0))
        # get actual positions of the arm by transforming the offsets
        arm_traj = [(ct[0] + cur_pos[0], ct[1] + cur_pos[1],
                                         ct[2] + cur_pos[2]) for ct in cart_traj]

        if dur is None:
            dur = trajt
        self.move_arm_trajectory(arm, arm_traj, dur)

if __name__ == '__main__':
    rospy.init_node(node_name, anonymous = True)
    log("Node initialized")

    simparm = SimpleArmManager()

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



