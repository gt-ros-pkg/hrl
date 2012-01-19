import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import actionlib
import actionlib_msgs.msg as amsg
import move_base_msgs.msg as mm
import sensor_msgs.msg as sm
import pr2_controllers_msgs.msg as pm
import trajectory_msgs.msg as tm
import pr2_mechanism_msgs.srv as pmm
import std_msgs.msg as stdm
import geometry_msgs.msg as gm
import dynamic_reconfigure.client as dc

import tf
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
import hrl_lib.rutils as ru
import hrl_lib.util as ut
import functools as ft
import numpy as np
import math
import time
import hrl_pr2_lib.msg as hm
#from sound_play.libsoundplay import SoundClient
#from interpolated_ik_motion_planner import ik_utilities as iku
import pr2_kinematics as pr2k
import os
import os.path as pt
import pdb


#Test this
def unwrap2(cpos, npos):
    two_pi = 2*np.pi
    nin = npos % two_pi
    n_multiples_2pi = np.floor(cpos/two_pi)
    return nin + n_multiples_2pi*two_pi


def unwrap(cpos, npos):
    two_pi = 2*np.pi
    if cpos < npos:
        while cpos < npos:
            npos = npos - two_pi
        npos = npos + two_pi
    elif cpos > npos:
        while cpos > npos:
            npos = npos + two_pi
        npos = npos - two_pi
    return npos


def diff_arm_pose(pose1, pose2):
    pcpy = pose2.copy()
    pcpy[4,0] = unwrap2(pose1[4,0], pose2[4,0])
    pcpy[6,0] = unwrap2(pose1[6,0], pose2[6,0])
    diff = pose1 - pose2
    for i in range(pose1.shape[0]):
        diff[i,0] = ut.standard_rad(diff[i,0])
    return diff


class KinematicsError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

class Joint:

    def __init__(self, name, joint_provider):
        self.joint_provider = joint_provider
        self.joint_names = rospy.get_param('/%s/joints' % name)
        self.pub = rospy.Publisher('%s/command' % name, tm.JointTrajectory)
        self.names_index = None
        self.zeros = [0 for j in range(len(self.joint_names))]

    def pose(self, joint_states=None):
        if joint_states == None:
            joint_states = self.joint_provider()

        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(joint_states.name):
                self.names_index[n] = i
            self.joint_idx = [self.names_index[n] for n in self.joint_names]

        return (np.matrix(joint_states.position).T)[self.joint_idx, 0]

    def _create_trajectory(self, pos_mat, times, vel_mat=None):
        #Make JointTrajectoryPoints
        points = [tm.JointTrajectoryPoint() for i in range(pos_mat.shape[1])]
        for i in range(pos_mat.shape[1]):
            points[i].positions = pos_mat[:,i].A1.tolist()
            points[i].accelerations = self.zeros
            if vel_mat == None:
                points[i].velocities = self.zeros
            else:
                points[i].velocities = vel_mat[:,i].A1.tolist()

        for i in range(pos_mat.shape[1]):
            points[i].time_from_start = rospy.Duration(times[i])

        #Create JointTrajectory
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points = points
        jt.header.stamp = rospy.get_rostime()
        return jt

    def set_poses(self, pos_mat, times):
        joint_trajectory = self._create_trajectory(pos_mat, times)
        self.pub.publish(joint_trajectory)


class PR2Arm(Joint):

    def __init__(self, joint_provider, tf_listener, arm):
        joint_controller_name = arm + '_arm_controller'
        cart_controller_name = arm + '_cart'
        Joint.__init__(self, joint_controller_name, joint_provider)
        self.arm = arm
        self.tf_listener = tf_listener
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % joint_controller_name, pm.JointTrajectoryAction)
        rospy.loginfo('pr2arm: waiting for server %s' % joint_controller_name)
        self.client.wait_for_server()
        self.joint_controller_name = joint_controller_name

        self.cart_posure_pub = rospy.Publisher("/%s/command_posture" % cart_controller_name, stdm.Float64MultiArray).publish
        self.cart_pose_pub = rospy.Publisher("/%s/command_pose" % cart_controller_name, gm.PoseStamped).publish
        if arm == 'l':
            self.full_arm_name = 'left'
        else:
            self.full_arm_name = 'right'
        self.kinematics = pr2k.PR2ArmKinematics(self.full_arm_name, self.tf_listener)
        #self.ik_utilities = iku.IKUtilities(self.full_arm_name, self.tf_listener) 

        self.POSTURES = {
            'off':          np.matrix([]),
            'mantis':       np.matrix([0, 1, 0,  -1, 3.14, -1, 3.14]).T,
            'elbowupr':     np.matrix([-0.79,0,-1.6,  9999, 9999, 9999, 9999]).T,
            'elbowupl':     np.matrix([0.79,0,1.6 , 9999, 9999, 9999, 9999]).T,
            'old_elbowupr': np.matrix([-0.79,0,-1.6, -0.79,3.14, -0.79,5.49]).T,
            'old_elbowupl': np.matrix([0.79,0,1.6, -0.79,3.14, -0.79,5.49]).T,
            'elbowdownr':   np.matrix([-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, -1.5498884526859626]).T, 
            'elbowdownl':   np.matrix([-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, -1.5565279256852611]).T
            }

    def set_posture(self, posture_mat):
        self.cart_posure_pub(stdm.Float64MultiArray(data=posture_mat.A1.tolist()))

    ##
    # Send a cartesian pose to *_cart controllers
    # @param trans len 3 list
    # @param rot len 3 list
    # @param frame string
    # @param msg_time float
    def set_cart_pose(self, trans, rot, frame, msg_time):
        ps = gm.PoseStamped()
        for i, field in enumerate(['x', 'y', 'z']):
            exec('ps.pose.position.%s = trans[%d]' % (field, i))
        for i, field in enumerate(['x', 'y', 'z', 'w']):
            exec('ps.pose.orientation.%s = rot[%d]' % (field, i))
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time(msg_time)
        self.cart_pose_pub(ps)

    ##
    # @param pos_mat column matrix of poses
    # @param times array of times
    def set_poses(self, pos_mat, times, vel_mat=None, block=True):
        p = self.pose()
        for i in range(pos_mat.shape[1]):
            pos_mat[4,i] = unwrap2(p[4,0], pos_mat[4,i])
            pos_mat[6,i] = unwrap2(p[6,0], pos_mat[6,i])
            p = pos_mat[:,i]

        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def stop_trajectory_execution(self):
        self.client.cancel_all_goals()

    def has_active_goal(self):
        s = self.client.get_state()
        if s == amsg.GoalStatus.ACTIVE or s == amsg.GoalStatus.PENDING:
            return True
        else:
            return False

    def set_poses_monitored(self, pos_mat, times, vel_mat=None, block=True, time_look_ahead=.050):
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        joint_traj.header.stamp = rospy.get_rostime() + rospy.Duration(1.)
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

    def set_pose(self, pos, nsecs=5., block=True):
        for i in range(2):
            cpos = self.pose()
        pos[4,0] = unwrap(cpos[4,0], pos[4,0])
        pos[6,0] = unwrap(cpos[6,0], pos[6,0])
        self.set_poses(np.column_stack([pos]), np.array([nsecs]), block=block)
        #self.set_poses(np.column_stack([cpos, pos]), np.array([min_time, min_time+nsecs]), block=block)

    def pose_cartesian(self, frame='base_link'):
        gripper_tool_frame = self.arm + '_gripper_tool_frame'
        return tfu.transform(frame, gripper_tool_frame, self.tf_listener)

    def pose_cartesian_tf(self, frame='base_link'):
        p, r = tfu.matrix_as_tf(self.pose_cartesian(frame))
        return np.matrix(p).T, np.matrix(r).T


class PR2Head(Joint):

    def __init__(self, name, joint_provider):
        Joint.__init__(self, name, joint_provider)
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', 
                pm.PointHeadAction)

    def look_at(self, pt3d, frame='base_link', pointing_frame="wide_stereo_link", 
                pointing_axis=np.matrix([1, 0, 0.]).T, wait=True):
        g = pm.PointHeadGoal()
        g.target.header.frame_id = frame
        g.target.point = gm.Point(*pt3d.T.A1.tolist())

        #pdb.set_trace()
        g.pointing_frame = pointing_frame
        g.pointing_axis.x = pointing_axis[0,0] 
        g.pointing_axis.y = pointing_axis[1,0]
        g.pointing_axis.z = pointing_axis[2,0]
        g.min_duration = rospy.Duration(1.0)
        g.max_velocity = 10.

        self.head_client.send_goal(g)
        if wait:
            self.head_client.wait_for_result(rospy.Duration(1.))
        if self.head_client.get_state() == amsg.GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def set_pose(self, pos, nsecs=5.):
        for i in range(2):
            cpos = self.pose()
        min_time = .1
        self.set_poses(np.column_stack([cpos, pos]), np.array([min_time, min_time+nsecs]))

###
# DANGER.  DON"T RUN STOP AND WALK AWAY.
##
class PR2Base:
    def __init__(self, tflistener):
        self.tflistener = tflistener
        self.client = actionlib.SimpleActionClient('move_base', mm.MoveBaseAction)
        rospy.loginfo('pr2base: waiting for move_base')
        self.client.wait_for_server()
        rospy.loginfo('pr2base: waiting transforms')
        try:
            self.tflistener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(20))
        except Exception, e:
            rospy.loginfo('pr2base: WARNING! Transform from map to base_footprint not found! Did you launch the nav stack?')
        #    pass

        self.go_angle_client = actionlib.SimpleActionClient('go_angle', hm.GoAngleAction)
        self.go_xy_client = actionlib.SimpleActionClient('go_xy', hm.GoXYAction)

    ##
    # Turns to given angle using pure odometry
    def turn_to(self, angle, block=True):
        goal = hm.GoAngleGoal()
        goal.angle = angle
        self.go_angle_client.send_goal(goal)
        if block:
            rospy.loginfo('turn_to: waiting for turn..')
            self.go_angle_client.wait_for_result()
            rospy.loginfo('turn_to: done.')
            

    ##
    # Turns a relative amount given angle using pure odometry
    def turn_by(self, delta_ang, block=True, overturn=False):
        #overturn
        if overturn and (abs(delta_ang) < math.radians(10.)):
            #turn in that direction by an extra 15 deg
            turn1 = np.sign(delta_ang) * math.radians(15.) + delta_ang
            turn2 = -np.sign(delta_ang) * math.radians(15.)
            rospy.loginfo('Requested really small turn angle.  Using overturn trick.')
            #pdb.set_trace()
            self._turn_by(turn1, block=True)
            time.sleep(3) #TODO remove this restriction
            self._turn_by(turn2, block)
        else:
            self._turn_by(delta_ang, block)


    def _turn_by(self, delta_ang, block=True):
        current_ang_odom = tr.euler_from_matrix(tfu.transform('base_footprint',\
                                'odom_combined', self.tflistener)[0:3, 0:3], 'sxyz')[2]
        self.turn_to(current_ang_odom + delta_ang, block)


    ##
    # Move to xy_loc_bf
    def move_to(self, xy_loc_bf, block=True):
        goal = hm.GoXYGoal()
        goal.x = xy_loc_bf[0,0]
        goal.y = xy_loc_bf[1,0]
        self.go_xy_client.send_goal(goal)
        if block:
            self.go_xy_client.wait_for_result()

    def set_pose(self, t, r, frame, block=True):
        g = mm.MoveBaseGoal()
        p = g.target_pose
        
        p.header.frame_id = frame
        p.header.stamp = rospy.get_rostime()
        p.pose.position.x = t[0]
        p.pose.position.y = t[1]
        p.pose.position.z = 0
        
        p.pose.orientation.x = r[0]
        p.pose.orientation.y = r[1]
        p.pose.orientation.z = r[2]
        p.pose.orientation.w = r[3]
    
        self.client.send_goal(g)
        if block:
            self.client.wait_for_result()
        return self.client.get_state()

    def get_pose(self):
        p_base = tfu.transform('map', 'base_footprint', self.tflistener) \
                * tfu.tf_as_matrix(([0., 0., 0., 1.], tr.quaternion_from_euler(0,0,0)))
        return tfu.matrix_as_tf(p_base)


class PR2Torso(Joint):

    def __init__(self, joint_provider):
        Joint.__init__(self, 'torso_controller', joint_provider)
        self.torso = actionlib.SimpleActionClient('torso_controller/position_joint_action', pm.SingleJointPositionAction)
        rospy.loginfo('waiting for torso_controller')
        self.torso.wait_for_server()

    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()


class PR2Gripper:

    def __init__(self, gripper, joint_provider):
        self.gripper = gripper
        self.joint_provider = joint_provider

        if gripper == 'l':
            self.client = actionlib.SimpleActionClient(
                'l_gripper_controller/gripper_action', pm.Pr2GripperCommandAction)
            self.full_gripper_name = 'left_gripper'
            self.joint_names = [rospy.get_param('/l_gripper_controller/joint')]
        else:
            self.client = actionlib.SimpleActionClient(
                'r_gripper_controller/gripper_action', pm.Pr2GripperCommandAction)
            self.full_gripper_name = 'right_gripper'
            self.joint_names = [rospy.get_param('/r_gripper_controller/joint')]
        self.client.wait_for_server()
        self.names_index = None

    def pose(self, joint_states=None):
        if joint_states == None:
            joint_states = self.joint_provider()

        if self.names_index == None:
            self.names_index = {}
            for i, n in enumerate(joint_states.name):
                self.names_index[n] = i
            self.joint_idx = [self.names_index[n] for n in self.joint_names]

        return (np.matrix(joint_states.position).T)[self.joint_idx, 0]

    def close(self, block, position=0.0, effort=-1):
        self.client.send_goal(pm.Pr2GripperCommandGoal(
                pm.Pr2GripperCommand(position = position, max_effort = effort)))
        if block:
            self.client.wait_for_result()
        return self.client.get_state()

    def open(self, block, position=0.1, effort = -1):
        self.client.send_goal(pm.Pr2GripperCommandGoal(
                pm.Pr2GripperCommand(position = position, max_effort = effort)))
        if block:
            self.client.wait_for_result()
        return self.client.get_state()

class StructuredLightProjector:
    def __init__(self):
        self.client = dc.Client("camera_synchronizer_node")
        self.node_config = self.client.get_configuration()

    def set(self, on):
        self.node_config["projector_mode"] = 2
        if on:
            self.node_config["narrow_stereo_trig_mode"] = 3
        else:
            self.node_config["narrow_stereo_trig_mode"] = 2
        self.client.update_configuration(self.node_config)

    def set_prosilica_inhibit(self, on):
        self.node_config['prosilica_projector_inhibit'] = on
        self.client.update_configuration(self.node_config)


class ControllerManager:

    def __init__(self):
        # LoadController        
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)
        # UnloadController        
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)
        # SwitchController
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

    def switch(self, start_con, stop_con):
        for n in start_con:
            self.load(n)
        resp = self._switch_controller(start_con, stop_con, pmm.SwitchControllerRequest.STRICT)
        for n in stop_con:
            self.unload(n)
        return resp.ok


class SoundPlay:

    def __init__(self):
        self.ros_home = pt.join(os.getenv("HOME"), '.ros')

    def say(self, phrase):
        wav_file_name = pt.join(self.ros_home, 'soundplay_temp.wav')
        os.system("text2wave %s -o %s" % (phrase, wav_file_name))
        os.system("aplay %s" % (wav_file_name))

    def play(self, filename):
        os.system("aplay %s" % filename)


class PR2:

    def __init__(self, tf_listener=None, arms=True, base=False, grippers=True):
        try:
            rospy.init_node('pr2', anonymous=True)
        except rospy.exceptions.ROSException, e:
            pass

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener
        jl = ru.GenericListener('joint_state_listener', sm.JointState, 'joint_states', 100)
        self.joint_provider = ft.partial(jl.read, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)

        if arms:
            self.left = PR2Arm(self.joint_provider, self.tf_listener, 'l')
            self.right = PR2Arm(self.joint_provider, self.tf_listener, 'r')

        if grippers:
            self.left_gripper = PR2Gripper('l', self.joint_provider)
            self.right_gripper = PR2Gripper('r', self.joint_provider)

        self.head = PR2Head('head_traj_controller', self.joint_provider)

        if base:
            self.base = PR2Base(self.tf_listener)
        self.torso = PR2Torso(self.joint_provider)
        self.controller_manager = ControllerManager()
        self.sound = SoundPlay()
        #SoundClient()
        self.projector = StructuredLightProjector()

    def pose(self):
        s = self.joint_provider()
        return {'larm': self.left.pose(s), 'rarm': self.right.pose(s), 'head_traj': self.head.pose(s)}


#if __name__ == '__main__':
#    #pr2 = PR2()
#    #pr2.controller_manager
#
#    raw_input('put robot in final pose')
#    pose2 = pr2.left.pose_cartesian()
#
#    raw_input('put robot in initial pose')
#    pose1 = pr2.left.pose_cartesian()
#    pose2 = pose1.copy()
#    pose2[0,3] = pose2[0,3] + .2
#    r = rospy.Rate(4)
#    while not rospy.is_shutdown():
#         cart = pr2.left.pose_cartesian()
#         ik_sol = pr2.left.kinematics.ik(cart, 'base_link')
#         if ik_sol != None:
#             diff = pr2.left.kinematics.fk(ik_sol, 'base_link') - cart
#             pos_diff = diff[0:3,3]
#             print '%.2f %.2f %.2f' % (pos_diff[0,0], pos_diff[1,0], pos_diff[2,0])
#
#    pdb.set_trace()
#    print 'going to final pose'
#    pr2.left.set_cart_pose_ik(pose2, 2.5)
#
#    print 'going back to initial pose'
#    pr2.left.set_cart_pose_ik(pose1, 2.5)
#
#
#    r = rospy.Rate(4)
#    while not rospy.is_shutdown():
#        cart   = pr2.left.pose_cartesian()
#        ik_sol = pr2.left.kinematics.ik(cart, 'base_link', seed=pr2.left.pose())
#        if ik_sol != None:
#            print ik_sol.T
#        r.sleep()






























#from class PR2Arm

    #def set_cart_pose_ik(self, cart, total_time, frame='base_link', block=True,
    #        seed=None, pos_spacing=.001, rot_spacing=.001, best_attempt=True):
    #    cpos                 = self.pose()
    #    start_pos, start_rot = tfu.matrix_as_tf(self.pose_cartesian(frame))

    #    #Check to see if there is an IK solution at end point.
    #    target_pose = None
    #    alpha = 1.
    #    dir_endpoint = cart[0:3,3] - start_pos

    #    while target_pose == None:
    #        target_pose = self.kinematics.ik(perturbed_cart, frame, seed)

    #    if target_pose == None:
    #        raise KinematicsError('Unable to reach goal at %s.' % str(cart))

    #    cpos                 = self.pose()
    #    start_pos, start_rot = tfu.matrix_as_tf(self.pose_cartesian(frame))
    #    end_pos, end_rot     = tfu.matrix_as_tf(cart)
    #    interpolated_poses   = self.ik_utilities.interpolate_cartesian(start_pos, start_rot, end_pos, end_rot, pos_spacing, rot_spacing)
    #    nsteps = len(interpolated_poses)
    #    tstep = total_time / nsteps
    #    tsteps = np.array(range(nsteps+1)) * tstep

    #    valid_wps = []
    #    valid_times = []
    #    #last_valid = seed
    #    #all_sols = []
    #    if seed == None:
    #        seed = cpos
    #    for idx, pose in enumerate(interpolated_poses):
    #        pos, rot = pose 
    #        #sol = self.kinematics.ik(tfu.tf_as_matrix((pos,rot)), frame, seed=last_valid)
    #        sol = self.kinematics.ik(tfu.tf_as_matrix((pos,rot)), frame, seed=seed)
    #        if sol != None:
    #            sol_cpy = sol.copy()
    #            sol_cpy[4,0] = unwrap2(cpos[4,0], sol[4,0])
    #            sol_cpy[6,0] = unwrap2(cpos[6,0], sol[6,0])
    #            valid_wps.append(sol_cpy)
    #            valid_times.append(tsteps[idx])
    #            #cpos = sol_cpy
    #            #all_sols.append(sol)
    #            #last_valid = sol_cpy

    #    #valid_wps.reverse()
    #    #all_sols = np.column_stack(all_sols)
    #    #pdb.set_trace()

    #    if len(valid_wps) > 2:
    #        rospy.loginfo('set_cart_pose_ik: number of waypoints %d' % len(valid_wps)) 
    #        valid_wps_mat = np.column_stack(valid_wps)
    #        valid_times_arr = np.array(valid_times) + .3
    #        #self.set_pose(valid_wps_mat[:,0])
    #        #pdb.set_trace()
    #        self.set_poses(valid_wps_mat, valid_times_arr, block=block)
    #    else:
    #        raise KinematicsError('Unable to reach goal at %s. Not enough valid IK solutions.' % str(cart))
