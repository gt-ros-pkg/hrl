import roslib; roslib.load_manifest('hai_sandbox')
import sys

import rospy
import actionlib
import move_base_msgs.msg as mm
import sensor_msgs.msg as sm
import pr2_controllers_msgs.msg as pm
import trajectory_msgs.msg as tm

import hrl_lib.util as ut
import hrl_lib.rutils as ru
import functools as ft
import numpy as np
import pdb
import time

class Joint:

    def __init__(self, name, joint_provider):
        self.joint_provider = joint_provider
        self.joint_names = rospy.get_param('/%s/joints' % name)
        self.pub = rospy.Publisher('%s/command' % name, tm.JointTrajectory)
        self.names_index = None
        self.zero_vel = [0 for j in range(len(self.joint_names))]

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
            if vel_mat == None:
                points[i].velocities = self.zero_vel
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
    def __init__(self, name, joint_provider):
        Joint.__init__(self, name, joint_provider)
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % name, pm.JointTrajectoryAction)
        rospy.loginfo('pr2arm: waiting for server %s' % name)
        self.client.wait_for_server()

    ##
    # @param pos_mat column matrix of poses
    # @param times array of times
    def set_poses(self, pos_mat, times, vel_mat=None, block=True):
        joint_traj = Joint._create_trajectory(self, pos_mat, times, vel_mat)

        #Create goal msg
        g = pm.JointTrajectoryGoal()
        g.trajectory = joint_traj
        self.client.send_goal(g)
        if block:
            return self.client.wait_for_result()
        return self.client.get_state()

class PR2Head(Joint):

    def __init__(self, name, joint_provider):
        Joint.__init__(self, name, joint_provider)

    def look_at(self):
        raise RuntimeError('not impletmented')


class PR2Base:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', mm.MoveBaseAction)
        rospy.loginfo('pr2base: waiting for server')
        self.client.wait_for_server()

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


class PR2Torso:

    def __init__(self):
        self.torso = actionlib.SimpleActionClient('torso_controller/position_joint_action', pm.SingleJointPositionAction)
        self.torso.wait_for_server()

    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()

class PR2:
    def __init__(self):
        jl = ru.GenericListener('joint_state_listener', sm.JointState, 'joint_states', 100)
        self.joint_provider = ft.partial(jl.read, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)

        self.left_arm = PR2Arm('l_arm_controller', self.joint_provider)
        self.right_arm = PR2Arm('r_arm_controller', self.joint_provider)
        self.head = PR2Head('head_traj_controller', self.joint_provider)
        self.base = PR2Base()
        self.torso = PR2Torso()


    def pose(self):
        s = self.joint_provider()
        return {'larm': self.left_arm.pose(s), 'rarm': self.right_arm.pose(s), 'head_traj': self.head.pose(s)}



loc_fname = sys.argv[1]
data = ut.load_pickle(loc_fname)
rospy.init_node('imitate')
robot = PR2()
state = 'init_manipulation'
#    data = {'base_pose': pose_base, 
#            'robot_pose': j0_dict,
#            'arm': arm_used,
#            'movement_states': None}

if state == 'drive':
    t, r = data['base_pose']
    print t
    r = robot.base.set_pose(t, r, '/map', block=True)
    rospy.loginfo('result is %s' % str(r))

#Put robot in the correct state
if state == 'init_manipulation':
    rospy.loginfo('STATE init_manipulation')
    j0_dict = data['robot_pose']
    cpos = robot.pose()
    robot.left_arm.set_poses (np.column_stack([cpos['larm'], j0_dict['poses']['larm']]), np.array([0.1, 5.]), block=False)
    robot.right_arm.set_poses(np.column_stack([cpos['rarm'], j0_dict['poses']['rarm']]), np.array([0.1, 5.]), block=False)
    robot.head.set_poses(np.column_stack([cpos['head_traj'], j0_dict['poses']['head_traj']]), np.array([.01, 5.]))
    robot.torso.set_pose(j0_dict['poses']['torso'][0,0], block=True)
    state = 'manipulate'

if state == 'manipulate':
    rospy.loginfo('STATE manipulate')
    rospy.loginfo('there are %d states' % len(data['movement_states']))
    for state in range(len(data['movement_states'])):
        cur_state = data['movement_states'][state]
        rospy.loginfo("starting %s" % cur_state['name'])

        larm, lvel, ltime, rarm, rvel, rtime = zip(*[[jdict['poses']['larm'], jdict['vels']['larm'], jdict['time'], \
                                                      jdict['poses']['rarm'], jdict['vels']['rarm'], jdict['time']] \
                                                            for jdict in cur_state['joint_states']])

        larm = np.column_stack(larm)
        rarm = np.column_stack(rarm)
        lvel = np.column_stack(lvel)
        rvel = np.column_stack(rvel)
        ltime = np.array(ltime) - cur_state['start_time']
        rtime = np.array(rtime) - cur_state['start_time']

        robot.left_arm.set_poses(larm[:,0], np.array([2.]), block=False)
        robot.right_arm.set_poses(rarm[:,0], np.array([2.]), block=True)

        robot.left_arm.set_poses(larm, ltime, vel_mat=lvel, block=False)
        robot.right_arm.set_poses(rarm, rtime, vel_mat=rvel, block=True)

        rospy.loginfo("%s FINISHED" % cur_state['name'])
        time.sleep(5)


## Need a refinement step

## load extracted file
#
##Need to be localized!!
## NOT LEARNED: go into safe state.
#
## drive. learned locations. (might learn path/driving too?)
#pose_base = base_T_map * pose_map
#
## move joints to initial state. learned initial state. (maybe coordinate this with sensors?)
#
## for each contact state
##      send trajectory. wait until contact state changes or traj. finished executing.
