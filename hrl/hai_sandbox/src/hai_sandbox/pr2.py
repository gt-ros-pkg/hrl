import roslib; roslib.load_manifest('hai_sandbox')
import rospy

import actionlib
import move_base_msgs.msg as mm
import sensor_msgs.msg as sm
import pr2_controllers_msgs.msg as pm
import trajectory_msgs.msg as tm
import pr2_mechanism_msgs.srv as pmm
import std_msgs.msg as stdm

import hrl_lib.rutils as ru
import functools as ft
import numpy as np

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

    def __init__(self, joint_controller_name, cart_controller_name, joint_provider):
        Joint.__init__(self, joint_controller_name, joint_provider)
        self.client = actionlib.SimpleActionClient('/%s/joint_trajectory_action' % joint_controller_name, pm.JointTrajectoryAction)
        rospy.loginfo('pr2arm: waiting for server %s' % joint_controller_name)
        self.client.wait_for_server()
        self.cart_posure_pub = rospy.Publisher("/%s/command_posture" % cart_controller_name, stdm.Float64MultiArray).publish
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
        rospy.loginfo('pr2base: waiting for move_base')
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
        rospy.loginfo('waiting for torso_controller')
        self.torso.wait_for_server()

    def set_pose(self, p, block=True):
        self.torso.send_goal(pm.SingleJointPositionGoal(position = p))
        if block:
            self.torso.wait_for_result()
        return self.torso.get_state()

class ControllerManager:

    def __init__(self):
        # LoadController        
        #string name
        #---
        #bool ok
        self.load = rospy.ServiceProxy('pr2_controller_manager/load_controller', pmm.LoadController)

        # UnloadController        
        #string name
        #---
        #bool ok
        self.unload = rospy.ServiceProxy('pr2_controller_manager/unload_controller', pmm.UnloadController)

        # SwitchController
        #string[] start_controllers
        #string[] stop_controllers
        #int32 strictness
        #int32 BEST_EFFORT=1
        #int32 STRICT=2
        self._switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pmm.SwitchController)

    def switch(self, start_con, stop_con):
        for n in start_con:
            self.load(n)
        resp = self._switch_controller(start_con, stop_con, pmm.SwitchControllerRequest.STRICT)
        for n in stop_con:
            self.unload(n)
        return resp.ok


class PR2:
    def __init__(self):
        jl = ru.GenericListener('joint_state_listener', sm.JointState, 'joint_states', 100)
        self.joint_provider = ft.partial(jl.read, allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)

        self.left_arm = PR2Arm('l_arm_controller',  'l_cart', self.joint_provider)
        self.right_arm = PR2Arm('r_arm_controller', 'r_cart', self.joint_provider)
        self.head = PR2Head('head_traj_controller', self.joint_provider)
        self.base = PR2Base()
        self.torso = PR2Torso()
        self.controller_manager = ControllerManager()


    def pose(self):
        s = self.joint_provider()
        return {'larm': self.left_arm.pose(s), 'rarm': self.right_arm.pose(s), 'head_traj': self.head.pose(s)}

