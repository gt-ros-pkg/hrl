#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'rosparam' )
roslib.load_manifest( 'actionlib' )
roslib.load_manifest( 'kinematics_msgs' )
roslib.load_manifest( 'trajectory_msgs' )
roslib.load_manifest( 'geometry_msgs' )
roslib.load_manifest( 'pr2_controllers_msgs' )
roslib.load_manifest( 'tf' )
roslib.load_manifest( 'sensor_msgs' )
roslib.load_manifest( 'planning_environment_msgs' )
roslib.load_manifest( 'motion_planning_msgs' )
roslib.load_manifest( 'std_msgs' )

import rospy
import rosparam

from actionlib import \
     SimpleActionClient, GoalStatus
from kinematics_msgs.srv import \
    GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import \
    JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import \
    Pose, Point, Quaternion
from pr2_controllers_msgs.msg import \
    JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from planning_environment_msgs.srv import \
    GetJointTrajectoryValidity, GetJointTrajectoryValidityRequest, GetRobotState

from motion_planning_msgs.srv import \
    FilterJointTrajectoryWithConstraints, FilterJointTrajectoryWithConstraintsRequest

from std_msgs.msg import Int8

import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin
import random

import sys

def getJointNames( controller_name ):
    return rosparam.get_param( ''.join( (controller_name, '/joints') ) ) 

def rpyToQuaternion( r, p, y ):
    q = [0] * 4
    q[0] = cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2)
    q[1] = sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2)
    q[2] = cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2)
    q[3] = cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2)
    return q

class TrajectoryFilter:
    
    def __init__( self, service_name ):
        rospy.loginfo( 'waiting for service: %s'%service_name )
        rospy.wait_for_service( service_name )
        self.proxy = rospy.ServiceProxy( service_name, FilterJointTrajectoryWithConstraints )

    def filter( self, trajectory ):
        # rospy.loginfo( 'filtering trajectory:\n %s'%trajectory )
        request = FilterJointTrajectoryWithConstraintsRequest()
        request.allowed_time = rospy.Duration.from_sec(1.0);
        request.trajectory = trajectory
        try:
            response = self.proxy( request )
            
            if response.error_code.val == response.error_code.SUCCESS:
                return response.trajectory
            else:
                rospy.logerr( 'Trajectory was not filtered! (error: %s)'%response.error_code.val )
                return None

        except rospy.ServiceException, e:
            rospy.logerr( 'service call failed: %s'%e)
            return None
        
    
class ArmIK:
    
    def __init__( self, controller_name, service_name ):
        rospy.loginfo( 'waiting for service: %s'%service_name )
        rospy.wait_for_service( service_name )
        self.proxy = rospy.ServiceProxy( service_name, GetPositionIK )

        # handle joint states for seek 
        self.joint_names = getJointNames( controller_name )
        self.joint_idx = None
        self.positions = None
        rospy.Subscriber( '/joint_states', JointState, self.js_callback )

        # set static fields of request
        self.request = GetPositionIKRequest()
        self.request.timeout = rospy.Duration.from_sec( 5.0 )
        self.request.ik_request.ik_link_name = 'r_wrist_roll_link'
        self.request.ik_request.pose_stamped.header.frame_id = "torso_lift_link"
        self.request.ik_request.ik_seed_state.joint_state.name = self.joint_names
        self.request.ik_request.ik_seed_state.joint_state.position = [0] * len( self.joint_names )

    def js_callback( self, msg ):
        if self.joint_idx is None:
            self.joint_idx = map( msg.name.index, self.joint_names )
        
        self.positions = map( lambda x : msg.position[x], self.joint_idx )

    def getJointNames( self ):
        return self.joint_names

    def getJointPositions( self ):
        return self.positions
    
    def getSolution( self, pose ):
        rospy.logdebug( 'requesting IK solution for pose:\n %s'%pose )
        self.request.ik_request.pose_stamped.pose = pose

        # set seed to current joint positions
        if not self.positions is None:
            self.request.ik_request.ik_seed_state.joint_state.position = self.positions

        rospy.logdebug( 'seed IK with: %s'%self.request.ik_request.ik_seed_state.joint_state.position )

        try:
            response = self.proxy( self.request )
            
            if response.error_code.val == response.error_code.SUCCESS:
                return np.asarray( response.solution.joint_state.position )
            else:
                rospy.logerr( 'IK failed! (error: %s)'%response.error_code.val )
                return None

        except rospy.ServiceException, e:
            rospy.logerr( 'service call failed: %s'%e)
            return None
    
    def get2PointTrajectory( self, positions, time_from_start ):
        if self.positions is None or positions is None:
            return None
        jt = JointTrajectory()
        jt.joint_names = self.joint_names
        jt.points = []
        jp = JointTrajectoryPoint()
        jp.time_from_start = rospy.Time.from_seconds( 0.0 )
        jp.positions = self.positions
        jt.points.append( jp )
        jp = JointTrajectoryPoint()
        jp.time_from_start = rospy.Time.from_seconds( time_from_start )
        jp.positions = positions
        jt.points.append( jp )
        jt.header.stamp = rospy.Time.now()
        return jt

class TrajectoryArmController:
    
    def __init__( self, controller_name ):
        topic = ''.join( ( controller_name, '/joint_trajectory_action' ) )
        self.client = SimpleActionClient( topic, JointTrajectoryAction )
        rospy.loginfo( 'waiting for action client: %s'%topic )
        self.client.wait_for_server()
        
    def move( self, trajectory ):
        # push trajectory goal to ActionServer
        goal = JointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.trajectory.header.stamp = rospy.Time.now()

        self.client.send_goal( goal )
        self.client.wait_for_result()
    
        if self.client.get_state == GoalStatus.SUCCEEDED:
            rospy.logerr( 'failed to move arm to goal:\n %s'%goal )
            return False
        else:
            return True

class PlanningEnvironment:
    
    def __init__( self, controller_name, node_name ):
        service = ''.join( (node_name, '/get_trajectory_validity') )
        rospy.loginfo( 'waiting for service: %s'%service )
        rospy.wait_for_service( service )
        self.validity_proxy = rospy.ServiceProxy( service, GetJointTrajectoryValidity )

        service = ''.join( (node_name, '/get_robot_state') )
        rospy.loginfo( 'waiting for service: %s'%service )
        rospy.wait_for_service( service )
        self.state_proxy = rospy.ServiceProxy( service, GetRobotState )

        self.joint_names = getJointNames( controller_name )        
                
        # set static fields of request
        self.request = GetJointTrajectoryValidityRequest()
        self.request.robot_state.joint_state.name = self.joint_names
        self.request.check_collisions = True
        #self.request.check_joint_limits = True
        
    def check( self, trajectory ):
        # rospy.loginfo( 'check joint positions %s for collision'%positions )
        
        self.request.robot_state = self.getRobotState()

        self.request.trajectory = trajectory

        try:
            response = self.validity_proxy( self.request )
            
            if response.error_code.val == response.error_code.SUCCESS:
                return True
            else:
                rospy.logerr( 'Joint-trajectory in collision! (error: %s)'%response.error_code.val )
                return False

        except rospy.ServiceException, e:
            rospy.logerr( 'service call failed: %s'%e)
            return False

    def getRobotState( self ):
        try:
            response = self.state_proxy()
            return response.robot_state

            # if response.error_code.val == response.error_code.SUCCESS:
            #     return response.robot_state
            # else:
            #     rospy.logerr( 'Can\'t get robot state! (error: %s)'%response.error_code.val )
            #     return False

        except rospy.ServiceException, e:
            rospy.logerr( 'service call failed: %s'%e)
            return None
        

def plotTrajectory( trajectory ):
    P = []
    for tp in trajectory.points:
        P.append( tp.positions )
    P = np.asarray( P )
    print P.transpose()
    plt.plot( P )
    plt.show


if __name__ == '__main__':
    node_name = 'move_arm'
    controller_name = 'r_arm_controller'
    rospy.init_node( node_name, log_level = rospy.DEBUG )
    
    controller = TrajectoryArmController( controller_name )
    ik = ArmIK( 'r_arm_controller', 'pr2_right_arm_kinematics/get_ik' )
    planning = PlanningEnvironment( controller_name, 
                                    'environment_server_right_arm' )
    traj_filter = TrajectoryFilter( 'trajectory_filter/filter_trajectory_with_constraints' )

    update_pub = rospy.Publisher( '/joint_state_viz/update', Int8 )

    random.random()

    counter = 0

    joint_names = getJointNames( controller_name )

    positions = None
    while positions == None:
        positions = ik.getJointPositions()
    
    joint_names = ik.getJointNames()

    last_point = JointTrajectoryPoint()
    last_point.positions = positions



    soft_lower_limit = -2.1353981634
    soft_upper_limit = 0.564601836603

    soft_lower_limit = -np.pi
    soft_upper_limit = np.pi
    joint_id = joint_names.index( 'r_forearm_roll_joint' )
    i = 0

    while not rospy.is_shutdown():
        i = i + 1

        print( '----- %d -----'%i )
    
        pos = np.random.rand() * ( soft_upper_limit - soft_lower_limit ) + soft_lower_limit
        rospy.loginfo( 'move to pos %6.4f'%pos )
    
        jt = JointTrajectory()
        jt.joint_names = joint_names
        jt.points = []
        last_point.time_from_start = rospy.Time.from_seconds( 0.0 )
        jt.points.append( last_point )
            
        jp = JointTrajectoryPoint()
        jp.time_from_start = rospy.Time.from_seconds( 5.0 )
        jp.positions = last_point.positions
        jp.positions[joint_id] = pos
        jt.points.append( jp )
        jt.header.stamp = rospy.Time.now()

        update_pub.publish( 0 )
        rospy.sleep( 1 )
        controller.move( jt )
        rospy.sleep( 2 )
        update_pub.publish( 1 )

        last_point = jp

    # while not rospy.is_shutdown():

    #     trajectory = JointTrajectory()
    #     trajectory.points = []
    #     trajectory.joint_names = joint_names

    #     last_point.time_from_start = rospy.Time().from_sec( 0.0 )
    #     trajectory.points.append( last_point )

    #     while len( trajectory.points ) < 3:

    #         x = random.random()
    #         y = random.random() * 2.0 - 1.0
    #         z = random.random() - 0.5

    #         roll = random.random() * 2 * np.pi - np.pi
    #         pitch = random.random() * 2 * np.pi - np.pi
    #         yaw = random.random() * 2 * np.pi - np.pi

    #         q = rpyToQuaternion( roll, pitch, yaw )
                
    #         pose = Pose( position = Point( x, y, z),
    #                      orientation = Quaternion( q[0], q[1], q[2], q[3] ) )

    #         positions = ik.getSolution( pose )

    #         if not positions is None:
    #             tp = JointTrajectoryPoint()
    #             tp.positions = positions
    #             tp.time_from_start = rospy.Time.from_sec( len( trajectory.points ) * 1.0 )
    #             trajectory.points.append( tp )

    #     last_point = trajectory.points[-1]
    #     trajectory.header.stamp = rospy.Time.now()


    #     trajectory = traj_filter.filter( trajectory )
    #     if not trajectory is None:
    #         if planning.check( trajectory ):
    #             trajectory.header.stamp = rospy.Time.now()
    #             print trajectory.points[-1].time_from_start.to_sec()
    #             controller.move( trajectory )




    # while not rospy.is_shutdown():
    #     x = random.random()
    #     y = random.random() * 2.0 - 1.0
    #     z = random.random() - 0.5

    #     roll = random.random() * 2 * np.pi - np.pi
    #     pitch = random.random() * 2 * np.pi - np.pi
    #     yaw = random.random() * 2 * np.pi - np.pi

    #     q = rpyToQuaternion( roll, pitch, yaw )
                
    #     pose = Pose( position = Point( x, y, z),
    #                  orientation = Quaternion( q[0], q[1], q[2], q[3] ) )

    #     positions = ik.getSolution( pose )

    #     if not positions is None:
            
    #         print 'get two point trajectory ...'
    #         trajectory = ik.get2PointTrajectory( positions, 1.0 )
    #         print 'done!'

    #         if not trajectory is None:

    #             # # filter trajectory
    #             # print 'filter trajectory ...'
    #             # trajectory = traj_filter.filter( trajectory )
    #             # print 'done!'

    #             if not trajectory is None:
    #                 if planning.check( trajectory ):

    #                     update_pub.publish( 0 )

    #                     rospy.sleep( 2 )
                        
    #                     controller.move( trajectory )

    #                     rospy.sleep( 2 )

    #                     update_pub.publish( 1 )

    #                     counter = counter + 1
    #                     print( '----- %d ----- '%counter )

    #                     if counter == 100:
    #                         sys.exit()

