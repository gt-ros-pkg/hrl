import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'rosparam' )
roslib.load_manifest( 'actionlib' )
roslib.load_manifest( 'trajectory_msgs' )
roslib.load_manifest( 'pr2_controllers_msgs' )

import rospy
import rosparam

import csv

from actionlib import \
     SimpleActionClient
from trajectory_msgs.msg import \
    JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import \
    JointTrajectoryAction, JointTrajectoryGoal

import numpy as np
from math import fmod, pi
import pickle

#-------------------------------------------------------------------------------
# Trajectory utils
#-------------------------------------------------------------------------------

def wrap_angle( a ):
    if a < 0:
        return fmod( a - pi, 2 * pi ) + pi
    if a > 0:
        return fmod( a + pi, 2 * pi ) - pi

def wrap_trajectory( t ):
    rospy.loginfo( 'warapping trajectory [#points: %d] ...', len( t ) )
    b = 0.1
    if len( t ) < 0:
        return
    last = map( wrap_angle, t[0])
    offsets = np.zeros( len( last ) )
    for i in range( len( t ) ):
        for j in range( len( t[i] ) ):
            a = wrap_angle( t[i,j] )
            
            if a > pi - b and last[j] < -pi + b:
                offsets[j] -= 2 * pi
            if a < -pi + b and last[j] > pi - b:
                offsets[j] += 2 * pi

            last[j] = a

        t[i] += offsets
    rospy.loginfo( 'done!' )

def compute_derivative( f, dt ):
    rospy.loginfo( 'computing derivative [#points: %d] ...', len( f ) )
    f_dot = np.zeros( f.shape )
    for i in range( 1, len( f ) - 1  ):    
        for j in range( len( f[i] ) ):
            f_dot[i,j] = (f[i+1,j] - f[i-1,j]) / (2.0 * dt)
    f_dot[0,:] = (f[1,:] - f[-1,:]) / (2.0 * dt)
    f_dot[-1,:] = (f[0,:] - f[-2,:]) / (2.0 * dt)
    rospy.loginfo( 'done!' )
    return f_dot

def save_motion( fname, name, time, pos, vel, acc ):
    rospy.loginfo( 'save motion \'%s\' to file: %s'%( name, fname ) )
    pkl_file = open( fname, 'wb' )
    data = ( name, time, pos, vel, acc )
    pickle.dump( data, pkl_file )
    pkl_file.close()
    rospy.loginfo( 'done!' )

def load_motion( fname ):
    rospy.loginfo( 'loading motion from file: %s'%fname )
    pkl_file = open( fname, 'rb' )
    data = pickle.load( pkl_file )
    rospy.loginfo( 'done!' )
    return data

def load_motion_ascii( fname ):
    reader = csv.reader( open( fname, 'rb' ), delimiter="\t" )
    t = []
    pos = []
    vel = []
    acc = []
    for row in reader:
        t.append( float(row[0]) )
        pos.append( map( lambda x: float(x), row[1:8] ) )
        vel.append( map( lambda x: float(x), row[8:15] ) )
        acc.append( map( lambda x: float(x), row[15:22] ) )
    return (fname, np.asarray(t), np.asarray(pos), np.asarray(vel), np.asarray(acc))

#-------------------------------------------------------------------------------
# JointTrajectoryAction client interfacing
#-------------------------------------------------------------------------------

def init_jt_client(arm = 'r'):
    name = "".join( [ arm, "_arm_controller/joint_trajectory_action" ] )
    client = SimpleActionClient(name, JointTrajectoryAction)
    rospy.loginfo( 'waiting for action client : %s ...'%name )
    client.wait_for_server()
    rospy.loginfo( '%s is up and running!'%name )
    return client

def get_joint_names( node_name ):
    return rosparam.get_param( ''.join( (node_name, '/joints') ) ) 

def move_arm( positions, time_from_start = 3.0, arm = 'r', client = None ):

    if (client == None):
        client = init_jt_client(arm)
    else:
        arm = client.action_client.ns[0:1]; # ignore arm argument

    rospy.loginfo( 'move arm \'%s\' to position %s'%( arm, positions ) )    

    joint_names = get_joint_names( ''.join( ( arm, '_arm_controller' ) ) )
    
    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.points = []
    jp = JointTrajectoryPoint()
    jp.time_from_start = rospy.Time.from_seconds( time_from_start )
    jp.positions = positions
    jt.points.append( jp )
    
    # push trajectory goal to ActionServer
    jt_goal = JointTrajectoryGoal()
    jt_goal.trajectory = jt
    jt_goal.trajectory.header.stamp = rospy.Time.now() # + rospy.Duration.from_sec( time_from_start )
    client.send_goal( jt_goal )
    client.wait_for_result()    
    return client.get_state()
        

def track_trajectory( pos, vel, acc, time, 
                      arm = 'r', client = None,
                      stamp = None):

    if (acc == None):
        acc = np.zeros( pos.shape )

    if ( len( pos ) != len( vel ) or len( pos ) != len( time ) ):
        rospy.logerr( '[track_trajectory] dimensions do not agree' )

    rospy.loginfo( 'arm \'%s\' tracking trajectory with %d points'
                   %( arm, len( pos ) ) )    

    if (client == None):
        client = init_jt_client(arm)
    else:
        arm = client.action_client.ns[0:1]; # ignore arm argument

    joint_names = get_joint_names( ''.join( ( arm, '_arm_controller' ) ) )

    # create trajectory message
    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.points = []

    t = 0
    for i in range( len( pos ) ):
        jp = JointTrajectoryPoint()
        jp.time_from_start = rospy.Time.from_sec( time[i] )
        jp.positions = pos[i,:]
        jp.velocities = vel[i,:]
        jp.accelerations = acc[i,:]
        jt.points.append( jp )
        t += 1

    if ( stamp == None ):
        stamp = rospy.Time.now()

    # push trajectory goal to ActionServer
    jt_goal = JointTrajectoryGoal()
    jt_goal.trajectory = jt
    jt_goal.trajectory.header.stamp = stamp;
    client.send_goal(jt_goal)

