#!/usr/bin/python

from scipy import pi, sin, cos, array, asarray, linspace, zeros, ones, sqrt
import matplotlib.pyplot as plt  

import roslib
roslib.load_manifest( 'pr2_controllers_msgs' )
roslib.load_manifest( 'trajectory_msgs' )
roslib.load_manifest( 'actionlib' )
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib


def main():
    joint_names = ['l_shoulder_pan_joint',
                   'l_shoulder_lift_joint',
                   'l_upper_arm_roll_joint',
                   'l_elbow_flex_joint',
                   'l_forearm_roll_joint',
                   'l_wrist_flex_joint',
                   'l_wrist_roll_joint']

    N = len( joint_names )

    # lower_limit = asarray( [-2.13, -0.35, -3.75, -2.32, -2.0 * pi, -2.09, -2.0 * pi ] )
    # upper_limit = asarray( [ 0.56, 
    #                          0.8, #1.29, 
    #                          0.65,  0.0,   2.0 * pi,   0.0,  2.0 * pi ] )

    lower_limit = asarray( [2.13,
                            -0.35,
                            -0.5, #3.75,
                            -2.0, #-2.32,
                            -2.0 * pi,
                            -2.09,
                            -2.0 * pi ] )
    upper_limit = asarray( [-0.56, 
                             0.6, #1.29, 
                             2, #0.65,
                             0.0,
                             2.0 * pi,
                             0.0,
                             2.0 * pi ] )

    A =  (lower_limit - upper_limit) * 1/4 

    f = 0.1

    f1 = asarray([f * sqrt(2), 
                  f * sqrt(2), 
                  f * sqrt(2), 
                  f * sqrt(9),  
                  f * sqrt(7),
                  f * sqrt(1),
                  f * sqrt(7)])
    
    f2 = asarray([0.4, 
                  0.4, 
                  0.6, 
                  0.9, 
                  1.1, 
                  0.3, 
                  0.9])

    start_time = 0.0
    dt = 0.001
    max_time = 60.0
    t = linspace( 0.0, max_time - dt, int( max_time / dt ) )
    
    q = zeros( (N, len( t )))
    dq = zeros( (N, len( t )))
    ddq = zeros( (N, len( t )))

    # start_pos = [ 0, 0, 0, 0, 0, 0, 0 ]
    # joint = 2;

    for i in range(N):
        #if i == joint:
            q[i,:] = A[i] * sin( 2.0 * pi * f1[i] * t ) \
                + A[i]/3.0 * sin( 2.0 * pi * f2[i] * t ) + ( upper_limit[i] + lower_limit[i]) / 2.0
            dq[i,:] = 2.0 * pi * f1[i] * A[i] * cos( 2.0 * pi * f1[i] * t ) \
                + 2.0/3.0 * pi * f2[i] * A[i] * cos( 2.0 * pi * f2[i] * t )
            ddq[i,:] = - 4.0 * pi ** 2 * f1[i] ** 2 * A[i] * sin( 2.0 * pi * f1[i] * t ) \
                - 4.0/3.0 * pi ** 2 * f2[i] ** 2 * A[i] * sin( 2.0 * pi * f2[i] * t )
        #else:
        #     q[i,:] = ones( (1, len( t )) ) * start_pos[i]


    # plt.subplot( 3, 1, 1);
    # plt.plot( q[joint,:] )
    # plt.subplot( 3, 1, 2);
    # plt.plot( dq[joint,:] )
    # plt.subplot( 3, 1, 3);
    # plt.plot( ddq[joint,:] )
    # plt.show()                  
    
    rospy.init_node( 'gpr_controller_trajectory_generator' )
    

    client = actionlib.SimpleActionClient( 'l_arm_controller/joint_trajectory_action', 
                                           JointTrajectoryAction )
    client.wait_for_server()

    jt = JointTrajectory()
    jt.joint_names = joint_names
    jt.points = []

    # start at neutral position
    jp = JointTrajectoryPoint()
    jp.time_from_start = rospy.Time.from_seconds( start_time )
    jp.positions = q[:,0]
    jp.velocities = [0] * len( joint_names )
    jp.accelerations = [0] * len( joint_names )
    jt.points.append( jp )

    # add precomputed trajectory
    for i in range( len( t ) ):
        jp = JointTrajectoryPoint()
        jp.time_from_start = rospy.Time.from_seconds( t[i] + start_time )
        jp.positions = q[:,i]
        jp.velocities = dq[:,i]
        jp.accelerations = ddq[:,i]
        jt.points.append( jp )

    # push trajectory goal to ActionServer
    goal = JointTrajectoryGoal()
    goal.trajectory = jt
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(4.0)
    client.send_goal(goal)
    client.wait_for_result()
    

if __name__ == "__main__":
    main()
