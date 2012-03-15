#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )

import rospy

import sys
import numpy as np
import dynamics_utils as dynutils

if __name__== '__main__':

    if ( len( sys.argv ) != 2 ):
        rospy.logerr( 'no filename passed to play_motion.py' )
        exit(-1)
    
    fname = sys.argv[1];

    node_name = "play_motion"
    rospy.loginfo( 'register node %s ...'%node_name )
    rospy.init_node( node_name )
    rospy.loginfo( 'node %s up and running!'%node_name )

    print dynutils.get_joint_names( 'l_arm_controller' )

    # move right arm out of the way
    dynutils.move_arm( [ -np.pi / 3, 
                          np.pi / 3, 
                          0,
                          -3 * np.pi/4,
                          0, 
                          0, 
                          0], arm = 'r' )

    # load motion
    ( name, time, pos, vel, acc ) = dynutils.load_motion( fname );
    dt = time[1] - time[0]
    total_time = dt * len( time )
    
    
    # start client for left arm
    l_jt_client = dynutils.init_jt_client(arm = 'l')
    
    # move left arm to starting posiiton
    dynutils.move_arm( pos[0,:], 
                       time_from_start = 3.0, 
                       client = l_jt_client ) # left arm goes to first pos of trajectory

    rospy.loginfo( 'playing \'%s\' motion ...'%name )

    # initial call
    last_call = rospy.Time().now() + rospy.Duration().from_sec( .5 );
    dynutils.track_trajectory(pos, vel, acc, time, arm = 'l', client = l_jt_client, stamp = last_call )

    # loop
    while ( not rospy.is_shutdown() ):
        dur = rospy.Time().now() - last_call;
        if ( dur.to_sec() > total_time - (total_time / 2.0) ):
            last_call += rospy.Duration().from_sec( total_time );
            dynutils.track_trajectory(pos, vel, acc, time, arm = 'l', client = l_jt_client, stamp = last_call )
