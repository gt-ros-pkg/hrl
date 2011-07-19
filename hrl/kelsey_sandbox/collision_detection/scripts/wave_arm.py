#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'kinematics_msgs' )
roslib.load_manifest( 'geometry_msgs' )

import rospy
from kinematics_msgs.srv import \
    GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import \
    Pose, Point, Quaternion

import numpy as np
import dynamics_utils as dynutils

#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

if __name__== '__main__':
    node_name = "wave_arm"
    rospy.loginfo( 'register node %s ...'%node_name )
    rospy.init_node( node_name )
    rospy.loginfo( 'node %s up and running!'%node_name )

   
    joint_names = dynutils.get_joint_names( 'l_arm_controller' )
    ik_service_name = 'pr2_left_arm_kinematics/get_ik'
    rospy.loginfo( 'wait for service %s ...'%ik_service_name )
    rospy.wait_for_service( ik_service_name )
    ik_service = rospy.ServiceProxy( ik_service_name, GetPositionIK )
    rospy.loginfo( 'ik service %s is up and running!'%ik_service_name )

    # trajectory in cartesian space
    time = 10.0; # trajectory time in sec
    n = int(time * 200)
    dt = time / n;
    alphas = np.linspace( 0, 2 * np.pi, n + 1 )
    alphas = alphas[0:-1]
    ys = np.cos( alphas ) * 0.4
    zs = -np.sin( 2 * alphas ) * 0.2

    # creating trajectory in joint space
    rospy.loginfo( 'creating trajectory ...' )
    trajectory = []
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(5.0)
    req.ik_request.ik_link_name = "l_wrist_roll_link"
    req.ik_request.pose_stamped.header.frame_id = "torso_lift_link"
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.ik_seed_state.joint_state.position =  [0.61309537, 0.45494851, 0.03, -1.03480809,  2.23232079, -0.79696399, -2.44271129]
    joint_positions = []

    for (y, z) in zip( ys, zs ):
        pose = Pose( position = Point( 0.6, 0.20 + y, 0.0 + z ), 
                     orientation = Quaternion( 0.0, 0.0, 0.0, 1.0 ) )
        req.ik_request.pose_stamped.pose = pose
    
        # make the call
        try:
            # print "send request :" #, req
            res = ik_service( req )
            
            if res.error_code.val == res.error_code.SUCCESS:
                joint_positions = np.asarray( res.solution.joint_state.position )
            else:
                print "IK failed, error code : ", res.error_code.val
                break
        except rospy.ServiceException, e:
            print "service call failed: %s"%e
            break
    
        trajectory.append( joint_positions )
        req.ik_request.ik_seed_state.joint_state.position = joint_positions
        
    rospy.loginfo( 'done!' )

    pos = np.asarray( trajectory )
    dynutils.wrap_trajectory( pos )

    vel = dynutils.compute_derivative( pos, dt )
    acc = dynutils.compute_derivative( vel, dt )

    # plt.plot( acc )
    # plt.show()
   
    # move_arms to neutral positions
    dynutils.move_arm( [ -np.pi / 3, 
                          np.pi / 3, 
                          0,
                          -3 * np.pi/4,
                          0, 
                          0, 
                          0], arm = 'r' )

    l_jt_client = dynutils.init_jt_client(arm = 'l')
    dynutils.move_arm( pos[0,:], 
              time_from_start = 5.0, 
              client = l_jt_client ) # left arm goes to first pos of trajectory

    # loop
    last_call = rospy.Time().now() + rospy.Duration().from_sec( .5 );
    dynutils.track_trajectory(pos, vel, acc, dt, arm = 'l', client = l_jt_client, stamp = last_call )

    while ( not rospy.is_shutdown() ):
        dur = rospy.Time().now() - last_call;
        if ( dur.to_sec() > time - (time / 2.0) ):
            last_call += rospy.Duration().from_sec( time );
            dynutils.track_trajectory(pos, vel, acc, dt, arm = 'l', client = l_jt_client, stamp = last_call )
