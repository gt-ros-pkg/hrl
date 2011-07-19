#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'kinematics_msgs' )
roslib.load_manifest( 'geometry_msgs' )
roslib.load_manifest( 'trajectory_msgs' )
roslib.load_manifest( 'pr2_controllers_msgs' )
roslib.load_manifest( 'motion_planning_msgs' )

import rospy
from kinematics_msgs.srv import \
    GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import \
    Pose, Point, Quaternion

from trajectory_msgs.msg import \
    JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import \
    JointTrajectoryAction, JointTrajectoryGoal
from motion_planning_msgs.srv import \
    FilterJointTrajectoryWithConstraints, FilterJointTrajectoryWithConstraintsRequest

import numpy as np
import dynamics_utils as dynutils


class TrajectoryFilter:
    
    def __init__( self, service_name ):
        rospy.loginfo( 'waiting for service: %s'%service_name )
        rospy.wait_for_service( service_name )
        self.proxy = rospy.ServiceProxy( service_name, FilterJointTrajectoryWithConstraints )

    def filter( self, trajectory ):
        # rospy.loginfo( 'filtering trajectory:\n %s'%trajectory )
        request = FilterJointTrajectoryWithConstraintsRequest()
        request.allowed_time = rospy.Duration.from_sec(5.0);
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
        

if __name__== '__main__':
    node_name = "reaching_node"
    rospy.loginfo( 'register node %s ...'%node_name )
    rospy.init_node( node_name )
    rospy.loginfo( 'node %s up and running!'%node_name )

    joint_names = dynutils.get_joint_names( 'l_arm_controller' )
    ik_service_name = 'pr2_left_arm_kinematics/get_ik'
    rospy.loginfo( 'wait for service %s ...'%ik_service_name )
    rospy.wait_for_service( ik_service_name )
    ik_service = rospy.ServiceProxy( ik_service_name, GetPositionIK )
    rospy.loginfo( 'ik service %s is up and running!'%ik_service_name )

    traj_filter = TrajectoryFilter( 'trajectory_filter/filter_trajectory_with_constraints' )

    neutral = [ np.pi / 3, 
                 np.pi / 3, 
                 0,
                 -3 * np.pi/4,
                 0, 
                 0, 
                 0]

    # creating way points in joint space
    rospy.loginfo( 'creating trajectory ...' )
    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(5.0)
    req.ik_request.ik_link_name = "l_wrist_roll_link"
    req.ik_request.pose_stamped.header.frame_id = "torso_lift_link"
    req.ik_request.ik_seed_state.joint_state.name = joint_names
    req.ik_request.ik_seed_state.joint_state.position =  neutral

    joint_positions = []
    
    center = Pose( position = Point( 0.4, 0.2, 0.0 ), 
                   orientation = Quaternion( 0.0, 0.0, 0.0, 1.0 ) )

    home_pose = Pose( position = Point( 0.25, 0.55, 0.0 ), 
                   orientation = Quaternion( 0.0, 0.0, 0.0, 1.0 ) )

    req.ik_request.pose_stamped.pose = home_pose
    try:
        # print "send request :" #, req
        res = ik_service( req )
        if res.error_code.val == res.error_code.SUCCESS:
                joint_positions = np.asarray( res.solution.joint_state.position )
                req.ik_request.ik_seed_state.joint_state.position = joint_positions
        else:
            print "IK failed, error code : ", res.error_code.val
    except rospy.ServiceException, e:
        print "service call failed: %s"%e

    home_joint_positions = joint_positions

    arm = 'l'
    l_arm_client = dynutils.init_jt_client( arm )

    n_points = 50;
    i = 0;
    home = True;

    while i < n_points:

        if not home:

            x = center.position.x + abs(np.random.rand() / 5)
            y = center.position.y + np.random.rand() / 2
            z = center.position.z + np.random.rand() / 10.0
            
            pose = Pose( position = Point( x, y, z ),
                         orientation = Quaternion( 0.0, 0.0, 0.0, 1.0 ) )
            
            i = i + 1

            req.ik_request.pose_stamped.pose = pose
            # make the call
            try:
                # print "send request :" #, req
                res = ik_service( req )
                if res.error_code.val == res.error_code.SUCCESS:
                    joint_positions = np.asarray( res.solution.joint_state.position )
                    req.ik_request.ik_seed_state.joint_state.position = joint_positions
                else:
                    print "IK failed, error code : ", res.error_code.val
                    break
            except rospy.ServiceException, e:
                print "service call failed: %s"%e
                break
            
            dynutils.move_arm(joint_positions, time_from_start = 1.0, client = l_arm_client)
        else:
            dynutils.move_arm(home_joint_positions, time_from_start = 1.0, client = l_arm_client)

        home = not home
   
    dynutils.move_arm(home_joint_positions, time_from_start = 1.0, client = l_arm_client)     
        

    # waypoints = []

    # while len( waypoints ) < n_points:
    #     x = center.position.x;
    #     y = center.position.y + np.random.rand() / 5.0;
    #     z = center.position.z + np.random.rand() / 5.0;

    #     pose = Pose( position = Point( x, y, z ),
    #                  orientation = Quaternion( 0.0, 0.0, 0.0, 1.0 ) )
    #     req.ik_request.pose_stamped.pose = pose
    #     # make the call
    #     try:
    #         # print "send request :" #, req
    #         res = ik_service( req )
    #         if res.error_code.val == res.error_code.SUCCESS:
    #             joint_positions = np.asarray( res.solution.joint_state.position )
    #             waypoints.append( joint_positions )
    #             req.ik_request.ik_seed_state.joint_state.position = joint_positions
    #         else:
    #             print "IK failed, error code : ", res.error_code.val
    #             break
    #     except rospy.ServiceException, e:
    #         print "service call failed: %s"%e
    #         break
    #     pass

    # tmp_points = np.asarray( waypoints )
    # waypoints = []
    # for i in range(n_points ):
    #     waypoints.append( neutral )
    #     waypoints.append( tmp_points[i,:] )
    #     waypoints.append( neutral )
    # print( waypoints )

    # # creating trajectory
    # time = 3.0;
    # time_step = 2.0;

    # arm = 'l'
    # client = dynutils.init_jt_client( arm )
    # joint_names = dynutils.get_joint_names( ''.join( ( arm, '_arm_controller' ) ) )


    # jt = JointTrajectory()
    # jt.joint_names = joint_names
    # jt.points = []
    
    # for i in range( len( waypoints ) ):
    #     jp = JointTrajectoryPoint()
    #     jp.time_from_start = rospy.Time.from_seconds( time )
    #     jp.positions = waypoints[i]
    #     jt.points.append( jp )
    #     time = time + time_step;

    # print( len( jt.points ) )
    # jt.header.stamp = rospy.Time.now()
    # # jt = traj_filter.filter( jt )
    # print( len( jt.points ) )


    # # push trajectory goal to ActionServer
    # jt_goal = JointTrajectoryGoal()
    # jt_goal.trajectory = jt
    # jt_goal.trajectory.header.stamp = rospy.Time.now()
    # client.send_goal( jt_goal )
    # client.wait_for_result()    



    
# #    dynutils.save_motion( 'reaching.pkl', 'reaching', t, pos, vel, acc )
