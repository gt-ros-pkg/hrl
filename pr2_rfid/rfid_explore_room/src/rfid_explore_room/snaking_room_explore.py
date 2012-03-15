#!/usr/bin/python

import roslib
roslib.load_manifest('rfid_explore_room')
roslib.load_manifest('explore_hrl')
import rospy

import tf
import tf.transformations as tft
import actionlib
import rfid_explore_room.util as ut
from actionlib.simple_action_client import SimpleGoalState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.srv import GetPlan, GetPlanRequest
from rfid_explore_room.srv import ExploreRoomSrv, ExploreRoomSrvResponse
from explore_hrl.msg import ExploreAction, ExploreResult, ExploreGoal

import yaml
import numpy as np, math
import string
import os


def retfalse():
    return False


class SnakingExplore( ):
    def __init__( self, df, room, start_rad = 1.5, markers_update = lambda x: True ):
        try:
            rospy.init_node( 'SnakingExplore' )
        except:
            pass

        self.frame = '/' + room

        self.markers_update = markers_update
        
        self.length = df[room]['length']
        self.width = df[room]['width']

        rospy.logout( 'snaking_explore: Initializing' )

        self.listener = tf.TransformListener()
        self.listener.waitForTransform( '/base_link', '/map', rospy.Time(0), timeout = rospy.Duration(100))
        self.listener.waitForTransform( self.frame, '/map', rospy.Time(0), timeout = rospy.Duration(100))

        self.setup_poses( radius = start_rad )

        self.client = actionlib.SimpleActionClient( 'move_base', MoveBaseAction )
        self.client.wait_for_server()

        self._service = rospy.Service( '/explore/explore' , ExploreRoomSrv, self.service_request )

        self._as = actionlib.SimpleActionServer( '/explore', ExploreAction, execute_cb = self.action_request )
        self._as.start()

        rospy.logout( 'snaking_explore: Ready and waiting.' )

    def service_request( self, req ):
        res = self.begin_explore( req.radius )
        rospy.logout( 'snaking_explore: Returning result \'%s\'' % res )
        return res        

    def action_request( self, goal ):
        rospy.logout( 'snaking_explore: action_request received for radius: \'%2.2f\'' % goal.radius )
        
        def preempt_func():
            # self._as should be in scope and bound @ function def. (I think for python...)
            check = self._as.is_preempt_requested()
            if check:
                rospy.logout( 'snaking_explore: action_request preempted!' )
            return check

        rv = self.begin_explore( goal.radius, preempt_func = preempt_func )
        rospy.logout( 'snaking_explore: action_request received result %d' % int(rv) )

        if rv == 'succeeded':
            rospy.logout( 'snaking_explore: returning \'succeeded\'' )
            self._as.set_succeeded( ExploreResult() )
        elif rv == 'preempted':
            rospy.logout( 'snaking_explore: returning \'preempted\'' )
            self._as.set_preempted( )
        else: # rv == 'aborted'
            rospy.logout( 'snaking_explore: returning \'aborted\'' )
            self._as.set_aborted()

    def robot_in_map( self ):
        ps_base = PoseStamped()
        ps_base.header.frame_id = '/base_link'
        ps_base.header.stamp = rospy.Time(0)
        ps_base.pose.orientation.w = 1.0
        return self.ps_in_map( ps_base )

    def ps_in_map( self, ps ):
        # ps is pose_stamped
        try:
            ps_map = self.listener.transformPose( '/map', ps )
        except:
            rospy.logout( 'snaking_explore: %s -> map Transform failed.' % ps.header.frame_id )
            ps_map = None
        return ps_map

    def range_to_waypoint( self, r, w ):
        # r => robot in map, w => waypoint in map
        return np.sqrt( (r.pose.position.x - w.pose.position.x)**2.0 +
                        (r.pose.position.y - w.pose.position.y)**2.0 )

    def begin_explore( self, radius, preempt_func = retfalse ):
        rospy.logout( 'snaking_explore: setting radius' )
        
        waypoints = self.setup_poses( radius ) # PoseStamped in self.frame

        local_planner = rospy.get_param('/move_base_node/base_local_planner') # : dwa_local_planner/DWAPlannerROS
        local_planner = local_planner[string.find(local_planner,'/')+1:]
        obs_range = rospy.get_param('/move_base_node/global_costmap/obstacle_range')
        move_tol = rospy.get_param('/move_base_node/'+local_planner+'/xy_goal_tolerance')
        no_progress = rospy.get_param( rospy.get_name() + '/no_progress_move', 5)
        unreached_time = rospy.get_param( rospy.get_name() + '/not_reached_move', 30)

        # I'm not entirely sure which service to use.  I do know that
        # non-NavfnROS make_plan sometimes fails for an unknown
        # reason... and thus NavfnROS/make_plan is more robust.

        # srv_name = '/move_base_node/make_plan'
        srv_name = '/move_base_node/NavfnROS/make_plan'
        get_plan = rospy.ServiceProxy( srv_name, GetPlan )

        # Clear costmap...?  Do this here or in smach...?
        
        if preempt_func(): # immediately exit if overall action preempted
            return 'preempted'

        for i,w in enumerate( waypoints ):
            if preempt_func(): # immediately exit if overall action preempted
                return 'preempted'

            rospy.logout( 'snaking_explore: Seeking waypoint %d of %d' % (i+1,len(waypoints)))
            # rospy.logout( 'snaking_explore: %2.2f %2.2f in frame %s' % (w.pose.position.x, w.pose.position.y, w.header.frame_id))

            rim = self.robot_in_map()
            w.header.stamp = rospy.Time(0) # transform it _now_
            wim = self.ps_in_map( w )      # waypoint in map
            
            if not rim or not wim: # if transforms failed
                rospy.logout( 'snaking_explore: Bad transforms. Aborting explore' )
                return 'aborted'

            if self.range_to_waypoint( rim, wim ) < 0.9 * obs_range:
                # We're nearby the waypoint, so we'll just trust that the planner
                # has a good view of its surroundings to determine reachability.

                req = GetPlanRequest()
                req.tolerance = 0.1
                req.start = rim
                req.goal = wim
                resp = get_plan( req )
                found_plan = bool( resp.plan.poses != [] )

                if not found_plan:
                    rospy.logout( 'snaking_explore: No plan to nearby waypoint. Proceeding to next waypoint' )
                    # Perhaps its worth pulling the waypoint closer until a path _is_ found?
                    continue

                # We now know that a path exists.  Send the waypoint to the client.
                rospy.logout( 'snaking_explore: Near by with good plan.  Calling move_base action.' )

            else: 
                # Any nav plan at beyond obstacle range will require the
                # nav stack to get a better vantage.  Send the waypoint to
                # the client.
                rospy.logout( 'snaking_explore: Far away.  Calling move_base action.' )


            # If we made it this far, it's time to call move_base action.
            # Cancel any previous goals
            self.client.cancel_goals_at_and_before_time( rospy.Time.now() )
            if os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim':
                rospy.logout( 'running in sim: give the system a little time to respond' )
                rospy.sleep( 1 )

            # Send our new goal
            rospy.logout( 'snaking_explore: sending movebase action goal.' )
            move_goal = MoveBaseGoal()
            move_goal.target_pose = wim
            self.client.send_goal( move_goal )
            if os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim':
                rospy.logout( 'running in sim: give the system a little time to respond' )
                rospy.sleep( 1 )

            # We'll monitor the state ourselves, and allow ourselves
            # the opportunity to cancel the goal for various reasons
            # (eg. if we haven't moved for a while or if new plans
            # aren't found after getting within obstacle distance)

            # When this loop is broken, we'll go to the next goal.  We
            # don't need to cancel the goals here -- they will be
            # cancelled before a new one is sent.
            r = rospy.Rate( 10 )
            xytt_old = None # Checks for movement
            stime = rospy.Time.now().to_sec()
            while True:
                if self.client.simple_state == SimpleGoalState.DONE:
                    res = self.client.get_result()
                    rospy.logout('snaking_explore: Movebase actionlib completed with result.  Proceeding to next waypoint')
                    break

                rim = self.robot_in_map()
                w.header.stamp = rospy.Time( 0 )
                wim = self.ps_in_map( w )

                # Proceed when close enough to goal (within two tolerance factors)
                if rim and wim: # only do this check if transform succeeds
                    if self.range_to_waypoint( rim, wim ) < move_tol * 2.0:
                        rospy.logout( 'snaking_explore: Close enough to waypoint.  Proceeding to next waypoint' )
                        break # We're near the waypoint, so let's must move on to next

                # Proceed when there has been no movement (x,y, or theta) for X sec.
                if rim: # only do this check if transform succeeds
                    xytt = self.xythetatime( rim )
                    if not xytt_old:
                        xytt_old = xytt

                    if np.abs( xytt[0] - xytt_old[0] ) > 0.05 or \
                       np.abs( xytt[1] - xytt_old[1] ) > 0.05 or \
                       np.abs( xytt[2] - xytt_old[2] ) > math.radians(10):
                        xytt_old = xytt

                    if xytt[3] - xytt_old[3] > no_progress:
                        rospy.logout( 'snaking_explore: No movement in %2.1f seconds.  Proceeding to next waypoint' % no_progress )
                        break

                if rospy.Time.now().to_sec() - stime > unreached_time:
                        rospy.logout( 'snaking_explore: Goal unreached in %2.1f seconds.  Proceeding to next waypoint' % unreached_time )
                        break
                
                r.sleep()
                
        self.client.cancel_all_goals()
        rospy.logout( 'snaking_explore: Finished with exploration.' )

        return 'succeeded'

    def xythetatime( self, ps ):
        o = ps.pose.orientation
        r,p,y = tft.euler_from_quaternion(( o.x, o.y, o.z, o.w ))
        return ps.pose.position.x, ps.pose.position.y, y, rospy.Time.now().to_sec()
                        
    def setup_poses( self, radius ):
        # poses = self.setup_poses_xonly( radius )
        poses = self.setup_poses_snaking( radius )
        self.markers_update( poses )
        return poses

    def setup_poses_snaking( self, radius ):
        # tt are the min's and maxes of the snake along x-axis
        cycles = np.ceil( self.length / radius )
        tt = np.linspace( 0.0, self.length, 2 * cycles + 1 )

        # Determine waypoints at crest and valleys (lengthwise)
        crests = []
        atmin = False
        for xx in tt:
            atmin = not atmin # alternate between ys = 0 and ys = width
            yy = float( atmin ) * self.width
            crests.append( (xx, yy) )
            
        # Determine intermediate waypoints (widthwise)
        xyinter = []
        inter = np.ceil( self.width / radius )
        x_next, y_next = 0.0, 0.0
        for i in xrange( len(crests) - 1 ):
            x_prev = crests[i][0]
            y_prev = crests[i][1]
            x_next = crests[i+1][0]
            y_next = crests[i+1][1]
            
            x_inter = np.linspace( x_prev, x_next, inter * 2, endpoint = False )
            y_inter = np.linspace( y_prev, y_next, inter * 2, endpoint = False )

            xyinter += zip( x_inter, y_inter )
        xyinter += [ (x_next, y_next) ]

        # Determine headings (in self.frame)
        xytheta = []
        def add_xytheta( x, y, t ):
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.header.stamp = rospy.Time(0)

            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, t ))

            xytheta.append( ps )

        theta = 0.0
        for i in xrange( len(xyinter) - 1 ):
            theta = np.arctan2( xyinter[i+1][1] - xyinter[i][1],
                                xyinter[i+1][0] - xyinter[i][0] )
            add_xytheta( xyinter[i][0], xyinter[i][1], theta )
        add_xytheta( xyinter[-1][0], xyinter[-1][1], theta )

        return xytheta            

    def setup_poses_xonly( self, radius ):
        poses = []
        xpos = 0.0
        
        def ps_x( x ):
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = x
            ps.pose.orientation.w = 1.0
            return ps
            
        for i in xrange(3):
            poses.append( ps_x( xpos ))
            xpos += 0.5
        xpos += 3.0
        for i in xrange(3):
            poses.append( ps_x( xpos ))
            xpos += 0.5

        return poses



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--yaml', action='store', type='string', dest='yaml',
                 help='Room definition file (required)', default='')
    p.add_option('--room', action='store', type='string', dest='room',
                 help='Specific room [default \'hrl\']', default='hrl')
    opt, args = p.parse_args()

    if opt.yaml == '':
        print 'room_explore: YAML FILE REQUIRED.'
    else:
        print 'Using YAML file: ', opt.yaml
    
    f = open( opt.yaml )
    dat = yaml.load( f )
    f.close()
    
    room_transforms = ut.TFthread( dat )
    pm = ut.PosesMarkers()
    
    re = SnakingExplore( df = dat,
                         room = opt.room,
                         start_rad = 1.5,
                         markers_update = pm.update_poses )
    # rospy.sleep( 4 )
    # re.publish_markers()
    
    rospy.spin()
    
    # r = rospy.Rate( 2 )
    # while not rospy.is_shutdown():
    #     re.publish_markers()
    #     r.sleep()

    #re.begin_explore( 0.8 )
    # rosservice call /explore 0.8
    
    # explore_act = actionlib.SimpleActionClient('explore', ExploreAction)
    # rospy.logout( 'Waiting for explore.' )
    # explore_act.wait_for_server()
    # rospy.logout( 'Done waiting.' )

    # goal = ExploreGoal( radius = 0.8 )
    # explore_act.send_goal(goal)
    # rospy.sleep( 0.5 )
    # explore_act.wait_for_result()
        
    
        
