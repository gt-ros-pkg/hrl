#!/usr/bin/python

import roslib
roslib.load_manifest('rfid_explore_room')
import rospy

import tf
from threading import Thread
import actionlib

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA

from explore_hrl.msg import ExploreAction, ExploreResult, ExploreGoal

import yaml
import numpy as np, math

def retfalse():
    return False

class TFthread( Thread ):
    def __init__( self, df ):
        Thread.__init__( self )
        try:
            rospy.init_node( 'TFthread' )
        except:
            pass # parent probably already initialized node.
        self.df = df # yaml dictionary of rooms
        self.bc = tf.TransformBroadcaster()
        self.should_run = True
        self.start()

    def publish_transform( self ):
        for room in self.df.keys():
            self.bc.sendTransform( ( self.df[room]['position']['x'],
                                     self.df[room]['position']['y'],
                                     self.df[room]['position']['z'] ),
                                   ( self.df[room]['orientation']['x'],
                                     self.df[room]['orientation']['y'],
                                     self.df[room]['orientation']['z'],
                                     self.df[room]['orientation']['w'] ),
                                   rospy.Time.now(),
                                   room,
                                   'map' )

    def run( self ):
        rospy.logout( 'TFthread: Starting ' )
        r = rospy.Rate( 10 )
        while self.should_run and not rospy.is_shutdown():
            self.publish_transform()
            try:
                r.sleep()
            except:
                pass # ROS exception.
        rospy.logout( 'TFthread: Starting ')
        
    def stop( self ):
        # Kill off the poller thread.
        self.should_run = False
        self.join(5)
        if (self.isAlive()):
            raise RuntimeError('TFthread: Unable to stop thread ' )


# if __name__ == '__main__':
#     f = open( room )
#     dat = yaml.load( f )
#     f.close()

#     TFthread( dat )
#     rospy.spin()

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi


class RoomExplore( ):
    def __init__( self, df, room, start_rad = 1.5 ):
        try:
            rospy.init_node( 'RoomExplore' )
        except:
            pass
        self.should_run = True
        self.frame = '/' + room
        self.old_markers = []
        self.mid = 1

        self.height = df[room]['height']
        self.width = df[room]['width']

        self.listener = tf.TransformListener()
        self.listener.waitForTransform( '/base_link', '/map', rospy.Time(0), timeout = rospy.Duration(100))
        self.pub = rospy.Publisher( 'visarr', Marker )

        self.setup_poses( radius = start_rad ) # also initializes radius
        print 'Len: ', len(self.poses), ' Publishing.'
        self.publish_markers()

        self._as = actionlib.SimpleActionServer( '/explore', ExploreAction, execute_cb = self.action_request )
        self._as.start()
        self.publish_markers()
        rospy.logout('Action should be started.')

    def action_request( self, goal ):
        rospy.logout( 'room_explore: action_request received for radius: \'%2.2f\'' % goal.radius )
        
        def preempt_func():
            # self._as should be in scope and bound @ function def. (I think for python...)
            check = self._as.is_preempt_requested()
            if check:
                rospy.logout( 'room_explore: action_request preempted!' )
            return check

        rv = self.begin_explore( goal.radius, preempt_func = preempt_func )
        rospy.logout( 'room_explore: action_request received result %d' % int(rv) )

        if rv == True:
            self._as.set_succeeded( ExploreResult() )


    def begin_explore( self, radius, preempt_func = retfalse ):
        client = actionlib.SimpleActionClient( 'move_base', MoveBaseAction )
        rospy.logout( 'Waiting for move_base server' )
        client.wait_for_server()

        rospy.logout( 'room_explore: setting radius' )
        all_goals = self.setup_poses( radius )
        self.publish_markers()
        for goal in all_goals:
            #print 'room_explore: sending goal ', goal
            goal.target_pose.header.stamp = rospy.Time.now()

            client.send_goal( goal )

            time_last_moving = rospy.Time.now()
            goal_time = rospy.Time.now()

            new_pose = self.current_robot_position()
            ref_pose = self.current_robot_position()

            r = rospy.Rate( 5 )
            while not rospy.is_shutdown():
                state = client.get_state()
                states = { 0: 'WAITING FOR GOAL ACK',
                           1: 'PENDING',
                           2: 'ACTIVE',
                           3: 'WAITING FOR RESULT',
                           4: 'WAITING FOR CANCEL ACK',
                           5: 'RECALLING',
                           6: 'PREEMPTING',
                           7: 'DONE' }
                #print 'State: ', state, ' ', states[state]
                #print 'Result: ', client.get_result()
                
                #rospy.logout( 'room_explore: loop' )
                if state == 7 or state == 3:
                    rospy.logout( 'room_explore: client no longer active' )
                    print 'State: ', state, ' ', states[state]
                    time_last_moving = rospy.Time.now()
                    break

                if preempt_func():
                    rospy.logout( 'room_explore: preempted at a higher level.' )
                    time_last_moving = rospy.Time.now()
                    break

                new_pose = self.current_robot_position()
                dx = new_pose[0] - ref_pose[0]
                dy = new_pose[1] - ref_pose[1]
                da = new_pose[-1] - ref_pose[-1] # yaw
                
                if dx*dx + dy*dy > 0.02 or da*da > math.radians( 5 ):
                    time_last_moving = rospy.Time.now()
                    ref_pose = self.current_robot_position()
                    rospy.logout('WE ARE MOVING')

                if rospy.Time.now() - time_last_moving > rospy.Duration( 8 ):
                    rospy.logout( 'We do not appear to have moved.  Aborting current goal.' )
                    client.cancel_all_goals() # Should force a break on GoalState
                    time_last_moving = rospy.Time.now()
                    break

                if rospy.Time.now() - goal_time > rospy.Duration( 30 ):
                    rospy.logout( 'Goal not achieved after 30 seconds.  Aborting.' )
                    client.cancel_all_goals() # Should force a break on GoalState
                    time_last_moving = rospy.Time.now()
                    break

                r.sleep()
                
            client.cancel_all_goals() # Just in case

            if preempt_func(): # immediately exit if overall action preempted
                break

        return True
                        

    def current_robot_position( self ):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time(0)
        ps.header.frame_id = '/base_link'
        ps.pose.orientation.w = 1.0

        try:
            ps_map = self.listener.transformPose( '/map', ps )
        except:
            rospy.logout( 'room_explore: Transform failed' )
            ps_map = PoseStamped()
            ps_map.header.stamp = rospy.Time.now()
            ps_map.header.frame_id = '/map'
            ps_map.pose.orientation = new_quat

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(( ps_map.pose.orientation.x,
                                                                      ps_map.pose.orientation.y,
                                                                      ps_map.pose.orientation.z,
                                                                      ps_map.pose.orientation.w ))

        rv = [ ps_map.pose.position.x,
               ps_map.pose.position.y,
               ps_map.pose.position.z,
               roll,
               pitch,
               yaw ]
        #print 'RV: ', rv
            
        return rv
        

    def setup_poses( self, radius ):
        self.radius = radius
        xdir = np.arange( self.radius / 2.0, self.height + self.radius / 2.0, self.radius )
        ydir = np.arange( self.radius / 2.0, self.width + self.radius / 2.0, self.radius )

        move_dir = 0.0
        self.poses = []
        mod = 0
        for y in ydir:
            if mod == 0:
                xord = xdir
            else:
                xord = xdir[::-1]
                
            for x in xord:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.frame
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                quat = tf.transformations.quaternion_from_euler( 0.0, 0.0, move_dir )
                goal.target_pose.pose.orientation.x = quat[0]
                goal.target_pose.pose.orientation.y = quat[1]
                goal.target_pose.pose.orientation.z = quat[2]
                goal.target_pose.pose.orientation.w = quat[3]

                self.poses.append( goal )
            move_dir = standard_rad( move_dir + np.pi )
            mod = ( mod + 1 ) % 2
        return self.poses

    def destroy_old_markers( self ):
        for m in self.old_markers:
            m.action = m.DELETE
            self.pub.publish( m )
        self.old_markers = []
    
    def publish_markers( self ):
        self.destroy_old_markers()
        for i,g in enumerate(self.poses):
            self.mid += 1
            m = Marker()
            m.ns = 'explore_poses'
            m.id = self.mid
            m.action = m.ADD
            m.type = m.ARROW
            m.header.frame_id = self.frame
            m.header.stamp = rospy.Time.now()
            m.scale = Vector3( 0.15, 1.0, 1.0 )
            m.color = ColorRGBA( 0.2, 1.0, 0.2, 0.7 )
            m.pose = g.target_pose.pose

            self.old_markers.append( m )
            self.pub.publish( m )

    def goals( self ):
        return self.poses



#self.explore_act = actionlib.SimpleActionClient('explore', explore_hrl.msg.ExploreAction)



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

    TFthread( dat )    
    
    re = RoomExplore( dat, opt.room, 1.5 )
    rospy.sleep( 4 )
    re.publish_markers()
    
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
        
    
        
