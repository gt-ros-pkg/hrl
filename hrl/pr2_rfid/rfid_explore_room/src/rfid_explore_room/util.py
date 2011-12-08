#!/usr/bin/python

import roslib
roslib.load_manifest('room_explore')
import rospy

import tf

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA

from threading import Thread
import numpy as np, math

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


class PosesMarkers( Thread ):
    def __init__( self ):
        Thread.__init__( self )
        try:
            rospy.init_node( 'snaking_room_posesmarkers' )
        except:
            pass # parent probably already initialized node.

        self.poses = []
        self.markers = []
        self.mid = 0
        self.new_poses = False
        
        self.should_run = True
        self.pub = rospy.Publisher( 'visarr', Marker )
        self.start()

    def update_poses( self, poses ):
        self.poses = list( poses )
        self.new_poses = True

    def run( self ):
        rospy.logout( 'PosesMarkers: Starting ' )
        r = rospy.Rate( 5 )
        while self.should_run and not rospy.is_shutdown():
            if self.new_poses == True:
                self.new_poses = False

                # Delete old markers
                for old_m in self.markers:
                    old_m.action = old_m.DELETE
                    self.pub.publish( old_m )

                # Create new markers
                self.markers = []
                for p in self.poses:
                    self.mid += 1
                    m = Marker()
                    m.ns = 'explore_poses'
                    m.id = self.mid
                    m.action = m.ADD
                    m.type = m.ARROW
                    m.header.frame_id = p.header.frame_id
                    m.header.stamp = rospy.Time.now()
                    m.scale = Vector3( 0.15, 0.75, 0.75 )
                    m.color = ColorRGBA( 0.2, 0.2, 1.0, 0.9 )
                    m.pose = p.pose
                    self.markers.append( m )

            # Publish markers.
            for m in self.markers:
                m.header.stamp = rospy.Time.now()
                self.pub.publish( m )

            r.sleep()

        # Delete old markers
        for old_m in self.markers:
            old_m.action = old_m.DELETE
            self.pub.publish( old_m )

        rospy.logout( 'PoseMarkers: Stopping. ')
        
    def stop( self ):
        # Kill off the poller thread.
        self.should_run = False
        self.join(5)
        if (self.isAlive()):
            raise RuntimeError('TFthread: Unable to stop thread ' )



def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi


def robot_in_map( l, fail_msg = '' ):
    # l is tf listener
    ps = PoseStamped()
    ps.header.frame_id = '/base_link'
    ps.pose.orientation.w = 1.0

    try:
        ps_map = l.transformPose( '/map', ps )
    except:
        rospy.logout( fail_msg + 'Transform failed.' )
        ps_map = None
    return ps_map

