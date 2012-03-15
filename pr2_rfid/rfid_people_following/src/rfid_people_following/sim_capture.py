#! /usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest( 'move_base_msgs' )
roslib.load_manifest('tf')
roslib.load_manifest('std_srvs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')
roslib.load_manifest('rfid_people_following')
import rospy

import tf
import tf.transformations as tft
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from rfid_people_following.srv import *

import numpy as np, math
import os
import time
import pickle as pkl
from threading import Thread
from collections import deque


class BasePose( Thread ):
    def __init__( self ):
        self.should_run = True
        Thread.__init__( self )

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.ps = PointStamped()
        self.ps.header.frame_id = '/base_link'
        self.pts = []
        self.start()

    def run( self ):
        rate = rospy.Rate( 5 )
        while self.should_run and not rospy.is_shutdown():
            try:
                self.ps.header.stamp = rospy.Time(0)
                ps_map = self.listener.transformPoint( '/odom_combined', self.ps )
                x = ps_map.point.x
                y = ps_map.point.y
                self.pts.append([ x, y ])

                
            except:
                rospy.logout( 'base_pose: Failed transform.' )
                pass
            rate.sleep()
        

    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ServoMode: unable to stop thread")
        


#        sm = ServoMode( self.follow1 )

class ServoMode( Thread ):
    def __init__( self, func ):
        self.done = False
        self.should_run = True
        Thread.__init__( self )
        self.func = func
        self.start()

    def run( self ):
        self.func()
        self.done = True
        while self.should_run and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ServoMode: unable to stop thread")
        

class SimCapture():
    def __init__( self ):
        try:
            rospy.init_node( 'sim_capture' )
        except:
            pass

        rospy.logout( 'sim_capture: Initializing' )
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        
        rospy.logout( 'sim_capture: Waiting for services' )
        self._sh = rospy.Service( '/sim_capture/capture' , FloatFloatFloatFloat_Int32, self.capture )
        rospy.wait_for_service( '/rfid_servo/servo' )
        rospy.wait_for_service( '/rfid_servo/abort' )
        rospy.wait_for_service( '/rotate_backup/navstack' )

        self._servo = rospy.ServiceProxy( '/rfid_servo/servo', StringInt32_Int32 )
        self.follow1 = lambda : self._servo( 'person      ', 1 ) # Stops at first obs

        self._abort = rospy.ServiceProxy( '/rfid_servo/abort', Empty )
        self.abort = lambda : self._abort( ) # Kill current servoing

        self._navstack = rospy.ServiceProxy( '/rotate_backup/navstack' , FloatFloatFloatFloat_Int32 )
        self.navstack = lambda x,y,z,theta: self._navstack( x,y,z,theta )


        rospy.logout( 'sim_capture: Services ready' )

    def capture( self, request, fname = None ):
        rospy.logout( 'sim_capture: New capture initiated @ %3.2f.' % rospy.Time.now().to_sec() )
        rospy.logout( 'sim_capture: Moving to <%3.2f, %3.2f, %3.2f> %3.2f-deg' % ( request.x,
                                                                                   request.y,
                                                                                   request.z,
                                                                                   math.degrees( request.ang )))
        self.navstack( request.x, request.y, request.z, request.ang )
        rospy.logout( 'sim_capture: Arrived at location.' )
        rospy.logout( 'sim_capture: Initializing recorder and servoing.' )

        
        ps = PointStamped()
        ps.header.frame_id = '/base_link'

        ps2 = PointStamped()
        ps2.header.frame_id = '/base_link'
        ps2.point.x = 0.1

        # Begin Servoing.  Warning: Hack!
        sm = ServoMode( self.follow1 )
        #bp = BasePose( )
        pts = []
        
        t0 = rospy.Time.now().to_sec()
        while sm.done == False:
            if rospy.Time.now().to_sec() - t0 > 180:
                rospy.logout( 'sim_capture: Time up.  Aborting.' )
                self.abort()

            try:
                ps.header.stamp = rospy.Time(0)
                ps_map = self.listener.transformPoint( '/map', ps )
                x = ps_map.point.x
                y = ps_map.point.y

                ps2.header.stamp = rospy.Time(0)
                ps2_map = self.listener.transformPoint( '/map', ps2 )

                pts.append([ x, y, ps2_map.point.x, ps2_map.point.y ])

                inside = x > -0.5 and x < 10.5 and y > -0.5 and y < 6.5
                if not inside:
                    self.abort()

            except:
                rospy.logout( 'sim_capture: Failed transform.' )
                pass
                    
            rospy.sleep( 0.2 )
        rospy.logout( 'sim_capture: Done servoing.' )
        #pts = list( bp.pts )

        sm.stop()
        #bp.stop()

        # Stop recorder and shuttle to disk.
        if fname:
            f = open( fname, 'w' )
        else:
            f = open( 'trajectory_caps/' + str(int(time.time())) + '_cap.pkl', 'w' )
        pkl.dump( pts, f )
        f.close()
        
        rospy.logout( 'sim_capture: Capture complete' )
        return int( True )

        
        
        
if __name__ == '__main__':
    sc = SimCapture()
    rospy.spin()
