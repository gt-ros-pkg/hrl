#! /usr/bin/python

import roslib; 
roslib.load_manifest('rfid_people_following')
roslib.load_manifest('std_srvs')
roslib.load_manifest('explore_hrl')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('hrl_rfid')
import rospy

import tf
import tf.transformations as tft
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import hrl_rfid.ros_M5e_client as rmc

from rfid_people_following.srv import StringInt32_Int32
from rfid_people_following.srv import String_Int32
from rfid_people_following.srv import Int32_Int32
from rfid_people_following.srv import String_StringArr
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped
import actionlib
import explore_hrl.msg

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

import numpy as np, math
import time
from threading import Thread
import os


# A more general form of this should be folded back into ros_M5e_client!
# It also appears in new_servo_node (rfid_people_following)
class rfid_poller( Thread ):
    def __init__( self, tagid ):
        Thread.__init__( self )
        self.reader = rmc.ROS_M5e_Client('ears')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/ear_antenna_left', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/ear_antenna_right', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )

        self.should_run = True
        self.should_poll = False
        self.data = []
        self.tagid = tagid
        self.start()

    def transform( self, antname ):
        ps = PointStamped()
        ps2 = PointStamped()
        ps2.point.x = 0.1

        if antname == 'EleLeftEar':
            ps.header.frame_id = '/ear_antenna_left'
            ps.header.stamp = rospy.Time( 0 )

            ps2.header.frame_id = '/ear_antenna_left'
            ps2.header.stamp = rospy.Time( 0 )
        elif antname == 'EleRightEar':
            ps.header.frame_id = '/ear_antenna_right'
            ps.header.stamp = rospy.Time( 0 )

            ps2.header.frame_id = '/ear_antenna_right'
            ps2.header.stamp = rospy.Time( 0 )
        else:
            rospy.logout( 'Bad ear' )
            return False, 0.0, 0.0, 0.0

        try:
            ps_map = self.listener.transformPoint( '/map', ps )
            x = ps_map.point.x
            y = ps_map.point.y
            #rospy.logout( 'Done 1' )

            ps2_map = self.listener.transformPoint( '/map', ps2 )
            x2 = ps2_map.point.x
            y2 = ps2_map.point.y
            #rospy.logout( 'Done 1' )

            # We'll pass back the position of the base (which will be a "safe" place to traverse, whereas the antenna pos not necessarily in the clear)
            ps_base = PointStamped()
            ps_base.header.frame_id = '/base_link'
            ps_base.header.stamp = rospy.Time( 0 )
            ps_base_map = self.listener.transformPoint( '/map', ps_base )
            x_base = ps_base_map.point.x
            y_base = ps_base_map.point.y
            
            #rospy.logout( 'Transform Success ' + ps.header.frame_id )
            return True, x_base, y_base, np.arctan2( y2 - y, x2 - x )
        except:
            rospy.logout( 'Transform failed! ' + ps.header.frame_id )
            return False, 0.0, 0.0, 0.0
            

    def start_poller( self ):
        # Start appending into self.data
        #self.reader.track_mode( self.tagid )
        self.should_poll = True

    def stop_poller( self ):
        # Stop appending into self.data
        #self.reader.stop()
        self.should_poll = False

    def run( self ):
        rospy.logout( 'rfid_poller: Starting' )
        while self.should_run and not rospy.is_shutdown():
            if self.should_poll:
                left = self.reader.read('EleLeftEar')[-1]
                success, x, y, ang = self.transform( 'EleLeftEar' )
                if success:
                    self.data.append( [left, [x,y,ang]] )

                right = self.reader.read('EleRightEar')[-1]
                success, x, y, ang = self.transform( 'EleRightEar' )
                if success:
                    self.data.append( [right, [x,y,ang]] )
            else:
                rospy.sleep( 0.050 )
            
        try:  # Shut it down to conserve power.  Something of a race condition (exception)
            self.reader.stop()
        except:
            pass
        rospy.logout( 'rfid_poller: Exiting' )
        
    def stop( self ):
        # Kill off the poller thread.
        self.should_run = False
        self.join(5)
        # if (self.isAlive()):
        #     raise RuntimeError("rfid_poller: Unable to stop thread")


# A more general form of this should be folded back into orient_node (rfid_people_following)!
class Flapper( Thread ):
    def __init__( self, tagid = 'person      '):
        Thread.__init__( self )
        rospy.logout('Flapper: Initializing' )
        rospy.wait_for_service( '/rfid_orient/flap' )
        rospy.logout('Flapper: flap service ready.' )

        self._flap = rospy.ServiceProxy( '/rfid_orient/flap', String_StringArr )
        self.flap = lambda : self._flap( tagid )

        self.should_run = True
        self.should_flap = False
        self.start()

    def start_flapper( self ):
        # Start appending into self.data
        #self.reader.track_mode( self.tagid )
        self.should_flap = True

    def stop_flapper( self ):
        # Stop appending into self.data
        #self.reader.stop()
        self.should_flap = False

    def run( self ):
        rospy.logout( 'Flapper: Starting' )
        r = rospy.Rate( 10 )
        while self.should_run and not rospy.is_shutdown():
            if self.should_flap:
                self.flap()
            else:
                r.sleep()
            
        rospy.logout( 'Flapper: Exiting' )
        
    def stop( self ):
        # Kill off the poller thread.
        self.should_run = False
        self.join(15)
        if (self.isAlive()):
            raise RuntimeError("Flapper: Unable to stop thread")


def navstack( x, y, ang ):
    try:
        rospy.logout( 'Requesting navstack move to <x,y,ang-deg> %3.3f %3.3f %3.3f.' % (x, y, math.degrees(ang)) )

        client = actionlib.SimpleActionClient( 'move_base', MoveBaseAction )
        client.wait_for_server()

        ps = PoseStamped()
        ps.header.frame_id = '/map'
        ps.header.stamp = rospy.Time(0)
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, ang ))

        goal = MoveBaseGoal( ps )
        client.send_goal( goal )
        rospy.logout( 'Waiting for base to stop moving.' )
        client.wait_for_result()
        rospy.logout( 'Successfully navigated to desired position.' )
        return True
    except:
        rospy.logout( 'Navstack did not achieve desired position.' )
        return False


class RFIDSearch():
    def __init__( self ):
        try:
            rospy.init_node( 'rfid_search' )
        except:
            pass
        
        rospy.logout( 'rfid_search: Initializing.' )
        #rospy.wait_for_service( '/rfid_servo/servo' )
        rospy.wait_for_service( '/rfid_orient/orient' )
        rospy.wait_for_service( '/rfid_orient/flap' )
        #rospy.wait_for_service( '/rfid_demo/demo' )
        #rospy.wait_for_service( '/rfid_gui/select' )
        self.explore_act = actionlib.SimpleActionClient('explore', explore_hrl.msg.ExploreAction)
        rospy.logout( 'rfid_search: Waiting for explore.' )
        self.explore_act.wait_for_server()
        rospy.logout( 'rfid_search: Done Initializing.' )

        #self._servo = rospy.ServiceProxy( '/rfid_servo/servo', StringInt32_Int32 )
        #self.follow1 = lambda : self._servo( 'person      ', 1 ) # Stops at first obs
        #self.follow = lambda : self._servo( 'person      ', 0 ) # Runs forever

        #self._demo = rospy.ServiceProxy( '/rfid_demo/demo', Empty )
        #self.demo = lambda : self._demo() 

        #self._servo_stop = rospy.ServiceProxy( '/rfid_servo/stop_next_obs', Int32_Int32 )
        #self.servo_toggle = lambda : self._servo_stop( 1 ) 

        self._orient = rospy.ServiceProxy( '/rfid_orient/orient', String_Int32 )
        self.orient = lambda tagid: self._orient( tagid )

        self.rp = rfid_poller('person      ')
        self.flapper = Flapper()

        rospy.logout( 'rfid_search: ready to go!' )

    def wait_for_finish( self, radius = 2.0 ):
        print 'Starting RFID tag scanning'
        self.rp.start_poller()
        self.flapper.start_flapper()
        rospy.sleep( 0.3 )

        print 'Starting Search'
        goal = explore_hrl.msg.ExploreGoal( radius = radius )
        self.explore_act.send_goal(goal)
        rospy.sleep( 0.5 )
        self.explore_act.wait_for_result()
        res = self.explore_act.get_result()
        print 'Search Complete: ', res
        status = self.explore_act.get_state()

        print 'Disabling RFID scanning'
        self.flapper.stop_flapper()
        self.rp.stop_poller()

        print 'Computing Best Position'
        readings =  self.rp.data
        print readings
        rr = list( self.rp.data ) # rfid_reads: [ [rssi,[x,y,ang]], ...]
        rssi = [ r for r,vec in rr ]
        max_rssi, max_pose = rr[ np.argmax( rssi ) ]

        print 'Moving to best Position: ', max_pose, ' RSSI: ', max_rssi
        navstack( *max_pose )

        print 'Executing Remainder of Demo'
        # if (os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim'):
        #     self.follow1() # Only do servoing in simulation
        # else:
        #     try:
        #         self.demo() # Run the full demo IRL
        #     except: # for some reason, NoneType throws exception...
        #         pass

        print 'Shutting down threads'
        self.rp.stop()
        self.flapper.stop()

        if status == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'aborted'

if __name__ == '__main__':
    rs = RFIDSearch()
    time.sleep( 3 )
    rs.wait_for_finish( radius = 1.7 )
    #print rs.flap()

#     while True:
#         print 'DoneSearching: ', rs.doneSearching()
