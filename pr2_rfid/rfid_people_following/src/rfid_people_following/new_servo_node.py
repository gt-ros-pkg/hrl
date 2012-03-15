#! /usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')
roslib.load_manifest('rfid_people_following')

import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rfid_people_following.srv import StringInt32_Int32
from rfid_people_following.srv import Int32_Int32
from rfid_people_following.srv import StringInt32_Int32Response
from std_srvs.srv import Empty

import numpy as np, math
import os

import hrl_rfid.ros_M5e_client as rmc
import robotis.ros_robotis as rr
    
from threading import Thread
from collections import deque

SCORE_BUFFER_LEN = 5
SCORE_BUFFER_MOVE = 3  # number of checked buffer elements that must not be -1.
ANGLE_FILTER_LEN = 5
ANGLE_SLEW = math.radians( 10 )

def ctime():
    return rospy.Time.now().to_time()

class ServoNode( ):
    def __init__( self ):
        rospy.logout('servo_node: Initializing')
        try:
            rospy.init_node('servo_node')
        except:
            pass

        # Unfold "ears" to servo positions
        self.p_left = rr.ROS_Robotis_Client( 'left_pan' )
        self.t_left = rr.ROS_Robotis_Client( 'left_tilt' )
        self.p_right = rr.ROS_Robotis_Client( 'right_pan' )
        self.t_right = rr.ROS_Robotis_Client( 'right_tilt' )
    

        # Use assisted teleop as pseudo-callback to see if valid movement:
        self.scores = deque()
        self.og_pub = rospy.Publisher( 'assisted_teleop_check', Twist )
        self.og_sub = rospy.Subscriber( 'assisted_teleop_score', Float64, self.og_score_cb )
        self.command_pub = rospy.Publisher( 'rfid_cmd_vel', Twist )
        self._service_servo = rospy.Service('/rfid_servo/servo', 
                                            StringInt32_Int32,
                                            self.run )
        self._service_stop = rospy.Service('/rfid_servo/stop', Empty, self.servo_stop )
        self._service_stop = rospy.Service('/rfid_servo/abort', Empty, self.servo_abort )
        self._service_stop_next_obs = rospy.Service('/rfid_servo/stop_next_obs', Int32_Int32, self.stop_request )
        self.stop_next_obs = False # run forever
        self.should_run = True
        self.abort = False
        rospy.logout( 'servo_node: Service ready and waiting' )

    def stop_request( self, msg = None ):
        #self.stop_next_obs = bool( msg.data )
        self.stop_next_obs = not self.stop_next_obs
        rospy.logout( 'servo_node: Stop after next obstacle flag set to %s' % str( self.stop_next_obs ))
        return True

    def servo_stop( self, msg = None ):
        # Note... should now be called "toggle"
        self.should_run = not self.should_run
        rospy.logout( 'servo_node: Toggled should_run to %s' % str( self.should_run ))

    def servo_abort( self, msg = None ):
        # Note... should now be called "toggle"
        self.abort = True
        rospy.logout( 'servo_node: Aborting!' )

    def og_score_cb( self, msg ):
        self.scores.append( msg.data )
        if len( self.scores ) > SCORE_BUFFER_LEN:
            self.scores.popleft()

    def run( self, request ):
        rospy.logout( 'servo_node: Called for tagid: \'%s\', continous: %d' % (request.data, request.cont) )

        # Move Antennas into Servo mode
        self.t_left.move_angle( 0.0, math.radians(30), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(30), blocking = False )
        self.p_left.move_angle( math.radians(-40), math.radians(30), blocking = False )
        self.p_right.move_angle( math.radians(40), math.radians(30), blocking = True )

        while self.p_left.is_moving() or self.p_right.is_moving():
            #time.sleep( 0.01 ) # Not such a good idea when doing simulation.
            rospy.sleep( 0.05 )

        self.stop_next_obs = bool( request.cont )
        self.should_run = True
        self.abort = False

        # Startup RFID reads
        self.rp = rfid_poller( request.data )
                
        rospy.logout('servo_node: Running')
        rate = rospy.Rate( 15.0 )
        zed = deque([ 0.0 ])
        last_zed = 0.0
        #self.scores = deque()
        last_print = ctime()

        while self.should_run and not rospy.is_shutdown():
            rate.sleep()
            left, right = self.rp.values

            #zed.append( 0.07 * (left - right)/5.0 )
            zed.append( 0.10 * (left - right)/5.0 )
            if len( zed ) > ANGLE_FILTER_LEN:
                zed.popleft()

            target_zed = np.mean( zed )
            new_zed = last_zed + np.clip( target_zed - last_zed, -1.0 * ANGLE_SLEW, ANGLE_SLEW )
            last_zed = new_zed

            check = Twist()
            check.linear.x = 0.1
            check.angular.z = new_zed
            self.og_pub.publish( check )

            move_command = Twist()

            if self.abort:
                self.should_run = False
            elif len( self.scores ) < SCORE_BUFFER_LEN:
                rospy.logout( 'servo_node: Score buffer filling: %d of %d' % (len(self.scores), SCORE_BUFFER_LEN)  )
            elif sum([ i != -1 for i in self.scores]) < SCORE_BUFFER_MOVE:
                # Check for tag detected!
                if ctime() - last_print > 1.0:
                    rospy.logout( 'servo_node: Obstacle detected' )
                    last_print = ctime()
                if self.stop_next_obs:  # Stop after next obstacle detected
                    self.should_run = False
            elif len( zed ) < ANGLE_FILTER_LEN:
                rospy.logout( 'servo_node: Angle buffer filling' )
            else:
                move_command.linear.x = check.linear.x
                move_command.angular.z = check.angular.z

            self.command_pub.publish( move_command )

        # When servo node shuts down...
        self.command_pub.publish( Twist() )  # Halt the base
        self.rp.stop() # Stop the reader

        # Return servos to home position
        self.t_left.move_angle( 0.0, math.radians(10), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(10), blocking = False )
        self.p_left.move_angle( -1.350, math.radians(10), blocking = False )
        if not (os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim'):
            self.p_right.move_angle( 1.350, math.radians(10), blocking = False )
        else:
            self.p_right.move_angle( 1.350, math.radians(10), blocking = True )

        rospy.logout('servo_node: Returning')
        return StringInt32_Int32Response( int( True ))


class ServoNodeClient():
    def __init__( self, service_name = '/rfid_servo/servo' ):
        rospy.logout( 'servo_node_client: Waiting for service: \'%s\'' % service_name )
        rospy.wait_for_service( service_name )
        rospy.logout( 'servo_node_client: Service ready.' )
        
        self._servo_service = rospy.ServiceProxy( service_name, StringInt32_Int32 )

    def servo( self, tagid, continuous_operation = True ):
        return self._servo_service( tagid, int( continuous_operation ))


class rfid_poller( Thread ):
    def __init__( self, tagid ):
        Thread.__init__( self )
        self.reader = rmc.ROS_M5e_Client('ears')
        self.reader.track_mode( tagid )
        self.should_run = True
        self.values = [75.0, 75.0]
        self.start()

    def stop( self ):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("rfid_poller: Unable to stop thread")

    def run( self ):
        rospy.logout( 'rfid_poller: Starting' )
        while self.should_run and not rospy.is_shutdown():
            left = np.clip( self.reader.read('EleLeftEar')[-1], 75.0, 110.0 )
            right = np.clip( self.reader.read('EleRightEar')[-1], 75.0, 110.0 )
            self.values = [left, right]
        try:  # Shut it down to conserve power.  Something of a race condition (exception)
            self.reader.stop()
        except:
            pass
        rospy.logout( 'rfid_poller: Exiting' )
        

        



if __name__ == '__main__':
    sn = ServoNode()
    rospy.spin()

