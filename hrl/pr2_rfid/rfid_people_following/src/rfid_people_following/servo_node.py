#! /usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')

import time
import rospy
from geometry_msgs.msg import Twist
import numpy as np, math
import hrl_rfid.ros_M5e_client as rmc
import robotis.ros_robotis as rr
from threading import Thread

class ServoNode( Thread ):
    def __init__( self ):
        Thread.__init__( self )

        self.should_run = True

        rospy.logout('servo_node: Initializing')
        rospy.init_node('servo_node')

        # Unfold "ears" to servo positions
        p_left = rr.ROS_Robotis_Client( 'left_pan' )
        t_left = rr.ROS_Robotis_Client( 'left_tilt' )

        p_left.move_angle( math.radians(-40), math.radians(10), blocking = False )
        t_left.move_angle( 0.0, math.radians(10), blocking = False )

        p_right = rr.ROS_Robotis_Client( 'right_pan' )
        t_right = rr.ROS_Robotis_Client( 'right_tilt' )
    
        p_right.move_angle( math.radians(40), math.radians(10), blocking = False )
        t_right.move_angle( 0.0, math.radians(10), blocking = False )

        self.reader = rmc.ROS_M5e_Client('ears')
        self.reader.track_mode('person      ')

        # Use assisted teleop as pseudo-callback to see if valid movement:
        self.check_pub = rospy.Publisher( 'rfid_cmd_vel_check', Twist )
        self.sub = rospy.Subscriber( "assisted_teleop_response", Twist, self.callback )
        self.command_pub = rospy.Publisher( "rfid_cmd_vel", Twist )

        self.moving = True
        time.sleep(3.0) # give antennas time to settle

        self.start()

    def callback( self, msg ):
        #print 'Hit callback: ', msg
        self.response_received = True
        self.response = msg

    def sloppy_check( self, desired_movement, timeout = 5.0 ):
        #print 'Checking: ', desired_movement
        # Simple statemachine with self.callback.  Checks to see if
        # assisted teleop returns an unaltered command
        self.response_received = False
        self.response = Twist()

        self.check_pub.publish( desired_movement )
        t0 = time.time()

        # wait for the callback to be received
        while not self.response_received and time.time() - t0 < timeout and self.should_run and not rospy.is_shutdown():
            time.sleep(0.01)

        # Apply some hysteresis to starting & stopping conditions
        if self.moving:
            if self.response.linear.x > 0.04:
                self.moving = True  # As long as safe move is within 40%, keep moving
                return self.response, True  
            else:
                self.moving = False
                return Twist(), False # Otherwise stop
        else:
            if self.response.linear.x > 0.075: 
                self.moving = True  # Wait until it is within 75% to resume movement
                return self.response, True  
            else:
                self.moving = False
                return Twist(), False # Otherwise remain stop
        
                
            

    def run( self ):
        rospy.logout('servo_node: Running')
        filt_x = []
        filt_z = []

        while self.should_run and not rospy.is_shutdown():
            left = self.reader.read('EleLeftEar')[-1]
            if left == -1 or left < 75:
                left = 75
            right = self.reader.read('EleRightEar')[-1]
            if right == -1 or right < 75:
                right = 75

            check = Twist()
            check.linear.x = 0.1
            check.angular.z = 0.07 * (left - right)/5.0 

            safe_move, continue_moving = self.sloppy_check( check )

            if continue_moving:
                filt_z.append( check.angular.z )
                filt_x.append( check.linear.x )
            else:
                filt_z.append( 0.0 )
                filt_x.append( 0.0 )
                
            if len( filt_z ) < 5 or len( filt_x ) < 5: # let the 5-point averager fill up.
                continue
            filt_z = filt_z[1:]
            filt_x = filt_x[1:]

            filt_movement = Twist()
            filt_movement.linear.x = np.average( filt_x )
            filt_movement.angular.z = np.average( filt_z )

            self.command_pub.publish( filt_movement )
            time.sleep(0.01)

        # Stop the RFID reader (conserve power and hardware)
        try:
            r.stop()
        except: 
            # If servo_node is being run in the same launch file, stop is a
            #    race condition with service disappearing.  Just ignore.
            pass
        
        self.command_pub.publish( Twist() )  # Halt the base (just in case)
        rospy.logout('servo_node: Exiting')


    def stop( self ):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("servo_node: unable to stop thread")


if __name__ == '__main__':
    sn = ServoNode()
    rospy.spin()
    sn.stop()

