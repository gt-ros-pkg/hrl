#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_servoing')
import rospy

import hrl_rfid.ros_M5e_client as rmc
from hrl_rfid.msg import RFIDread
import robotis.ros_robotis as rr
import costmap_services.python_client as costmap
from geometry_msgs.msg import Twist
import actionlib

from rfid_servoing.srv import ServoSrv
from rfid_servoing.msg import ServoAction, ServoResult, ServoGoal


import numpy as np, math
from collections import deque
import os
from threading import Lock

def ctime():
    return rospy.Time.now().to_time()

def retfalse():
    return False

class ServoNode( ):
    def __init__( self ):
        rospy.logout('servo_node: Initializing')
        try:
            rospy.init_node('servo_node')
        except: # Node probably already initialized elsewhere
            pass

        # Create servos.
        self.p_left = rr.ROS_Robotis_Client( 'left_pan' )
        self.t_left = rr.ROS_Robotis_Client( 'left_tilt' )
        self.p_right = rr.ROS_Robotis_Client( 'right_pan' )
        self.t_right = rr.ROS_Robotis_Client( 'right_tilt' )

        # Create Costmap Services obj
        self.cs = costmap.CostmapServices( accum = 3 )
        # Note: After moving, this will require accum * -1's before stopping.

        # Publish move_base command
        self._pub = rospy.Publisher( 'rfid_cmd_vel', Twist )

        # Alterative ways to call servoing using ROS services / actionlib
        self._service_servo = rospy.Service( '/rfid_servo/servo', ServoSrv, self.service_request )
        self._as = actionlib.SimpleActionServer( '/rfid_servo/servo_act',
                                                 ServoAction, execute_cb = self.action_request )
        self._as.start()
    
        rospy.logout( 'servo_node: Service ready and waiting' )


    def service_request( self, req ):
        return self.run( req.tagid )

    def action_request( self, goal ):
        rospy.logout( 'servo_node: action_request received for tagid: \'%s\'' % goal.tagid )
        
        def preempt_func():
            # self._as should be in scope and bound @ function def. (I think for python...)
            check = self._as.is_preempt_requested()
            if check:
                rospy.logout( 'servo_node: action_request preempted!' )
            return check

        rv = self.run( goal.tagid, preempt_func = preempt_func )
        rospy.logout( 'servo_node: action_request received result %d' % int(rv) )

        if preempt_func():  # this is a relatively new addition
            rospy.logout('servo_node: returning actionlib state preempted.')
            self._as.set_preempted()
        elif rv == True:
            self._as.set_succeeded( ServoResult( int(rv) ))
            

    def run( self, tag_id, preempt_func = retfalse ):
        # tag_id => 12-character tag id
        # preempt_func => returns true when a preempt request is received, else false
        rospy.logout( 'servo_node: Run called for tagid: \'%s\'' % tag_id )

        # Ears out.
        self.robotis_init()
        
        # Startup RFID reads
        zc = ZedCalc( filt_len = 5, tag_id = tag_id ) 
                
        rospy.logout('servo_node: Running')

        zed_next = zc.next_pub()
        r = rospy.Rate( 10 )
        while not rospy.is_shutdown() and self.cs.scoreTraj_PosHyst( 0.1, 0.0, zed_next ) != -1.0 and not preempt_func():
            move_command = Twist()
            move_command.linear.x = 0.1
            move_command.angular.z = zed_next

            self._pub.publish( move_command )
            zc.update_last_pub( zed_next )
            zed_next = zc.next_pub() 

            # Don't do this too fast (avoid unwanted buffering / rate issues)
            try:
                r.sleep()
            except rospy.ROSInterruptException: # rospy shutdown request received
                pass

        self._pub.publish( Twist() ) # Stop moving.

        # Tuck ears
        # self.robotis_end()

        zc.stop() # Stop the RFID reader...

        if preempt_func():
            rospy.logout( 'servo_node: Preempt was requested. May not have finished.' )
            rospy.logout( 'servo_node: Exiting' )
            return False
        else:
            rospy.logout( 'servo_node: Exiting' )
            return True
        

    def robotis_init( self ):
        # Move Antennas into Servo mode
        self.t_left.move_angle( 0.0, math.radians(30), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(30), blocking = False )
        self.p_left.move_angle( math.radians(40), math.radians(30), blocking = False )
        self.p_right.move_angle( math.radians(-40), math.radians(30), blocking = True )

        while self.p_left.is_moving() or self.p_right.is_moving():
            rospy.sleep( 0.05 )

    def robotis_end( self ):
        # Return servos to home position
        self.t_left.move_angle( 0.0, math.radians(10), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(10), blocking = False )
        self.p_left.move_angle( 1.350, math.radians(10), blocking = False )
        if not (os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim'):
            self.p_right.move_angle( -1.350, math.radians(10), blocking = False )
        else:
            self.p_right.move_angle( -1.350, math.radians(10), blocking = True )



class ZedCalc( ):
    ### WARNING: Do not instantiate in init's.  This object queries the RFID reader!
    def __init__( self, tag_id, sub_name = '/rfid/ears_reader', filt_len = 5 ):
        rospy.logout( 'ZedCalc: Initializing' )
        # We will process the reads ourself
        self.values = { 'EleLeftEar': 75.0,
                        'EleRightEar': 75.0 }

        self.lock = Lock()
        self.buff_len = filt_len * 2 # two antennas' worth
        self.zed_buff = deque( np.zeros( self.buff_len ))  # Angle filter length

        self.last_zed = 0.0 # last published command
        self.new_zed = 0.0  # projected new command

        self._sub = rospy.Subscriber( sub_name, RFIDread, self.callback)
        self.reader = rmc.ROS_M5e_Client('ears')
        self.reader.track_mode( tag_id )

        rospy.logout( 'ZedCalc: Ready.' )

    def stop( self ):
        rospy.logout( 'ZedCalc: Stopping RFID reader' )
        self.reader.stop() # Stop RFID reads


    def callback( self, msg ):
        self.values[ msg.antenna_name ] = np.clip( msg.rssi, 75.0, 110.0 )
        left = self.values[ 'EleLeftEar' ]
        right = self.values[ 'EleRightEar' ]

        self.lock.acquire() # lock on buff 
        self.zed_buff.append( 0.10 * (left - right)/5.0 )
        if len( self.zed_buff ) > self.buff_len:
            self.zed_buff.popleft()
        self.lock.release()

    def next_pub( self, angle_slew = math.radians( 10 )):
        self.lock.acquire() # lock on buff
        target = np.mean( self.zed_buff )
        self.lock.release()
        
        lz = self.last_zed
        return lz + np.clip( target - lz, -1.0 * angle_slew, angle_slew )

    def update_last_pub( self, pub_val ):
        self.last_zed = pub_val
        


if __name__ == '__main__':

    sn = ServoNode()
    rospy.spin()
    
    # There are (at least) three ways to call this code.
    # 1) Python:
    
    # print sn.run( 'person      ' )
    # rospy.spin()

    # 2) ROS service:

    # rosservice call /rfid_servo/servo "'person      '"

    # 3) ROS actionlib:

    # try:
    #     client = actionlib.SimpleActionClient( '/rfid_servo/servo_act', ServoAction )
    #     client.wait_for_server()
    #     client.send_goal( ServoGoal( 'person      ' ))
    #     client.wait_for_result()
    #     print client.get_result()
    # except rospy.ROSInterruptException:
    #     print 'Program interrupted before completion'

    # rospy.spin()

    

