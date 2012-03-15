#! /usr/bin/python

import roslib; 
roslib.load_manifest('hrl_pr2_kinematics_tutorials')
roslib.load_manifest('rfid_people_following')
roslib.load_manifest('std_srvs')
roslib.load_manifest('joy')
import rospy

from rfid_people_following.srv import StringInt32_Int32
from rfid_people_following.srv import String_Int32
from rfid_people_following.srv import Int32_Int32
from rfid_people_following.srv import String_StringArr
from rfid_people_following.srv import rfid_gui as gui_srv
from rfid_people_following.srv import HandoffSrv
from std_srvs.srv import Empty
from joy.msg import Joy
from cmd_process import CmdProcess

import numpy as np, math
import time

class DemoNode():
    def __init__( self ):
        rospy.init_node( 'rfid_demo', anonymous = True )
        rospy.logout( 'demo_node: Initializing' )

        rospy.logout( 'demo_node: Waiting for services' )
        rospy.wait_for_service( '/rfid_servo/servo' )
        rospy.wait_for_service( '/rfid_orient/orient' )
        rospy.wait_for_service( '/rfid_orient/flap' )
        rospy.wait_for_service( '/rfid_handoff/initialize' )
        rospy.wait_for_service( '/rfid_handoff/handoff' )
        rospy.wait_for_service( '/rfid_handoff/wave' )
        #rospy.wait_for_service( '/rfid_gui/select' )
        rospy.logout( 'demo_node: All services ready.' )

        self.last_button = time.time()
        self.run_demo = False
        self.run_hoinit = False
        self.run_handoff = False

        self._joy_sub = rospy.Subscriber( 'joy', Joy, self.joy_process )

        self._servo = rospy.ServiceProxy( '/rfid_servo/servo', StringInt32_Int32 )
        self.follow1 = lambda : self._servo( 'person      ', 1 ) # Stops at first obs
        self.follow = lambda : self._servo( 'person      ', 0 ) # Runs forever

        self._servo_stop = rospy.ServiceProxy( '/rfid_servo/stop_next_obs', Int32_Int32 )
        self.servo_toggle = lambda : self._servo_stop( 1 ) 

        self._handoff = rospy.ServiceProxy( '/rfid_handoff/handoff', HandoffSrv )
        self.handoff = lambda : self._handoff()

        self._hoinit = rospy.ServiceProxy( '/rfid_handoff/initialize', HandoffSrv )
        self.hoinit = lambda : self._hoinit()

        self._howave = rospy.ServiceProxy( '/rfid_handoff/wave', HandoffSrv )
        self.howave = lambda : self._howave()

        self._flap = rospy.ServiceProxy( '/rfid_orient/flap', String_StringArr )
        self.flap = lambda : self._flap( '' )

        self._orient = rospy.ServiceProxy( '/rfid_orient/orient', String_Int32 )
        self.orient = lambda tagid: self._orient( tagid )

        #self._gui = rospy.ServiceProxy( '/rfid_gui/select', gui_srv )
        #self.gui = lambda tags: self._gui( tags )

        self._service = rospy.Service( '/rfid_demo/demo', Empty, self.demo )
        self._service = rospy.Service( '/rfid_demo/gui', Empty, self.gui_test )


        rospy.logout( 'demo_node: Demo service ready!' )

    def joy_process( self, msg ):
        if msg.buttons[11] == 1 and time.time() - self.last_button > 1.0:
            if msg.buttons[12] == 1: # tri + R1
                rospy.logout( 'demo_node: joy_process: Calling demo service' )
                p = CmdProcess( ['rosservice', 'call', '/rfid_demo/demo'] )
                p.run()
                self.last_button = time.time()
            elif msg.buttons[13] == 1: # circle + R1
                rospy.logout( 'demo_node: joy_process: Calling handoff service' )
                p = CmdProcess( ['rosservice', 'call', '/rfid_handoff/handoff'] )
                p.run()
                self.last_button = time.time()
            elif msg.buttons[14] == 1: # X + R1
                rospy.logout( 'demo_node: joy_process: Calling servo toggle service' )
                p = CmdProcess( ['rosservice', 'call', '/rfid_servo/stop_next_obs', '1'] )
                p.run()
                self.last_button = time.time()
            elif msg.buttons[15] == 1: # square + R1
                rospy.logout( 'demo_node: joy_process: Calling handoff initialization service' )
                p = CmdProcess( ['rosservice', 'call', '/rfid_handoff/initialize'] )
                p.run()
                self.last_button = time.time()
            elif msg.buttons[9] == 1: # R2 + R1
                rospy.logout( 'demo_node: joy_process: Calling handoff wave service' )
                p = CmdProcess( ['rosservice', 'call', '/rfid_handoff/wave'] )
                p.run()
                self.last_button = time.time()

    def gui_test( self, msg = None ):
        tags = self.flap().value
        gui_resp = self.gui( tags )

    def demo( self, msg = None ):
        tags = self.flap().value
        #gui_resp = self.gui( tags )

        #self.orient( gui_resp.selection )
        self.orient( 'person      ' )
        
        #self.follow1()
        self.follow()
        self.handoff()

if __name__ == '__main__':
    dn = DemoNode()
    rospy.spin()
