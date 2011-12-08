#! /usr/bin/python

import roslib;
roslib.load_manifest('rfid_demos')
roslib.load_manifest('hrl_pr2_kinematics_tutorials')
# roslib.load_manifest('rfid_people_following')
roslib.load_manifest('std_srvs')
roslib.load_manifest('joy')
import rospy

from smach_ros import IntrospectionServer
from rfid_servoing.srv import ServoSrv
from rfid_behaviors.srv import HandoffSrv
from rfid_behaviors.srv import String_Int32
from rfid_behaviors.srv import FlapEarsSrv
# from rfid_people_following.srv import rfid_gui as gui_srv
from rfid_demos import sm_rfid_delivery
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

        rospy.logout( 'demo_node: Setting up state machine.' )
        self.sm = sm_rfid_delivery.sm_delivery()

        sis = IntrospectionServer('RFID_delivery', self.sm, '/SM_ROOT')
        sis.start()

        rospy.logout( 'demo_node: Done setting up state machine.' )
        
        self.last_button = time.time()
        self.run_demo = False
        self.run_hoinit = False
        self.run_handoff = False

        self._joy_sub = rospy.Subscriber( 'joy', Joy, self.joy_process )

        self._servo = rospy.ServiceProxy( '/rfid_servo/servo', ServoSrv )
        self.follow1 = lambda : self._servo( 'person      ', 1 ) # Stops at first obs
        self.follow = lambda : self._servo( 'person      ', 0 ) # Runs forever

        self._handoff = rospy.ServiceProxy( '/rfid_handoff/handoff', HandoffSrv )
        self.handoff = lambda : self._handoff()

        self._hoinit = rospy.ServiceProxy( '/rfid_handoff/initialize', HandoffSrv )
        self.hoinit = lambda : self._hoinit()

        self._howave = rospy.ServiceProxy( '/rfid_handoff/wave', HandoffSrv )
        self.howave = lambda : self._howave()

        self._flap = rospy.ServiceProxy( '/rfid_orient/flap', FlapEarsSrv )
        self.flap = lambda : self._flap( '' )

        self._orient = rospy.ServiceProxy( '/rfid_orient/orient', String_Int32 )
        self.orient = lambda tagid: self._orient( tagid )

        #self._gui = rospy.ServiceProxy( '/rfid_gui/select', gui_srv )
        #self.gui = lambda tags: self._gui( tags )

        self._service = rospy.Service( '/rfid_demo/demo', Empty, self.demo )
        # self._service = rospy.Service( '/rfid_demo/gui', Empty, self.gui_test )


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
            # SERVO_TOGGLE NO LONGER DEFINED!
            # elif msg.buttons[14] == 1: # X + R1  
            #     rospy.logout( 'demo_node: joy_process: Calling servo toggle service' )
            #     p = CmdProcess( ['rosservice', 'call', '/rfid_servo/stop_next_obs', '1'] )
            #     p.run()
            #     self.last_button = time.time()
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

    # def gui_test( self, msg = None ):
    #     tags = self.flap().value
    #     gui_resp = self.gui( tags )

    def demo( self, msg = None ):
        rospy.logout( 'demo_node: Calling.' )
        self.sm.userdata.tagid = 'person      '
        outcome = self.sm.execute()
        print outcome

if __name__ == '__main__':
    dn = DemoNode()
    rospy.spin()
