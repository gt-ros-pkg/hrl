#! /usr/bin/python

import roslib; 
roslib.load_manifest('hrl_pr2_kinematics_tutorials')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('std_srvs')
import rospy

import hrl_pr2_kinematics_tutorials.hrl_pr2 as hrl_pr2
import hrl_lib.transforms as tr
import rfid_behaviors.tactile_sensors as tsen
from rfid_behaviors.srv import ArmSrv
from rfid_behaviors.srv import HandoffSrv

import numpy as np, math
import os
import time

class HandoffNode():
    def __init__( self ):
        rospy.init_node( 'handoff', anonymous = True )
        rospy.logout( 'handoff_node: Have run hrl_pr2_gains/change_gains.sh yet?' )

        self.robot = hrl_pr2.HRL_PR2()
        if not (os.environ.has_key('ROBOT') and os.environ['ROBOT'] == 'sim'):
            self.ts = tsen.TactileSensor()
        self.arm = 'right_arm'

        self.start_ja = [0.040304940763152608, 1.2398003444166741, -1.2204088251845415, -1.9324078526157087, -31.197472992401149, -1.7430222641585842, -1.5358378047038517]
        #self.target_ja = [0.35891507126604916, 0.13778228113494312, -0.01277662779292843, -1.4992538841561938, -28.605807802842136, -0.96590944225972863, -3.0950669743130161]
        self.target_ja = [0.0818, 0.377, -0.860, -2.144, -3.975, -1.479, 3.907]

        self.grasp_ja = [ -1.57263428749, -0.347376409246, -1.58724516843, -1.61707941489, -51.4022142048, -1.36894875484, -5.9965378332 ]
        self.stowgrasp_ja = [-0.130, 1.18, -1.410, -1.638, -141.06, -1.695, 48.616 ]


        self._sh = rospy.Service( '/rfid_handoff/handoff' , HandoffSrv, self.handoff )
        self._si = rospy.Service( '/rfid_handoff/initialize' , HandoffSrv, self.initialize )
        self._sj = rospy.Service( '/rfid_handoff/handoff_pos' , ArmSrv, self.handoff_pos )
        self._ss = rospy.Service( '/rfid_handoff/stow' , HandoffSrv, self.stow )
        self._sp = rospy.Service( '/rfid_handoff/pre_stow' , HandoffSrv, self.pre_stow )
        self._sg = rospy.Service( '/rfid_handoff/grasp' , HandoffSrv, self.grasp )
        self._senough = rospy.Service( '/rfid_handoff/stow_grasp' , HandoffSrv, self.stow_grasp )
        self._swave = rospy.Service( '/rfid_handoff/wave' , HandoffSrv, self.wave )

        # self.initialize()  # Prefer to do this manually... (rosservice call /rfid_handoff/initialize)
        rospy.logout( 'handoff_node: Waiting for service calls.' )

    def initialize( self, msg = None ):
        rospy.logout( 'handoff_node: Initializing. Hand me an object!' )

        # Put into handoff position, ready to accept object
        self.robot.set_jointangles( self.arm, self.target_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        self.robot.open_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))

        self.stow()
        return True

    def wave( self, msg = None ):
        wave_a = [0.0131, 0.325, -0.832, -1.762,-6.511, -0.191, 0.162]
        wave_b = [-0.180, 0.034, 0.108, -1.295, -6.224, -0.383, 0.119]
        self.robot.set_jointangles( self.arm, wave_a, 2.0 )
        rospy.sleep( rospy.Duration( 2.0 ))
        self.robot.set_jointangles( self.arm, wave_b, 1.0 )
        rospy.sleep( rospy.Duration( 1.0 ))
        self.robot.set_jointangles( self.arm, wave_a, 1.0 )
        rospy.sleep( rospy.Duration( 1.0 ))
        self.robot.set_jointangles( self.arm, wave_b, 1.0 )
        rospy.sleep( rospy.Duration( 1.0 ))
        self.robot.set_jointangles( self.arm, wave_a, 1.0 )
        rospy.sleep( rospy.Duration( 1.0 ))
        self.robot.set_jointangles( self.arm, self.start_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True
                


    def stow( self, msg=None ):
        # Grab object
        self.robot.close_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.5 ))

        # Stow
        self.robot.set_jointangles( self.arm, self.start_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True

    def pre_stow( self, msg=None ):
        # Grab object
        self.robot.close_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))

        # Stow
        self.robot.set_jointangles( self.arm, self.target_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True

    def grasp( self, msg=None ):
        # Grab object
        self.robot.close_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))

        # Stow
        self.robot.set_jointangles( self.arm, self.grasp_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True

    def stow_grasp( self, msg=None ):
        # Stow
        self.robot.set_jointangles( self.arm, self.stowgrasp_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True


    def open( self ):
        self.robot.open_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))


    def close( self ):
        self.robot.close_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))


    def handoff( self, msg = None ):
        # Put into handoff position.
        self.robot.set_jointangles( self.arm, self.target_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))

        # Tactile Sensor detector
        rospy.sleep( rospy.Duration( 0.5 ))
        self.ts.thresh_detect( 3000 )

        # Release object
        self.robot.open_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))
        
        # Stow
        self.robot.set_jointangles( self.arm, self.start_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))
        return True

    def handoff_pos( self, msg ):
        #pos = np.matrix([0.6977, -0.03622, 0.2015]).T
        #ang = tr.Rx(math.radians(0.))
        print msg
        pos = np.matrix([ msg.x, msg.y, msg.z ]).T
        ang = tr.Rx( msg.ang )
        
        q = [0, 0, 0, 0, 0, 0, 0]
        j = self.robot.IK('right_arm', pos, ang, self.target_ja)
        #j = self.robot.IK('right_arm', pos, ang, q)
        self.robot.set_jointangles( 'right_arm', j, 3.0 )

        # Tactile Sensor detector
        rospy.sleep( rospy.Duration( 0.5 ))
        self.ts.thresh_detect( 3000 )

        # Release object
        self.robot.open_gripper( self.arm )
        rospy.sleep( rospy.Duration( 2.0 ))
        
        # Stow
        self.robot.set_jointangles( self.arm, self.start_ja, 3.0 )
        rospy.sleep( rospy.Duration( 3.0 ))

        return True
        


if __name__ == '__main__':
    hon = HandoffNode()
    #hon.handoff_pos()
    rospy.spin()

    #ho.handoff()
    
