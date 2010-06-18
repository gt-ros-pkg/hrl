#!/usr/bin/python
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# ROS wrapper for lib_zenither.
## author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)


import roslib
roslib.load_manifest('zenither')
import zenither.lib_zenither as lib_z
import rospy
from hrl_lib.msg import FloatArray
from hrl_lib.srv import Float_Int
import hrl_lib.rutils as ru
import hrl_lib.util as ut
from std_srvs.srv import Empty
import tf

import threading
import time
import sys

class ZenitherServer(threading.Thread):
    def __init__(self, zenither, frequency=100):
        threading.Thread.__init__(self)
        try:
            rospy.init_node('ZenitherServer')
            rospy.logout('ZenitherServer: Initialized Node')
        except rospy.ROSException:
            pass

        self.frequency = frequency
        self.should_run = True
        self.calibrating = False
        self.last_warn = -1.0
        self.zenither = zenither
        self.zenither.callbacks.append( self.publish_position ) # callback used any time get_position_meters is called

        name = 'zenither'
        rospy.logout('ZenitherServer: publishing %s', name + '_pose')
        self.pub = rospy.Publisher(name, FloatArray)
        self.srv_set_origin = rospy.Service( name + '/set_origin',
                                             Empty,
                                             self.set_origin )
        self.srv_move_position = rospy.Service( name + '/move_position',
                                                Float_Int,
                                                self.move_position )
        self.start()

    def run( self ):
        rospy.logout('ZenitherServer: Position publishing thread started')
        while self.should_run and not rospy.is_shutdown():
            time.sleep(1.0/self.frequency)
            if not self.zenither.calibrated:
                if time.time() - self.last_warn > 2.0 and not self.calibrating:
                    rospy.logout('ZenitherServer: Origin not yet calibrated')
                    self.last_warn = time.time()
            else:
                self.zenither.get_position_meters() # will callback publish_position from this call and any others during lower-level functions

    def publish_position( self, pose ):
        self.pub.publish( FloatArray( None, [pose] ))

    def move_position( self, msg ):
        self.zenither.move_position( msg.value )
        return True

    def set_origin( self, msg ):
        self.calibrating = True
        rospy.logout('ZenitherServer: Initializing calibration')
        raw_input('ZenitherServer: SAFE TO CALIBRATE? [Hit ENTER]')
        self.zenither.nadir()
        time.sleep(10)
        self.zenither.estop() # This is somewhat of a hack...
        self.zenither.set_origin()
        self.zenither.move_position(0.25)
        rospy.logout('ZenitherServer: Calibration complete')
        self.calibrating = False

    def stop( self ):
        self.should_run = False 
        self.join(3) 
        if (self.isAlive()): 
            raise RuntimeError('ZenitherServer: Unable to stop position publishing thread.')

if __name__ == '__main__':
    z = lib_z.Zenither('El-E')
    zs = ZenitherServer( z )
    while not rospy.is_shutdown():
        time.sleep(0.1)

    # rosservice call /zenither/set_origin
    # rosservice call /zenither/move_position 0.3
