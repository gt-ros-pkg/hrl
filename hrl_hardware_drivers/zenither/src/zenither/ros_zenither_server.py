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

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)
## author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)


import roslib
roslib.load_manifest('zenither')
import rospy
from hrl_srvs.srv import Float_Int

import time
import sys


class ZenitherServer():
    def __init__(self, zenither):
        try:
            rospy.init_node('ZenitherServer')
            rospy.logout('ZenitherServer: Initialized Node')
        except rospy.ROSException:
            pass

        self.zenither = zenither
        self.smp = rospy.Service('/zenither/move_position', Float_Int,
                                 self.move_position)
        self.ss = rospy.Service('/zenither/stop', Float_Int,
                                self.estop)
        self.sat = rospy.Service('/zenither/apply_torque', Float_Int,
                                 self.apply_torque)
        self.stmp = rospy.Service('/zenither/torque_move_position',
                                  Float_Int, self.torque_move_position)

    def move_position(self, msg):
        print 'move_position is UNTESTED'
        #self.zenither.move_position( msg.value )
        return True
    
    def apply_torque(self, req):
        self.zenither.nadir(req.value)
        return True

    def estop(self, req):
        self.zenither.estop()
        return True

    def torque_move_position(self, req):
        self.zenither.torque_move_position(req.value)
        return True



if __name__ == '__main__':
    import zenither.zenither as zenither
    z = zenither.Zenither('HRL2', pose_read_thread = True)
    zs = ZenitherServer(z)
    rospy.spin()

    # rosservice call /zenither/move_position 0.3
