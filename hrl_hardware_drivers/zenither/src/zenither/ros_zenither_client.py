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
# ROS wrapper for zenither.

## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
## author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)


from threading import RLock

import roslib
roslib.load_manifest('zenither')
import zenither_config as zc
import rospy
from hrl_srvs.srv import Float_Int
from hrl_msgs.msg import FloatArray



class ZenitherClient():
    def __init__(self, robot):
        try:
            rospy.init_node('ZenitherClient')
            rospy.logout('ZenitherServer: Initialized Node')
        except rospy.ROSException:
            pass

        if robot not in zc.calib:
            raise RuntimeError('unknown robot')
        self.calib = zc.calib[robot]

        srv = '/zenither/move_position'
        rospy.wait_for_service(srv)
        self.move_position = rospy.ServiceProxy(srv, Float_Int)
        
        srv = '/zenither/stop'
        rospy.wait_for_service(srv)
        self.stop = rospy.ServiceProxy(srv, Float_Int)
        
        srv = '/zenither/apply_torque'
        rospy.wait_for_service(srv)
        self.apply_torque = rospy.ServiceProxy(srv, Float_Int)

        srv = '/zenither/torque_move_position'
        rospy.wait_for_service(srv)
        self.torque_move_position = rospy.ServiceProxy(srv, Float_Int)

        zenither_pose_topic = 'zenither_pose'
        self.h = None
        self.lock = RLock()
        rospy.Subscriber(zenither_pose_topic, FloatArray, self.pose_cb)
        
    #---------- functions to send zenither commands. -------------
    def estop(self):
        self.stop(0)

    def zenith(self, torque=None):
        if torque == None:
            torque=self.calib['zenith_torque']
        self.apply_torque(torque)

    def nadir(self, torque=None):
        if torque == None:
            torque=self.calib['nadir_torque']
        self.apply_torque(torque)


    #--------- zenither height functions --------------
    def pose_cb(self, fa):
        self.lock.acquire()
        self.h = fa.data[0]
        self.lock.release()

    ## return the current height of the zenither.
    def height(self):
        self.lock.acquire()
        h = self.h
        self.lock.release()
        return h



if __name__ == '__main__':
    import time

    cl = ZenitherClient('HRL2')

    cl.zenith()
    time.sleep(0.5)
    cl.estop()


