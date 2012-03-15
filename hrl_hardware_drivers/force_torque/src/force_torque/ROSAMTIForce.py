#!/usr/bin/python
#
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

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)
#  \author Cressel Anderson (Healthcare Robotics Lab, Georgia Tech.)
#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('force_torque')
import rospy
from hrl_msgs.msg import FloatArray
#import hrl_lib.rutils as ru

import AMTIForce2 as af

import threading
import time

#import rutils as ru

class AMTIForceServer(threading.Thread):
    def __init__(self, device_path, str_id):
        threading.Thread.__init__(self)
        try:
            rospy.init_node('AMTIForceServer')
            print 'AMTIForceServer: ros is up!'
        except rospy.ROSException:
            pass
        print 'AMTIForceServer: connecting to', device_path
        self.force_plate = af.AMTI_force(device_path)

        name = 'force_torque_' + str_id
        print 'AMTIForceServer: publishing', name, 'with type FloatArray'
        self.channel = rospy.Publisher(name, FloatArray, tcp_nodelay=True)

    def broadcast(self):
        print 'AMTIForceServer: started!'
        while not rospy.is_shutdown():
            time.sleep(1/5000.0)
            self.channel.publish(FloatArray(None, self.force_plate.read().T.tolist()[0]))

#DEPRECATED, use FTClient from ROSFTSensor with id = 0
#def AMTIForceClient():
#    return ru.FloatArrayListener('AMTIForceClient', 'force_plate', 100.0)

#import roslib; roslib.load_manifest('force_torque')
#import force_torque.ROSAMTIForce as ft
if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--path', action='store', default='/dev/robot/force_plate0', type = 'string',
                 dest='path', help='path to force torque device in (linux)')
    p.add_option('--name', action='store', default='ft1', type='string', 
                 dest='name', help='name given to FTSensor')
    opt, args = p.parse_args()

    server = AMTIForceServer(opt.path, opt.name)
    server.broadcast()


