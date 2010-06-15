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
import rospy
from hrl_lib.msg import FloatArray
from hrl_lib.srv import Float_None
import hrl_lib.rutils as ru
import hrl_lib.util as ut
from std_srvs.srv import Empty
import tf

import time
import sys
import numpy as np, math


class ZenitherClient():
    def __init__(self, name):

        rospy.init_node( name )
        self.sub = rospy.Subscriber('/zenither', FloatArray, self.pose_callback)
        self.height = None
        self.callbacks = []
        
    def pose( self ):
        return self.height

    def pose_callback( self, msg ):
        self.height = msg.data[0]
        for cb in self.callbacks:
            cb( self.height )

    


def update_transform( height, broadcaster ):
    # ELE only transform
    broadcaster.sendTransform( (+0.20, 0.0, height),
                               tf.transformations.quaternion_from_euler( 0.0, 0.0, 0.0 ),
                               rospy.Time.now(),
                               'zenither',
                               'base_link' )

if __name__ == '__main__':
    import functools
    br = tf.TransformBroadcaster()
    zc = ZenitherClient( 'ele_zen_client' )
    zc.callbacks.append( functools.partial(update_transform,
                                           broadcaster = br ))

    rospy.spin()
