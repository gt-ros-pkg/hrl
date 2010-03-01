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

#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)



# ROS imports
import roslib; roslib.load_manifest('hrl_rfid')
import rospy
from hrl_rfid.msg import RFIDread
from hrl_rfid.srv import StringArray_None
import hrl_rfid.lib_M5e as M5e
import hrl_lib.rutils as ru

import time


class ROS_M5e_Client():
    QUERY_MODE = 'query'
    TRACK_MODE = 'track'

    def __init__(self, name = 'reader1'):
        self.name = name
        self._create_ros_services()

    # For internal use only
    def _create_ros_services(self):
        reader_service_name = '/rfid/'+self.name+'_mode'
        rospy.wait_for_service(reader_service_name)
        self._mode_service_obj = rospy.ServiceProxy(reader_service_name,
                                                    StringArray_None)
      
    def stop(self):
        self._mode_service_obj([ '' ])

    def track_mode(self, tag_id):
        self._mode_service_obj([ self.TRACK_MODE, tag_id ])

    def query_mode(self):
        self._mode_service_obj([ self.QUERY_MODE ])
