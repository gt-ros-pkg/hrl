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
from hrl_rfid.srv import RfidSrv
import hrl_rfid.lib_M5e as M5e

import time
import thread

class ROS_M5e_Client():
    QUERY_MODE = 'query'
    TRACK_MODE = 'track'

    def __init__(self, name = 'reader1', callbacks=[]):
        self.name = name
        self._create_ros_objects()
    
        self.callbacks = callbacks
        self.last_read = ['', '', -1] # antenna_name, tagID, rssi

        try:
            rospy.init_node( self.name + '_listener', anonymous=True )
        except rospy.ROSException:
            pass

        self._sub = rospy.Subscriber( '/rfid/' + self.name + '_reader', RFIDread, self._sub_cb)

    def _sub_cb(self, datum):
        [ cb( datum ) for cb in self.callbacks ]
        self.last_read = [datum.antenna_name, datum.tagID, datum.rssi]

    def unregister( self ):
        # Stop processing new reads.
        self._sub.unregister()

    # ROS Services
    def stop(self):
        self._mode_service_obj([ '' ])

    def track_mode(self, tag_id):
        self._mode_service_obj([ self.TRACK_MODE, tag_id ])

    def query_mode(self):
        self._mode_service_obj([ self.QUERY_MODE ])

    # Create ROS Objects (for internal use only)
    def _create_ros_objects(self):
        reader_service_name = '/rfid/'+self.name+'_mode'
        rospy.wait_for_service(reader_service_name)
        self._mode_service_obj = rospy.ServiceProxy(reader_service_name,
                                                    RfidSrv)

    def read(self, antenna = ''):
        if antenna == '':
            return self.last_read
        else:
            r = self.last_read
            while r[0] != antenna and not rospy.is_shutdown():
                time.sleep(0.02)
                r = self.last_read
            return r
        
