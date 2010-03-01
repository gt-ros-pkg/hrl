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
from hrl_rfid.srv import StringArray_NoneResponse
import hrl_rfid.lib_M5e as M5e
import hrl_lib.rutils as ru

import time


# Modeled off lib_M5e.M5e_Poller
class ROS_M5e():
    QUERY_MODE = 'query'
    TRACK_MODE = 'track'
    
    def __init__(self, name = 'reader1', readPwr = 2300,
                 portStr = '/dev/robot/RFIDreader',
                 antFuncs = [], callbacks = []):
        try:
            rospy.init_node( 'rfid_m5e_' + name )
        except rospy.ROSException:
            pass

        self.mode = ''
        self.name = name + '_reader'

        print 'ROS_M5e: Launching RFID Reader'
        print 'ROS_M5e: Please check out our related work @ http://www.hsi.gatech.edu/hrl/project_rfid.shtml'

        print 'ROS_M5e: '+self.name+' Building & Connecting to reader'
        self.reader = M5e.M5e(readPwr=readPwr, portSTR = portStr)
        self.antFuncs = antFuncs
        self.callbacks = callbacks + [self.broadcast]

        print 'ROS_M5e: publishing RFID reader with type RFIDread to channel /rfid/'+name+'_reader'
        self.channel       = rospy.Publisher('/rfid/'+name+'_reader', RFIDread)
        self._mode_service_obj = rospy.Service('/rfid/'+name+'_mode',
                                                StringArray_None, self._mode_service)

        print 'ROS_M5e: '+self.name+' Inialized and awaiting instructions'

        while not rospy.is_shutdown():
            if self.mode == self.QUERY_MODE:
                for aF in self.antFuncs:
                    antennaName = aF(self.reader)    # let current antFunc make appropriate changes
                    results = self.reader.QueryEnvironment()
                    if len(results) == 0:
                        datum = [antennaName, '', -1]
                        [cF(datum) for cF in self.callbacks]
                    for tagid, rssi in results:
                        datum = [antennaName, tagid, rssi]
                        [cF(datum) for cF in self.callbacks]
            elif self.mode == self.TRACK_MODE:
                for aF in self.antFuncs:
                    antennaName = aF(self.reader)    # let current antFunc make appropriate changes
                    tagid = self.tag_to_track
                    rssi = self.reader.TrackSingleTag(tagid)
                    #if rssi != -1:
                    datum = [antennaName, tagid, rssi]
                    [cF(datum) for cF in self.callbacks]
            else:
                time.sleep(0.005)

        print 'ROS_M5e: '+self.name+' Shutting down reader'


            
    def broadcast(self, data):
        antName, tagid, rssi = data
        self.channel.publish(RFIDread(None, antName, tagid, rssi))
    
    # For internal use only
    def _mode_service(self, data):
        val = data.data
        if len(val) == 0:
            print 'ROS_M5e: Mode Service called with invalid argument: ', val
        elif len(val) == 1:
            if val[0] == self.QUERY_MODE:
                print 'ROS_M5e: '+self.name+' Entering Query Mode'
                self.mode = self.QUERY_MODE
            else:
                print 'ROS_M5e: '+self.name+' Stopping Reader'
                self.mode = ''
        elif len(val) == 2:
            if val[0] == self.TRACK_MODE and len(val[1]) == 12:
                print 'ROS_M5e: '+self.name+' Entering Track Mode: ', val[1]
                self.mode = self.TRACK_MODE
                self.tag_to_track = val[1]
            else:
                print 'ROS_M5e: Mode Service called with invalid argument: ', val
        else:
            print 'ROS_M5e: Mode Service called with invalid argument: ', val
        return StringArray_NoneResponse()



# -----------------------------------------------
# Likely Callbacks: (various antennas)
# -----------------------------------------------

def EleLeftEar(M5e):
    M5e.ChangeAntennaPorts(2,2)
    return 'EleLeftEar'

def EleRightEar(M5e):
    M5e.ChangeAntennaPorts(1,1)
    return 'EleRightEar'

def Hand_Right_1(M5e):
    # GPIO1 = 1, GPIO2 = 0
    M5e.TransmitCommand('\x02\x96\x01\x01')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x00')
    M5e.ReceiveResponse()
    return 'Hand_Right_1'

def Hand_Right_2(M5e):
    # GPIO1 = 1, GPIO2 = 1
    M5e.TransmitCommand('\x02\x96\x01\x01')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x01')
    M5e.ReceiveResponse()
    return 'Hand_Right_2'

def Hand_Left_1(M5e):
    # GPIO1 = 0, GPIO2 = 0
    M5e.TransmitCommand('\x02\x96\x01\x00')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x00')
    M5e.ReceiveResponse()
    return 'Hand_Left_1'

def Hand_Left_2(M5e):
    # GPIO1 = 0, GPIO2 = 1
    M5e.TransmitCommand('\x02\x96\x01\x00')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x01')
    M5e.ReceiveResponse()
    return 'Hand_Left_2'

def PrintDatum(data):
    ant, ids, rssi = data
    print data

if __name__ == '__main__':
    import optparse

    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='device',
                 help='Which RFID device to initialize.')
    opt, args = p.parse_args()

    if opt.device == 'ears':
        print 'Starting Ears RFID Services'
        ros_rfid = ROS_M5e( name = 'ears', readPwr = 3000,
                            portStr = '/dev/robot/RFIDreader',
                            antFuncs = [EleLeftEar, EleRightEar],
                            callbacks = [] )
        rospy.spin()

    if opt.device == 'inhand':
        print 'Starting Ears RFID Services'
        ros_rfid = ROS_M5e( name = 'inhand', readPwr = 3000,
                            portStr = '/dev/robot/inHandReader',
                            antFuncs = [Hand_Right_1, Hand_Right_2,
                                        Hand_Left_1, Hand_Left_2 ],
                            callbacks = [] )
        rospy.spin()


