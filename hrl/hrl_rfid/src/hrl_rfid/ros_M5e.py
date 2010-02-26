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




import roslib; roslib.load_manifest('rfid')
import rospy
from rfid.msg import RFIDread
import time
import rfid.M5e as M5e
import hrl_lib.rutils as ru

class Broadcast_M5e_ROS():
    def __init__(self):
        try:
            rospy.init_node('M5eRFIDServer')
            print 'M5eRFIDServer: ros is up!'
            print 'Please check out our related work @ http://www.hsi.gatech.edu/hrl/project_rfid.shtml'
        except rospy.ROSException:
            pass

        print 'M5eRFIDServer: connecting to M5e Poller'
        print 'M5eRFIDServer: publishing RFID reader with type RFIDread' 
        self.channel       = rospy.Publisher('rfid_reader', RFIDread)
        
    def broadcast(self, data):
        antName, tagid, rssi = data
        self.channel.publish(RFIDread(None, antName, tagid, rssi))

class Client_M5e_ROS():
    def __init__(self):
        self.listener     = ru.GenericListener('rfid_client', RFIDread, 'rfid_reader', 20)
    
    def read(self, fresh=False):
        return self.listener.read(fresh=fresh)

# client = MR.Client_M5e_ROS()

# while True:
#     a = client.read(True)
#     #print a
#     print [a.antenna_name, a.tagID, a.rssi]

if __name__ == '__main__':
    import rfid.M5e as M
    import time

    ros_serv = Broadcast_M5e_ROS()

    def EleLeftEar(M5e):
        M5e.ChangeAntennaPorts(1,1)
        return 'EleLeftEar'

    def EleRightEar(M5e):
        M5e.ChangeAntennaPorts(2,2)
        return 'EleRightEar'

    def PrintDatum(data):
        ant, ids, rssi = data
        print data
        
    r = M.M5e(readPwr=3000)
    print 'M5e RFID Server: started!'
    q = M.M5e_Poller(r, antfuncs=[EleLeftEar, EleRightEar], callbacks=[ros_serv.broadcast])
    #q = M.M5e_Poller(r, antfuncs=[EleLeftEar, EleRightEar], callbacks=[PrintDatum, ros_serv.broadcast])
    q.query_mode()
    
    #q.stop()


