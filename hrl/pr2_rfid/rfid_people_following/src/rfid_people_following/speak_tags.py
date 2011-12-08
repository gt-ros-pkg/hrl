#! /usr/bin/python
import time
import roslib
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('sound_play')

import rospy
import time
import numpy as np, math
import hrl_rfid.ros_M5e_client as rmc
import sound_play.libsoundplay as lsp

r = rmc.ROS_M5e_Client('ears')
r.query_mode()

speaker = lsp.SoundClient()

i = 0
reads_dict = {}

while not rospy.is_shutdown():
    if i % 2 == 0:
        read = r.read('EleLeftEar')
    else:
        read = r.read('EleRightEar')
    i = i + 1

    print 'read:', read

    if read[-1] > 92:
        if not reads_dict.has_key( read[-2] ):
            reads_dict[ read[-2] ] = time.time() - 3.5

        if time.time() - reads_dict[ read[-2] ] > 3.0:
            reads_dict[ read[-2] ] = time.time()
            print read
            speaker.say( read[-2] )

r.stop()



