#!/usr/bin/env python

import math
from threading import Condition
from collections import deque

import roslib; roslib.load_manifest('wouse')
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped

from mouse_listener_thread import MouseListener, MouseEvent
from wouse.srv import WouseRunStop, WouseRunStopRequest

class Wouse(object):
    """Listens for mouse events, detects wincing motions, and signals e-stop"""
    def __init__(self):
        try:
            rospy.wait_for_service('wouse_run_stop', 5) 
            self.runstop_client=rospy.ServiceProxy('wouse_run_stop',
                                                        WouseRunStop, True)
            rospy.loginfo("Found wouse run-stop service")
        except:
            rospy.logerr("Cannot find wouse run-stop service")
        
        self.mouse_event = MouseEvent()
        device_file = rospy.get_param('~wouse_device_file', '/dev/input/mouse1')
        self.condition = Condition()
        self.mouse_listener = MouseListener(self.mouse_event,
                                            self.condition,
                                            device_file)
        self.mouse_listener.start()
        
        #rospy.Timer(rospy.Duration(0.1), self.ping_server)
        self.vector_pub = rospy.Publisher('wouse_movement', Vector3Stamped)
        self.v3st = Vector3Stamped()
       
    def ping_server(self, event):
        """Send updated timestamp to Runstop server."""
        req = WouseRunStopRequest(False, False, rospy.Time.now())
        self.runstop_client(req)
#        print "Sent time: ", req.time
        #hdr = Header()
        #hdr.stamp = rospy.Time.now()
        #self.ping_server_pub.publish(hdr)

    def poll(self):
        """Wait for new mouse event from listener thread, then pub/process"""
        with self.condition:
            self.condition.wait() 
            self.v3st.header.stamp = rospy.Time.now()
            self.v3st.vector.x = self.mouse_event.rel_x
            self.v3st.vector.y = self.mouse_event.rel_y
            if self.mouse_event.x_overflow or self.mouse_event.y_overflow:
                self.v3st.vector.z = 1
        self.vector_pub.publish(self.v3st)
       # self.update_detection(self.v3st.vector.x, 
       #                       self.v3st.vector.y, 
       #                       self.v3st.header.stamp)

    def update_detection(self, x, y, time):
        """Use to point to detection function."""
        self.threshold(x, y, time)
        
    def threshold(self,x, y, time):
        """Detect wince signals based upon simple threshold"""
        thresh = 10
        if math.sqrt(x*x+y*y) > thresh:
            rospy.loginfo("Wince Detected, stopping robot!")
            self.runstop_client(WouseRunStopRequest(True, False,rospy.Time.now()))


if __name__=='__main__':
    rospy.init_node('wouse_node')
    wouse = Wouse()
    while not rospy.is_shutdown():
        wouse.poll()
