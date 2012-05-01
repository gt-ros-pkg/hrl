#!/usr/bin/env python

import sys
import numpy as np
from threading import Condition
import pickle

from sklearn import preprocessing as pps, svm

import roslib; roslib.load_manifest('wouse')
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped

from mouse_listener_thread import MouseListener, MouseEvent
from wouse.srv import WouseRunStop, WouseRunStopRequest


class Wouse(object):
    """Listens for mouse events, detects wincing motions, and signals e-stop"""
    def __init__(self):
        """Initialize mouse-listener thread and svm classifier."""
        try:
            rospy.wait_for_service('wouse_run_stop', 5) 
            self.runstop_client=rospy.ServiceProxy('wouse_run_stop',
                                                        WouseRunStop, True)
            rospy.loginfo("Found wouse run-stop service")
        except:
            rospy.logerr("Cannot find wouse run-stop service")
            sys.exit()
       
        if not rospy.has_param('~svm_data_file'):
            rospy.logerr("Cannot find svm training data file. Exiting")
            sys.exit()

        (self.scaler, self.classifier) = self.init_classifier()
        self.mouse_event = MouseEvent()
        device_file = rospy.get_param('~device_file')
        self.condition = Condition()
        self.mouse_listener = MouseListener(self.mouse_event, self.condition,
                                            device_file)
        self.mouse_listener.start()
        
        rospy.Timer(rospy.Duration(0.1), self.ping_server)
        self.vector_pub = rospy.Publisher('wouse_movement', Vector3Stamped)
        self.v3st = Vector3Stamped()
        self.window = []
      
    def init_classifier(self):
        """Unpickle svm training data, train classifier"""
        if not rospy.has_param('~svm_data_file'):
            rospy.logerr("Cannot find svm training data file. Exiting")
            sys.exit()
        else:
            filename = rospy.get_param('~svm_data_file')
            with open(filename, 'rb') as f:
                svm_data = pickle.load(f)
            labels = svm_data['labels']
            data = svm_data['data']
            scaler = pps.Scaler().fit(data)
            data_scaled = scaler.transform(data)
            classifier = svm.SVC()
            classifier.fit(data_scaled, labels)
            return (scaler, classifier)

    def ping_server(self, event):
        """Send updated timestamp to Runstop server."""
        req = WouseRunStopRequest(False, False, rospy.Time.now())
        self.runstop_client(req)

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
        self.update_detection(self.v3st.vector.x, 
                              self.v3st.vector.y, 
                              self.v3st.header.stamp)

    def update_detection(self, x, y, time):
        """Filter out small movements, check classifier, call stop if needed."""
        if (x**2+y**2)**(0.5) < 2.5:
            return
        if self.classify_svm(x, y, time):
            self.runstop_client(WouseRunStopRequest(True, False,rospy.Time.now()))
            rospy.loginfo("Wince Detected, stopping robot!")
        else:
            rospy.loginfo("Not a wince")

    def classify_svm(self, x, y, time):
        """Build the descriptor vector for the incoming mouse event, and classify with the svm."""
        datum = []
        mag = (x**2+y**2)**(0.5)
        angle = np.arctan2(y,x)
        datum.append(mag)
        datum.append(angle)
        self.window.append([x,y,time])
        while (self.window[-1][-1] - self.window[0][-1]) > rospy.Duration(0.25):
            self.window.pop(0)
        win = np.array(self.window)
        datum.append(len(self.window))
        win_x = np.sum(win[:,0])
        win_y = np.sum(win[:,1])
        win_mag = (win_x**2+win_y**2)**(0.5)
        datum.append(win_mag/len(self.window))
        datum.append(np.arctan2(win_y, win_x))
        datum_scaled = self.scaler.transform(datum)
        prediction = self.classifier.predict(datum_scaled)
        if prediction[0] == 1.:
            return True

if __name__=='__main__':
    rospy.init_node('wouse_node')
    wouse = Wouse()
    while not rospy.is_shutdown():
        wouse.poll()
