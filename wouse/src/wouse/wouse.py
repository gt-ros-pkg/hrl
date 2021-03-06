#!/usr/bin/env python



import sys
import numpy as np
from threading import Condition
import pickle
import csv

import pygame
from sklearn import preprocessing as pps, svm

import roslib; roslib.load_manifest('wouse')
from Object import ROSObject
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped

from wouse.srv import WouseRunStop, WouseRunStopRequest


class Wouse(object):
    """
    Subscribes to mouse movements, detects wincing, and signals e-stop.
    """
    def __init__(self, svm_datafile):
        """Initialize svm classifier and needed services."""
        try:
            rospy.wait_for_service('wouse_run_stop', 5) 
            self.runstop_client=rospy.ServiceProxy('wouse_run_stop',
                                                        WouseRunStop, True)
            rospy.loginfo("Found wouse run-stop service")
        except:
            rospy.logerr("Cannot find wouse run-stop service")
            sys.exit()

        (self.scaler, self.classifier) = self.init_classifier(svm_datafile)
        rospy.loginfo("SVM Trained on data from %s" %svm_datafile)
        rospy.Subscriber('wouse_movement', Vector3Stamped, self.movement_cb)
        rospy.Timer(rospy.Duration(0.1), self.ping_server)
        self.window = []
        self.data = []
        pygame.init()
        self.sound_new = pygame.mixer.Sound('../../sounds/new_item.wav')
        #self.csv_writer = csv.writer(open(out_file, 'ab'))
        rospy.loginfo("[%s]: Ready" %rospy.get_name())
      
    def init_classifier(self, filename):
        """Unpickle svm training data, train classifier"""
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

    def movement_cb(self, v3):
        """Filter out small movements, check classifier, call stop if needed."""
        #line = [v3.header.stamp.to_sec(), v3.vector.x, v3.vector.y]
        #self.csv_writer.writerow(line)
        if (v3.vector.x**2+v3.vector.y**2)**(0.5) < 2.5:
            return
        if self.classify_svm(v3.vector.x, v3.vector.y, v3.header.stamp):
            self.runstop_client(WouseRunStopRequest(True, False, rospy.Time.now()))
            self.sound_new.play()
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
    wouse = Wouse(sys.argv[1])
    rospy.spin()
