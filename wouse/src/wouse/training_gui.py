#!/usr/bin/env python
import sys
import os
import random
import csv
import math
import pygame

import roslib; roslib.load_manifest('wouse')
import rospy
from geometry_msgs.msg import Vector3Stamped

from PySide.QtCore import *
from PySide.QtGui import *
from PySide.QtUiTools import QUiLoader

#DEGREES = ['WEAK', 'AVERAGE', 'STRONG']
DEGREES = ['']
ACTIONS = ['WINCE', 'NOD', 'SHAKE', 'JOY', 'SUPRISE', 'FEAR', 'ANGER', 
            'DISGUST', 'SADNESS']
SYMBOLS = ["**"*25, "%%"*25, "^v"*25, '##'*25, '&&'*25, '$$'*25]

class WouseSetupDialog(object):
    """A dialog box for setting session parameters for training the wouse."""
    def __init__(self):
        """ Load .ui file from QtDesigner, add callbacks as necessary"""
        ui_file = WOUSE_PKG+'/src/wouse/wouse_train_options.ui'
        self.dialog = QUiLoader().load(ui_file)
        self.dialog.rounds_spin.valueChanged.connect(self.update_time)
        self.dialog.recording_spin.valueChanged.connect(self.update_time)
        self.dialog.recovery_spin.valueChanged.connect(self.update_time)
        self.dialog.file_button.clicked.connect(self.file_button_cb) 
        self.dialog.file_field_edit.setText(WOUSE_PKG+'/data/')
        self.dialog.buttonBox.accepted.connect(self.ok_cb)
        self.dialog.buttonBox.rejected.connect(self.cancel_cb)
        self.update_time()

    def file_button_cb(self):
        """Use file dialog to get .csv file.  Check for csv, update Lineedit"""
        direc = self.dialog.file_field_edit.text()
        filename = QFileDialog.getOpenFileName(self.dialog,
                                    caption="File to stop wouse training data",
                                    dir=direc,
                                    filter="*.csv")
        if len(filename[0]) != 0:
            if filename[0][-4:] != '.csv':
                QMessageBox.warning(self.dialog, "Warning: Invalid File",
                "Warning: Selected File does not appear to be a CSV (.csv)\
                data file.")
            self.dialog.file_field_edit.setText(filename[0])
         
    def calc_time(self):
        """Calculate the time (s) required for full run with current settings"""
        tot_time = (self.dialog.recording_spin.value()+
                    self.dialog.recovery_spin.value())
        return len(ACTIONS)*self.dialog.rounds_spin.value()*tot_time

    def update_time(self):
        """Parse time to minutes:seconds format, update interface"""
        time = self.calc_time()
        mins = str(int(time)/60)
        secs = str(int(round(time%60.)))
        if len(secs)==1:
            secs = "".join(['0',secs])
        self.dialog.duration.setText('%s:%s' %(mins,secs))

    def ok_cb(self):
        """Check for acceptable file. Warn if bad, if good, close, return 1"""
        if self.dialog.file_field_edit.text()[-4:] != '.csv':
            return QMessageBox.warning(self.dialog, "Warning: Invalid File",
            "Please choose a valid CSV (.csv) data file.")
        self.dialog.accept()

    def cancel_cb(self):
        """ Close dialog, return 0/Rejected"""
        self.dialog.reject()

class WouseTrainer(object):
    """ A class for printing random facial expression commands,\\
        and saving data from a topic of wouse movement data."""
    def __init__(self):
        rospy.Subscriber('/wouse_movement', Vector3Stamped, self.movement_cb)
        #QtDialog for setting parameters for session
        app = QApplication([])
        self.setup_gui = WouseSetupDialog()
        self.setup_gui.dialog.show()
        if self.setup_gui.dialog.exec_() == 0:
            sys.exit()
        self.rounds = self.setup_gui.dialog.rounds_spin.value()
        self.record_dur = self.setup_gui.dialog.recording_spin.value()
        self.recovery_dur = self.setup_gui.dialog.recovery_spin.value() 
        
        #Open file for recoding data
        output_file = self.setup_gui.dialog.file_field_edit.text()
        self.csv_writer = csv.writer(open(output_file, 'ab'))

        #Init pygame, used for playing sounds
        pygame.init()
        self.sound_new = pygame.mixer.Sound(WOUSE_PKG+'/sounds/new_item.wav')
        self.sound_done = pygame.mixer.Sound(WOUSE_PKG+'/sounds/item_done.wav')

        self.degree='AVERAGE'
        self.recording = False

    def movement_cb(self, v3s):
        """Write a new line to the csv file for incoming data."""
        if self.recording:
            line = [self.degree, self.behavior, v3s.header.stamp.to_sec(),
                    v3s.vector.x, v3s.vector.y]
            self.csv_writer.writerow(line)

    def run(self, rounds, record_dur, recovery_dur):
        """Perform training given parameters and actions."""
        act_list = rounds*ACTIONS
        random.shuffle(act_list)
        count = 1
        print "Starting in: "
        countdown = range(1,10)
        countdown.reverse()
        for number in countdown:
            print "%s" %number
            rospy.sleep(1)
        while len(act_list) > 0 and not rospy.is_shutdown():
            self.behavior = act_list.pop()
            bar = random.choice(SYMBOLS)
            self.recording = True
            self.sound_new.play()
            print "\r\n"*15
            print bar
            print "%s:     %s" %(count, self.behavior)
            print bar
            print "\r\n"*15
            rospy.sleep(record_dur)
            self.recording = False
            self.sound_done.play()
            count += 1
            rospy.sleep(recovery_dur)
        if not act_list:
            print "Training Session Completed"
        else:
            print "Training Session Ended Early"

if __name__=='__main__':
    rospy.init_node('wouse_trainer')
    WOUSE_PKG = roslib.packages.get_pkg_dir('wouse')
    wt = WouseTrainer()
    wt.run(wt.rounds, wt.record_dur, wt.recovery_dur)

