#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic

import roslib
roslib.load_manifest("std_msgs")
roslib.load_manifest("rospy")
roslib.load_manifest("smach_ros")

import rospy
from std_msgs.msg import Bool
from smach_msgs.msg import SmachContainerStatus

from servoing_ui import Ui_Frame as QTServoingFrame

disabled_button_ss = """QPushButton { font: 75 18pt "AlArabiya";
                                      background-color: rgb(190, 190, 190);}"""

STATUS_DICT = { "UI_FIND_TAG_WAIT" : "Navigate the robot so it can see\nthe AR tag.",
                "FIND_AR_TAG" : "Locating tag...",
                "UI_SERVO_WAIT" : "Confirm the robot has found the\ntag in rviz.",
                "SERVOING" : "Servoing to tag..."}

class ServoingFrame(QtGui.QFrame):
    def __init__(self):
        super(ServoingFrame, self).__init__()
        self.init_ui()

        rospy.Subscriber("/pr2_servo/smach/container_status", SmachContainerStatus, 
                         self.smach_status_cb)
        self.cur_smach_state = None

        self.find_tag_pub = rospy.Publisher("/pr2_ar_servo/find_tag", Bool)
        self.tag_confirm_pub = rospy.Publisher("/pr2_ar_servo/tag_confirm", Bool)
        self.preempt_pub = rospy.Publisher("/pr2_ar_servo/preempt", Bool)

    def smach_status_cb(self, msg):
        for state in msg.active_states:
            if state == self.cur_smach_state:
                continue
            if state in STATUS_DICT:
                self.ui.state_information.setText(STATUS_DICT[state])
                self.cur_smach_state = state
            if state == "UI_FIND_TAG_WAIT" or state == "FIND_AR_TAG":
                self.enable_button(self.ui.find_ar_tag, self.find_ar_tag_ss)
                self.disable_button(self.ui.begin_servoing)
            if state == "UI_SERVO_WAIT":
                self.enable_button(self.ui.begin_servoing, self.begin_servoing_ss)
                self.disable_button(self.ui.find_ar_tag)
            if state == "SERVOING":
                self.disable_button(self.ui.find_ar_tag)
                self.disable_button(self.ui.begin_servoing)
                
    def enable_button(self, button, ss):
        button.setStyleSheet(ss)
        button.setEnabled(True)
                
    def disable_button(self, button):
        button.setStyleSheet(disabled_button_ss)
        button.setDisabled(True)

    def init_ui(self):
        self.ui = QTServoingFrame()
        self.ui.setupUi(self)
        self.ui.stop_moving.clicked.connect(self.stop_moving_clk)
        self.ui.find_ar_tag.clicked.connect(self.find_ar_tag_clk)
        self.ui.begin_servoing.clicked.connect(self.begin_servoing_clk)
        self.ui.state_information.setText("State Information")
        self.stop_moving_ss = self.ui.stop_moving.styleSheet()
        self.find_ar_tag_ss = self.ui.find_ar_tag.styleSheet()
        self.begin_servoing_ss = self.ui.begin_servoing.styleSheet()

    def stop_moving_clk(self):
        self.preempt_pub.publish(Bool(True))

    def find_ar_tag_clk(self):
        self.find_tag_pub.publish(Bool(True))

    def begin_servoing_clk(self):
        self.tag_confirm_pub.publish(Bool(True))

def main():
    rospy.init_node("servoing_interface")
    app = QtGui.QApplication(sys.argv)
    sframe = ServoingFrame()
    sframe.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
