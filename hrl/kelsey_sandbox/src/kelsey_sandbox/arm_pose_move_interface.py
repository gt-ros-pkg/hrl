#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
import rospy
from std_msgs.msg import Bool

from arm_pose_move_gui import Ui_Frame as QTArmPoseMoveGUIFrame
from msg import ArmPoseMoveCmd

HEARTBEAT_TOPIC = '/arm_pose_move_gui/heartbeat'
COMMAND_TOPIC = '/arm_pose_move_gui/command'
MONITOR_RATE = 20

class ArmPoseMoveGUIFrame(QtGui.QFrame):
    def __init__(self):
        super(ArmPoseMoveGUIFrame, self).__init__()

        self.heartbeat_pub = rospy.Publisher(HEARTBEAT_TOPIC, Bool)
        self.cmd_pub = rospy.Publisher(COMMAND_TOPIC, ArmPoseMoveCmd)

        self.init_ui()

    def init_ui(self):
        self.ui = QTArmPoseMoveGUIFrame()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_clk)
        self.ui.stop_button.clicked.connect(self.stop_clk)
        self.ui.reset_button.clicked.connect(self.reset_clk)
        trajectories = ['test1', 'test2']
        for traj in trajectories:
            self.ui.traj_combo.addItem(traj)
        poses = ['ptest1', 'ptest2']
        for pose in poses:
            self.ui.joint_combo.addItem(pose)

        self.monitor_timer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.monitor_timer, QtCore.SIGNAL("timeout()"), self.monitor_cb)
        self.monitor_timer.start(MONITOR_RATE)

    def start_clk(self):
        msg = ArmPoseMoveCmd()
        msg.type = msg.START
        if self.ui.traj_button.isChecked():
            msg.traj_name = str(self.ui.traj_combo.currentText())
            msg.is_trajectory = True
            msg.is_forward = self.ui.forward_button.isChecked()
        else:
            msg.traj_name = str(self.ui.joint_combo.currentText())
            msg.is_trajectory = False
            msg.is_forward = self.ui.forward_button.isChecked()
        self.cmd_pub.publish(msg)

    def stop_clk(self):
        msg = ArmPoseMoveCmd()
        msg.type = msg.STOP
        self.cmd_pub.publish(msg)

    def reset_clk(self):
        msg = ArmPoseMoveCmd()
        msg.type = msg.RESET
        self.cmd_pub.publish(msg)

    def monitor_cb(self):
        self.heartbeat_pub.publish(Bool())

def main():
    rospy.init_node("arm_pose_move_interface")
    app = QtGui.QApplication(sys.argv)
    frame = ArmPoseMoveGUIFrame()
    frame.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
