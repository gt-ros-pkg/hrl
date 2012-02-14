#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("rosparam")
import rospy
import rosparam
from std_msgs.msg import Bool

from arm_pose_move_gui import Ui_Frame as QTArmPoseMoveGUIFrame
from msg import ArmPoseMoveCmd
from arm_pose_move_backend import HEARTBEAT_TOPIC, COMMAND_TOPIC, MONITOR_RATE

class ArmPoseMoveGUIFrame(QtGui.QFrame):
    def __init__(self):
        super(ArmPoseMoveGUIFrame, self).__init__()
        params = rosparam.get_param("/arm_pose_move")
        self.pose_dict = params['poses']
        self.traj_dict = params['trajectories']

        self.heartbeat_pub = rospy.Publisher(HEARTBEAT_TOPIC, Bool)
        self.cmd_pub = rospy.Publisher(COMMAND_TOPIC, ArmPoseMoveCmd)

        self.init_ui()

    def init_ui(self):
        self.ui = QTArmPoseMoveGUIFrame()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_clk)
        self.ui.stop_button.clicked.connect(self.stop_clk)
        self.ui.reset_button.clicked.connect(self.reset_clk)
        self.traj_name_list, self.pose_name_list = [], []
        for traj in self.traj_dict:
            self.ui.traj_combo.addItem(self.traj_dict[traj]['text'])
            self.traj_name_list.append(traj)
        for pose in self.pose_dict:
            self.ui.joint_combo.addItem(self.pose_dict[pose]['text'])
            self.pose_name_list.append(pose)

        self.monitor_timer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.monitor_timer, QtCore.SIGNAL("timeout()"), self.monitor_cb)
        self.monitor_timer.start(MONITOR_RATE)

    def start_clk(self):
        msg = ArmPoseMoveCmd()
        msg.type = msg.START
        if self.ui.traj_button.isChecked():
            msg.traj_name = self.traj_name_list[self.ui.traj_combo.currentIndex()]
            msg.is_trajectory = True
            msg.is_forward = self.ui.forward_button.isChecked()
            msg.is_setup = self.ui.setup_box.isChecked()
        else:
            msg.traj_name = self.pose_name_list[self.ui.joint_combo.currentIndex()]
            msg.is_trajectory = False
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
