#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
import rospy
from std_msgs.msg import String

from arm_cart_control_gui import Ui_Frame as QTArmControlGUIFrame

MOVE_BUTTONS = {'translate_up' : [0, 1], 'translate_down' : [0, -1],
                'translate_left' : [1, 1], 'translate_right' : [1, -1],
                'translate_in' : [2, 1], 'translate_out' : [2, -1],
                'rotate_x_pos' : [3, 1], 'rotate_x_neg' : [3, -1],
                'rotate_y_pos' : [4, 1], 'rotate_y_neg' : [4, -1],
                'rotate_z_pos' : [5, 1], 'rotate_z_neg' : [5, -1]}

POSE_PARAMS = ['position.x', 'position.y', 'position.z', 
               'orientation.x', 'orientation.y', 'orientation.z']

ARM_STYLESHEET = """image: url(:/icons/arm_%s_%s.png);
                    background-image: url(:/icons/empty.png);"""

class ArmCartControlGUIFrame(QtGui.QFrame):
    def __init__(self):
        super(ArmCartControlGUIFrame, self).__init__()
        self.has_set_arm = False

        self.move_state_pub = rospy.Publisher("/arm_control_gui/move_state", String)
        self.arm_state_pub = rospy.Publisher("/arm_control_gui/load_arm", String)

        self.init_ui()

        #axis, direction = button_data
        #        self.button_state = PoseStamped()
        #        exec('self.cmd_msg.pose.%s = %f' % (POSE_PARAMS[], ))

    def init_ui(self):
        self.ui = QTArmControlGUIFrame()
        self.ui.setupUi(self)

        self.ui.arm_left.clicked.connect(self.arm_left_clk)
        self.ui.arm_right.clicked.connect(self.arm_right_clk)
        self.monitor_timer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.monitor_timer, QtCore.SIGNAL("timeout()"), self.monitor_cb)
        self.monitor_timer.start(50)

    def monitor_cb(self):
        if not self.has_set_arm:
            return
        move_button = ""
        for button in MOVE_BUTTONS:
            exec("is_down = self.ui.%s.isDown()" % button)
            if is_down:
                move_button = button
        self.move_state_pub.publish(String(move_button))

    def arm_left_clk(self):
        self.arm_state_pub.publish(String("l"))
        self.ui.arm_left.setDisabled(True)
        self.ui.arm_right.setEnabled(True)
        self.ui.arm_left.setStyleSheet(ARM_STYLESHEET % ("left", "on"))
        self.ui.arm_right.setStyleSheet(ARM_STYLESHEET % ("right", "off"))
        self.has_set_arm = True

    def arm_right_clk(self):
        self.arm_state_pub.publish(String("r"))
        self.ui.arm_right.setDisabled(True)
        self.ui.arm_left.setEnabled(True)
        self.ui.arm_left.setStyleSheet(ARM_STYLESHEET % ("left", "off"))
        self.ui.arm_right.setStyleSheet(ARM_STYLESHEET % ("right", "on"))
        self.has_set_arm = True

def main():
    rospy.init_node("arm_cart_control_interface")
    app = QtGui.QApplication(sys.argv)
    frame = ArmCartControlGUIFrame()
    frame.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
