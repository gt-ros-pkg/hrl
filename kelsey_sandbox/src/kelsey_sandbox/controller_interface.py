#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic
import roslib
roslib.load_manifest("rospy")
import rospy

from controller_gui import Ui_Frame as QTControllerGUIFrame

from subprocess import Popen

class ControllerGUIFrame(QtGui.QFrame):
    def __init__(self):
        super(ControllerGUIFrame, self).__init__()
        self.init_ui()

    def init_ui(self):
        self.ui = QTControllerGUIFrame()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clk)
        self.ui.kill_button.clicked.connect(self.kill_button_clk)
        self.ui.restart_button.clicked.connect(self.restart_button_clk)
        self.ui.controller_combo.addItem("%s_arm_controller")
        self.ui.controller_combo.addItem("%s_cart")
        self.ui.controller_combo.addItem("%s_joint_controller_low")

    def start_button_clk(self):
        controller = str(self.ui.controller_combo.currentText())
        arm = str(self.ui.arm_combo.currentText())
        ctrl_filled = controller % arm[0]
        Popen("rosrun pr2_controller_manager pr2_controller_manager spawn %s" % ctrl_filled, 
              shell=True)

    def kill_button_clk(self):
        controller = str(self.ui.controller_combo.currentText())
        arm = str(self.ui.arm_combo.currentText())
        ctrl_filled = controller % arm[0]
        Popen("rosrun pr2_controller_manager pr2_controller_manager kill %s" % ctrl_filled, 
              shell=True)

    def restart_button_clk(self):
        controller = str(self.ui.controller_combo.currentText())
        arm = str(self.ui.arm_combo.currentText())
        ctrl_filled = controller % arm[0]
        Popen("rosrun pr2_controller_manager pr2_controller_manager kill %s" % ctrl_filled, 
              shell=True)
        rospy.sleep(0.1)
        Popen("rosrun pr2_controller_manager pr2_controller_manager spawn %s" % ctrl_filled, 
              shell=True)

def main():
    rospy.init_node("controller_gui")
    app = QtGui.QApplication(sys.argv)
    frame = ControllerGUIFrame()
    frame.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
