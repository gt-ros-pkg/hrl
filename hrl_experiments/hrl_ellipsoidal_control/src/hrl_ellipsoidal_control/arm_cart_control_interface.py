#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic
import numpy as np
import functools

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("std_srvs")
roslib.load_manifest("tf")
import rospy
from std_msgs.msg import String
import tf.transformations as tf_trans
from std_srvs.srv import Empty, EmptyResponse

from arm_cart_control_gui import Ui_Frame as QTArmControlGUIFrame

MOVE_BUTTONS = {'translate_up' : (True, tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]), 
                'translate_down' : (True, tf_trans.euler_matrix(0, np.pi/2, 0)[:3,:3]), 
                'translate_left' : (True, tf_trans.euler_matrix(0, 0, np.pi/2)[:3,:3]), 
                'translate_right' : (True, tf_trans.euler_matrix(0, 0, -np.pi/2)[:3,:3]), 
                'translate_in' : (True, tf_trans.euler_matrix(0, 0, np.pi)[:3,:3]), 
                'translate_out' : (True, tf_trans.euler_matrix(0, 0, 0)[:3,:3]),
                'rotate_x_pos' : (False, tf_trans.euler_matrix(0, 0, 0)[:3,:3]),
                'rotate_x_neg' : (False, tf_trans.euler_matrix(0, 0, np.pi)[:3,:3]),
                'rotate_y_pos' : (False, tf_trans.euler_matrix(0, 0, np.pi/2)[:3,:3]),
                'rotate_y_neg' : (False, tf_trans.euler_matrix(0, 0, -np.pi/2)[:3,:3]),
                'rotate_z_pos' : (False, tf_trans.euler_matrix(0, -np.pi/2, 0)[:3,:3]),
                'rotate_z_neg' : (False, tf_trans.euler_matrix(0, np.pi/2, 0)[:3,:3])}

BUTTON_STYLESHEET = """image: url(:/resources/%s_%s.png);
                       background-image: url(:/resources/empty.png);"""
MONITOR_RATE = 20.

class ArmCartControlGUIFrame(QtGui.QFrame):
    def __init__(self):
        super(ArmCartControlGUIFrame, self).__init__()
        self.enable_buttons = False
        self.disable_buttons = False
        self.set_ctrl_name = None
        self.set_status = None

        self.button_clk_pub = rospy.Publisher("/arm_cart_ctrl_gui/button_clk", String)
        self.buttons_enable_srv = rospy.Service("/arm_cart_ctrl_gui/buttons_enable", Empty, 
                                                self._buttons_enable_cb)
        self.buttons_disable_srv = rospy.Service("/arm_cart_ctrl_gui/buttons_disable", Empty,
                                                 self._buttons_disable_cb)
        self.set_ctrl_name_sub = rospy.Subscriber("/arm_cart_ctrl_gui/set_controller_name", String,
                                                  self._set_ctrl_name_cb)
        self.set_status_sub = rospy.Subscriber("/arm_cart_ctrl_gui/set_status", String,
                                               self._set_status_cb)

        self.init_ui()

    def init_ui(self):
        self.ui = QTArmControlGUIFrame()
        self.ui.setupUi(self)
        self.buttons_enabled()
        for button in MOVE_BUTTONS:
            _publish_button_clk = functools.partial(self._publish_button_clk, button)
            exec("self.ui.%s.clicked.connect(_publish_button_clk)" % button)

        self.monitor_timer = QtCore.QTimer(self)
        QtCore.QObject.connect(self.monitor_timer, QtCore.SIGNAL("timeout()"), self.monitor_cb)
        self.monitor_timer.start(MONITOR_RATE)

    def _buttons_enable_cb(self, req):
        self.enable_buttons = True
        return EmptyResponse()

    def _buttons_disable_cb(self, req):
        self.disable_buttons = True
        return EmptyResponse()

    def _publish_button_clk(self, button):
        self.button_clk_pub.publish(button)

    def _set_ctrl_name_cb(self, msg):
        self.set_ctrl_name = msg.data

    def _set_status_cb(self, msg):
        self.set_status = msg.data

    def monitor_cb(self):
        if self.enable_buttons:
            self.buttons_enabled(True)
            self.enable_buttons = False
        if self.disable_buttons:
            self.buttons_enabled(False)
            self.disable_buttons = False
        if self.set_ctrl_name is not None:
            self.ui.controller_name.setText(self.set_ctrl_name)
            self.set_ctrl_name = None
        if self.set_status is not None:
            self.ui.status_text.setText(self.set_status)
            self.set_status = None

    def buttons_enabled(self, enabled=True):
        for button in MOVE_BUTTONS:
            exec("cur_button = self.ui.%s" % button)
            if enabled:
                cur_button.setEnabled(True)
                cur_button.setStyleSheet(BUTTON_STYLESHEET % (button, 'on'))
            else:
                cur_button.setDisabled(True)
                cur_button.setStyleSheet(BUTTON_STYLESHEET % (button, 'off'))

def main():
    rospy.init_node("arm_cart_control_interface")
    app = QtGui.QApplication(sys.argv)
    frame = ArmCartControlGUIFrame()
    frame.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
