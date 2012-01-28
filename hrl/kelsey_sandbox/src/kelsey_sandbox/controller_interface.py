#! /usr/bin/python

import sys
from PyQt4 import QtCore, QtGui, uic

from controller_gui import Ui_Frame as QTControllerGUIFrame

from subprocess import Popen

class ControllerGUIFrame(QtGui.QFrame):
    def __init__(self, controller):
        super(ControllerGUIFrame, self).__init__()
        self.init_ui()
        self.controller = controller

    def init_ui(self):
        self.ui = QTControllerGUIFrame()
        self.ui.setupUi(self)

        self.ui.start_button.clicked.connect(self.start_button_clk)
        self.ui.kill_button.clicked.connect(self.kill_button_clk)

    def start_button_clk(self):
        Popen("rosrun pr2_controller_manager pr2_controller_manager start %s" % self.controller, shell=True)

    def kill_button_clk(self):
        Popen("rosrun pr2_controller_manager pr2_controller_manager stop %s" % self.controller, shell=True)

def main():
    app = QtGui.QApplication(sys.argv)
    assert(len(sys.argv) > 1)
    controller = sys.argv[1]
    frame = ControllerGUIFrame(controller)
    frame.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
