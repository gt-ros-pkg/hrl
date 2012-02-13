# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arm_pose_move_gui.ui'
#
# Created: Sat Feb 11 21:48:49 2012
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_Frame(object):
    def setupUi(self, Frame):
        Frame.setObjectName("Frame")
        Frame.resize(398, 355)
        Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.start_button = QtGui.QPushButton(Frame)
        self.start_button.setGeometry(QtCore.QRect(20, 10, 161, 181))
        self.start_button.setStyleSheet("font: 16pt \"AlArabiya\";\n"
"background-color: rgb(120, 255, 96);")
        self.start_button.setObjectName("start_button")
        self.kill_button = QtGui.QPushButton(Frame)
        self.kill_button.setGeometry(QtCore.QRect(210, 10, 161, 181))
        self.kill_button.setStyleSheet("background-color: rgb(255, 67, 67);\n"
"font: 16pt \"AlArabiya\";")
        self.kill_button.setObjectName("kill_button")
        self.comboBox = QtGui.QComboBox(Frame)
        self.comboBox.setGeometry(QtCore.QRect(20, 210, 351, 31))
        self.comboBox.setObjectName("comboBox")
        self.state_information = QtGui.QLabel(Frame)
        self.state_information.setGeometry(QtCore.QRect(30, 250, 341, 91))
        self.state_information.setStyleSheet("font: 16pt \"AlArabiya\";")
        self.state_information.setObjectName("state_information")

        self.retranslateUi(Frame)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        Frame.setWindowTitle(QtGui.QApplication.translate("Frame", "Frame", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("Frame", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.kill_button.setText(QtGui.QApplication.translate("Frame", "Pause", None, QtGui.QApplication.UnicodeUTF8))
        self.state_information.setText(QtGui.QApplication.translate("Frame", "State Information\n"
"", None, QtGui.QApplication.UnicodeUTF8))

