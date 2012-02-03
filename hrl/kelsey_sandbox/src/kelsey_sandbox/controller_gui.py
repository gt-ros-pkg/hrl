# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controller_gui.ui'
#
# Created: Thu Feb  2 04:31:00 2012
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_Frame(object):
    def setupUi(self, Frame):
        Frame.setObjectName("Frame")
        Frame.resize(609, 289)
        Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.start_button = QtGui.QPushButton(Frame)
        self.start_button.setGeometry(QtCore.QRect(20, 20, 161, 181))
        self.start_button.setStyleSheet("font: 16pt \"AlArabiya\";\n"
"background-color: rgb(120, 255, 96);")
        self.start_button.setObjectName("start_button")
        self.kill_button = QtGui.QPushButton(Frame)
        self.kill_button.setGeometry(QtCore.QRect(210, 20, 161, 181))
        self.kill_button.setStyleSheet("background-color: rgb(255, 67, 67);\n"
"font: 16pt \"AlArabiya\";")
        self.kill_button.setObjectName("kill_button")
        self.controller_combo = QtGui.QComboBox(Frame)
        self.controller_combo.setGeometry(QtCore.QRect(20, 230, 351, 31))
        self.controller_combo.setEditable(True)
        self.controller_combo.setObjectName("controller_combo")
        self.restart_button = QtGui.QPushButton(Frame)
        self.restart_button.setGeometry(QtCore.QRect(400, 20, 161, 181))
        self.restart_button.setStyleSheet("background-color: rgb(85, 170, 255);\n"
"font: 16pt \"AlArabiya\";")
        self.restart_button.setObjectName("restart_button")
        self.arm_combo = QtGui.QComboBox(Frame)
        self.arm_combo.setGeometry(QtCore.QRect(400, 230, 161, 31))
        self.arm_combo.setObjectName("arm_combo")
        self.arm_combo.addItem("")
        self.arm_combo.addItem("")

        self.retranslateUi(Frame)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        Frame.setWindowTitle(QtGui.QApplication.translate("Frame", "Frame", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("Frame", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.kill_button.setText(QtGui.QApplication.translate("Frame", "Kill", None, QtGui.QApplication.UnicodeUTF8))
        self.restart_button.setText(QtGui.QApplication.translate("Frame", "Restart", None, QtGui.QApplication.UnicodeUTF8))
        self.arm_combo.setItemText(0, QtGui.QApplication.translate("Frame", "right", None, QtGui.QApplication.UnicodeUTF8))
        self.arm_combo.setItemText(1, QtGui.QApplication.translate("Frame", "left", None, QtGui.QApplication.UnicodeUTF8))

