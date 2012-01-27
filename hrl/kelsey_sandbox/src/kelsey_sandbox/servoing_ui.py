# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'servoing_ui.ui'
#
# Created: Fri Jan 27 04:16:54 2012
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_Frame(object):
    def setupUi(self, Frame):
        Frame.setObjectName("Frame")
        Frame.resize(403, 274)
        Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.stop_moving = QtGui.QPushButton(Frame)
        self.stop_moving.setEnabled(True)
        self.stop_moving.setGeometry(QtCore.QRect(30, 20, 141, 131))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(193, 193, 193))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(197, 197, 197))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(163, 163, 163))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 57, 252))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Highlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(193, 193, 193))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(197, 197, 197))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(163, 163, 163))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 57, 252))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Highlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(193, 193, 193))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(18, 33, 126))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(163, 163, 163))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(163, 163, 163))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 57, 252))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Highlight, brush)
        self.stop_moving.setPalette(palette)
        self.stop_moving.setStyleSheet("font: 75 18pt \"AlArabiya\";\n"
"selection-background-color: rgb(255, 57, 252);\n"
"background-color: rgb(255, 0, 0);\n"
"color: rgb(18, 33, 126);")
        self.stop_moving.setObjectName("stop_moving")
        self.find_ar_tag = QtGui.QPushButton(Frame)
        self.find_ar_tag.setGeometry(QtCore.QRect(190, 20, 181, 61))
        self.find_ar_tag.setStyleSheet("font: 75 18pt \"AlArabiya\";\n"
"background-color: rgb(55, 168, 255);")
        self.find_ar_tag.setObjectName("find_ar_tag")
        self.begin_servoing = QtGui.QPushButton(Frame)
        self.begin_servoing.setGeometry(QtCore.QRect(190, 90, 181, 61))
        self.begin_servoing.setStyleSheet("font: 75 18pt \"AlArabiya\";\n"
"background-color: rgb(34, 255, 78);")
        self.begin_servoing.setObjectName("begin_servoing")
        self.state_information = QtGui.QLabel(Frame)
        self.state_information.setGeometry(QtCore.QRect(30, 160, 341, 91))
        self.state_information.setStyleSheet("font: 16pt \"AlArabiya\";")
        self.state_information.setObjectName("state_information")

        self.retranslateUi(Frame)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        Frame.setWindowTitle(QtGui.QApplication.translate("Frame", "Frame", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_moving.setText(QtGui.QApplication.translate("Frame", "Stop\n"
"Moving", None, QtGui.QApplication.UnicodeUTF8))
        self.find_ar_tag.setText(QtGui.QApplication.translate("Frame", "Find AR Tag", None, QtGui.QApplication.UnicodeUTF8))
        self.begin_servoing.setText(QtGui.QApplication.translate("Frame", "Begin Servoing", None, QtGui.QApplication.UnicodeUTF8))
        self.state_information.setText(QtGui.QApplication.translate("Frame", "State Information\n"
"", None, QtGui.QApplication.UnicodeUTF8))

