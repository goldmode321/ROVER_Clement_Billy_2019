# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\rover_calibration.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(859, 748)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scrollArea = QtWidgets.QScrollArea(self.centralwidget)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 814, 714))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(20)
        self.label.setFont(font)
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_4.addWidget(self.label)
        self.VisionData_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.VisionData_label.setObjectName("VisionData_label")
        self.verticalLayout_4.addWidget(self.VisionData_label)
        self.Calibration = Calibration(self.scrollAreaWidgetContents)
        self.Calibration.setMinimumSize(QtCore.QSize(500, 400))
        self.Calibration.setObjectName("Calibration")
        self.verticalLayout_4.addWidget(self.Calibration)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.XCalibration_constant_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.XCalibration_constant_label.setFont(font)
        self.XCalibration_constant_label.setAlignment(QtCore.Qt.AlignCenter)
        self.XCalibration_constant_label.setObjectName("XCalibration_constant_label")
        self.verticalLayout_8.addWidget(self.XCalibration_constant_label)
        self.XCalibration_constant_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.XCalibration_constant_slider.setMinimum(-100)
        self.XCalibration_constant_slider.setMaximum(100)
        self.XCalibration_constant_slider.setSingleStep(1)
        self.XCalibration_constant_slider.setProperty("value", 0)
        self.XCalibration_constant_slider.setOrientation(QtCore.Qt.Horizontal)
        self.XCalibration_constant_slider.setObjectName("XCalibration_constant_slider")
        self.verticalLayout_8.addWidget(self.XCalibration_constant_slider)
        self.XCalibration_constant_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.XCalibration_constant_spinBox.setMinimum(-100)
        self.XCalibration_constant_spinBox.setMaximum(100)
        self.XCalibration_constant_spinBox.setObjectName("XCalibration_constant_spinBox")
        self.verticalLayout_8.addWidget(self.XCalibration_constant_spinBox)
        self.horizontalLayout_3.addLayout(self.verticalLayout_8)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.YCalibration_constant_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.YCalibration_constant_label.setFont(font)
        self.YCalibration_constant_label.setAlignment(QtCore.Qt.AlignCenter)
        self.YCalibration_constant_label.setObjectName("YCalibration_constant_label")
        self.verticalLayout_7.addWidget(self.YCalibration_constant_label)
        self.YCalibration_constant_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.YCalibration_constant_slider.setStyleSheet("border-color: rgb(0, 0, 0);")
        self.YCalibration_constant_slider.setMinimum(-100)
        self.YCalibration_constant_slider.setMaximum(100)
        self.YCalibration_constant_slider.setProperty("value", 0)
        self.YCalibration_constant_slider.setOrientation(QtCore.Qt.Horizontal)
        self.YCalibration_constant_slider.setInvertedAppearance(False)
        self.YCalibration_constant_slider.setObjectName("YCalibration_constant_slider")
        self.verticalLayout_7.addWidget(self.YCalibration_constant_slider)
        self.YCalibration_constant_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.YCalibration_constant_spinBox.setMinimum(-100)
        self.YCalibration_constant_spinBox.setMaximum(100)
        self.YCalibration_constant_spinBox.setObjectName("YCalibration_constant_spinBox")
        self.verticalLayout_7.addWidget(self.YCalibration_constant_spinBox)
        self.horizontalLayout_3.addLayout(self.verticalLayout_7)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.AngleCalibration_constant_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.AngleCalibration_constant_label.setFont(font)
        self.AngleCalibration_constant_label.setAlignment(QtCore.Qt.AlignCenter)
        self.AngleCalibration_constant_label.setObjectName("AngleCalibration_constant_label")
        self.verticalLayout_6.addWidget(self.AngleCalibration_constant_label)
        self.AngleCalibration_constant_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.AngleCalibration_constant_slider.setMinimum(-30)
        self.AngleCalibration_constant_slider.setMaximum(30)
        self.AngleCalibration_constant_slider.setProperty("value", 0)
        self.AngleCalibration_constant_slider.setOrientation(QtCore.Qt.Horizontal)
        self.AngleCalibration_constant_slider.setObjectName("AngleCalibration_constant_slider")
        self.verticalLayout_6.addWidget(self.AngleCalibration_constant_slider)
        self.AngleCalibration_constant_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.AngleCalibration_constant_spinBox.setMinimum(-30)
        self.AngleCalibration_constant_spinBox.setMaximum(30)
        self.AngleCalibration_constant_spinBox.setObjectName("AngleCalibration_constant_spinBox")
        self.verticalLayout_6.addWidget(self.AngleCalibration_constant_spinBox)
        self.horizontalLayout_3.addLayout(self.verticalLayout_6)
        self.verticalLayout_4.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.XCalibration_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.XCalibration_label.setFont(font)
        self.XCalibration_label.setAlignment(QtCore.Qt.AlignCenter)
        self.XCalibration_label.setObjectName("XCalibration_label")
        self.verticalLayout_2.addWidget(self.XCalibration_label)
        self.XCalibration_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.XCalibration_slider.setMinimum(1)
        self.XCalibration_slider.setMaximum(300)
        self.XCalibration_slider.setSingleStep(1)
        self.XCalibration_slider.setProperty("value", 100)
        self.XCalibration_slider.setOrientation(QtCore.Qt.Horizontal)
        self.XCalibration_slider.setObjectName("XCalibration_slider")
        self.verticalLayout_2.addWidget(self.XCalibration_slider)
        self.XCalibration_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.XCalibration_spinBox.setMinimum(1)
        self.XCalibration_spinBox.setMaximum(300)
        self.XCalibration_spinBox.setProperty("value", 100)
        self.XCalibration_spinBox.setObjectName("XCalibration_spinBox")
        self.verticalLayout_2.addWidget(self.XCalibration_spinBox)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.YCalibration_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.YCalibration_label.setFont(font)
        self.YCalibration_label.setAlignment(QtCore.Qt.AlignCenter)
        self.YCalibration_label.setObjectName("YCalibration_label")
        self.verticalLayout_3.addWidget(self.YCalibration_label)
        self.YCalibration_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.YCalibration_slider.setStyleSheet("border-color: rgb(0, 0, 0);")
        self.YCalibration_slider.setMinimum(1)
        self.YCalibration_slider.setMaximum(300)
        self.YCalibration_slider.setProperty("value", 100)
        self.YCalibration_slider.setOrientation(QtCore.Qt.Horizontal)
        self.YCalibration_slider.setInvertedAppearance(False)
        self.YCalibration_slider.setObjectName("YCalibration_slider")
        self.verticalLayout_3.addWidget(self.YCalibration_slider)
        self.YCalibration_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.YCalibration_spinBox.setMinimum(1)
        self.YCalibration_spinBox.setMaximum(300)
        self.YCalibration_spinBox.setProperty("value", 100)
        self.YCalibration_spinBox.setObjectName("YCalibration_spinBox")
        self.verticalLayout_3.addWidget(self.YCalibration_spinBox)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.AngleCalibration_label = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("Bahnschrift Light")
        font.setPointSize(12)
        self.AngleCalibration_label.setFont(font)
        self.AngleCalibration_label.setAlignment(QtCore.Qt.AlignCenter)
        self.AngleCalibration_label.setObjectName("AngleCalibration_label")
        self.verticalLayout_5.addWidget(self.AngleCalibration_label)
        self.AngleCalibration_slider = QtWidgets.QSlider(self.scrollAreaWidgetContents)
        self.AngleCalibration_slider.setMinimum(1)
        self.AngleCalibration_slider.setMaximum(300)
        self.AngleCalibration_slider.setProperty("value", 100)
        self.AngleCalibration_slider.setOrientation(QtCore.Qt.Horizontal)
        self.AngleCalibration_slider.setObjectName("AngleCalibration_slider")
        self.verticalLayout_5.addWidget(self.AngleCalibration_slider)
        self.AngleCalibration_spinBox = QtWidgets.QSpinBox(self.scrollAreaWidgetContents)
        self.AngleCalibration_spinBox.setMinimum(1)
        self.AngleCalibration_spinBox.setMaximum(300)
        self.AngleCalibration_spinBox.setProperty("value", 100)
        self.AngleCalibration_spinBox.setObjectName("AngleCalibration_spinBox")
        self.verticalLayout_5.addWidget(self.AngleCalibration_spinBox)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.ResetBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(12)
        self.ResetBtn.setFont(font)
        self.ResetBtn.setObjectName("ResetBtn")
        self.horizontalLayout.addWidget(self.ResetBtn)
        self.ExportBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(12)
        self.ExportBtn.setFont(font)
        self.ExportBtn.setObjectName("ExportBtn")
        self.horizontalLayout.addWidget(self.ExportBtn)
        self.ImportBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(12)
        self.ImportBtn.setFont(font)
        self.ImportBtn.setObjectName("ImportBtn")
        self.horizontalLayout.addWidget(self.ImportBtn)
        self.ConfirmBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(12)
        self.ConfirmBtn.setFont(font)
        self.ConfirmBtn.setObjectName("ConfirmBtn")
        self.horizontalLayout.addWidget(self.ConfirmBtn)
        self.ExitBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(12)
        self.ExitBtn.setFont(font)
        self.ExitBtn.setObjectName("ExitBtn")
        self.horizontalLayout.addWidget(self.ExitBtn)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout.addWidget(self.scrollArea)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 859, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Calibration window"))
        self.VisionData_label.setText(_translate("MainWindow", "TextLabel"))
        self.XCalibration_constant_label.setText(_translate("MainWindow", "X Calibration : "))
        self.YCalibration_constant_label.setText(_translate("MainWindow", "Y Calibration  : "))
        self.AngleCalibration_constant_label.setText(_translate("MainWindow", "Angle Calibration : "))
        self.XCalibration_label.setText(_translate("MainWindow", "X Calibration multiple : "))
        self.YCalibration_label.setText(_translate("MainWindow", "Y Calibration multiple : "))
        self.AngleCalibration_label.setText(_translate("MainWindow", "Angle Calibration multiple : "))
        self.ResetBtn.setText(_translate("MainWindow", "Reset"))
        self.ExportBtn.setText(_translate("MainWindow", "Export"))
        self.ImportBtn.setText(_translate("MainWindow", "Import"))
        self.ConfirmBtn.setText(_translate("MainWindow", "Confirm"))
        self.ExitBtn.setText(_translate("MainWindow", "Exit"))

from calibration import Calibration
