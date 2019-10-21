# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\rover_v3.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(828, 766)
        MainWindow.setAutoFillBackground(False)
        MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("background-color: qconicalgradient(cx:0, cy:0, angle:135, stop:0 rgba(0, 255, 237, 69), stop:0.426471 rgba(0, 214, 138, 69), stop:0.441176 rgba(0, 255, 246, 145), stop:0.534314 rgba(255, 255, 0, 135), stop:0.57754 rgba(0, 201, 255, 130), stop:0.70098 rgba(0, 178, 171, 208), stop:0.779412 rgba(255, 218, 71, 130), stop:1 rgba(0, 237, 255, 69));")
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.scrollArea = QtWidgets.QScrollArea(self.centralwidget)
        self.scrollArea.setStyleSheet("")
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents_2 = QtWidgets.QWidget()
        self.scrollAreaWidgetContents_2.setGeometry(QtCore.QRect(0, 0, 803, 706))
        self.scrollAreaWidgetContents_2.setStyleSheet("background-color:transparent;")
        self.scrollAreaWidgetContents_2.setObjectName("scrollAreaWidgetContents_2")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents_2)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.TopLayout = QtWidgets.QHBoxLayout()
        self.TopLayout.setObjectName("TopLayout")
        self.title = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        font = QtGui.QFont()
        font.setFamily("Cooper Black")
        font.setPointSize(28)
        self.title.setFont(font)
        self.title.setStyleSheet("background-color: rgb(255, 255, 255,80%);")
        self.title.setObjectName("title")
        self.TopLayout.addWidget(self.title)
        self.warring = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        font = QtGui.QFont()
        font.setFamily("3ds")
        font.setPointSize(28)
        self.warring.setFont(font)
        self.warring.setStyleSheet("background-color: rgb(255, 239, 57);\n"
"")
        self.warring.setAlignment(QtCore.Qt.AlignCenter)
        self.warring.setObjectName("warring")
        self.TopLayout.addWidget(self.warring)
        self.OnOffBtnLayout = QtWidgets.QHBoxLayout()
        self.OnOffBtnLayout.setObjectName("OnOffBtnLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.RoverMainOnOffBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.RoverMainOnOffBtn.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.RoverMainOnOffBtn.setObjectName("RoverMainOnOffBtn")
        self.verticalLayout.addWidget(self.RoverMainOnOffBtn)
        self.VisionOnOffBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.VisionOnOffBtn.setStyleSheet("background-color: rgb(255, 3, 3);")
        self.VisionOnOffBtn.setObjectName("VisionOnOffBtn")
        self.verticalLayout.addWidget(self.VisionOnOffBtn)
        self.OnOffBtnLayout.addLayout(self.verticalLayout)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.LidarOnOffBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.LidarOnOffBtn.setStyleSheet("background-color: rgb(255, 3, 3);")
        self.LidarOnOffBtn.setObjectName("LidarOnOffBtn")
        self.verticalLayout_3.addWidget(self.LidarOnOffBtn)
        self.AlgorithmOnOffBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.AlgorithmOnOffBtn.setStyleSheet("background-color: rgb(255, 3, 3);")
        self.AlgorithmOnOffBtn.setObjectName("AlgorithmOnOffBtn")
        self.verticalLayout_3.addWidget(self.AlgorithmOnOffBtn)
        self.OnOffBtnLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.OnOffBtnLayout.addLayout(self.verticalLayout_2)
        self.StopAllBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.StopAllBtn.sizePolicy().hasHeightForWidth())
        self.StopAllBtn.setSizePolicy(sizePolicy)
        self.StopAllBtn.setStyleSheet("background-color: rgb(255, 3, 3);")
        self.StopAllBtn.setObjectName("StopAllBtn")
        self.OnOffBtnLayout.addWidget(self.StopAllBtn)
        self.TopLayout.addLayout(self.OnOffBtnLayout)
        self.verticalLayout_6.addLayout(self.TopLayout)
        self.MiddleLayout = QtWidgets.QHBoxLayout()
        self.MiddleLayout.setObjectName("MiddleLayout")
        self.FunctionBtnLayout = QtWidgets.QVBoxLayout()
        self.FunctionBtnLayout.setObjectName("FunctionBtnLayout")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout()
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_5 = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_9.addWidget(self.label_5)
        self.LidarUSB_text = QtWidgets.QTextEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.LidarUSB_text.sizePolicy().hasHeightForWidth())
        self.LidarUSB_text.setSizePolicy(sizePolicy)
        self.LidarUSB_text.setMaximumSize(QtCore.QSize(180, 50))
        self.LidarUSB_text.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.LidarUSB_text.setObjectName("LidarUSB_text")
        self.horizontalLayout_9.addWidget(self.LidarUSB_text)
        self.verticalLayout_13.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setObjectName("label")
        self.horizontalLayout_5.addWidget(self.label)
        self.VisionStatus_text = QtWidgets.QTextEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.VisionStatus_text.sizePolicy().hasHeightForWidth())
        self.VisionStatus_text.setSizePolicy(sizePolicy)
        self.VisionStatus_text.setMaximumSize(QtCore.QSize(180, 25))
        self.VisionStatus_text.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.VisionStatus_text.setObjectName("VisionStatus_text")
        self.horizontalLayout_5.addWidget(self.VisionStatus_text)
        self.verticalLayout_13.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_6 = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_11.addWidget(self.label_6)
        self.CurrentSpeed_text = QtWidgets.QTextEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CurrentSpeed_text.sizePolicy().hasHeightForWidth())
        self.CurrentSpeed_text.setSizePolicy(sizePolicy)
        self.CurrentSpeed_text.setMaximumSize(QtCore.QSize(180, 25))
        self.CurrentSpeed_text.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.CurrentSpeed_text.setObjectName("CurrentSpeed_text")
        self.horizontalLayout_11.addWidget(self.CurrentSpeed_text)
        self.verticalLayout_13.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_4 = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_4.sizePolicy().hasHeightForWidth())
        self.label_4.setSizePolicy(sizePolicy)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_8.addWidget(self.label_4)
        self.textEdit_2 = QtWidgets.QTextEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textEdit_2.sizePolicy().hasHeightForWidth())
        self.textEdit_2.setSizePolicy(sizePolicy)
        self.textEdit_2.setMaximumSize(QtCore.QSize(180, 25))
        self.textEdit_2.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.textEdit_2.setObjectName("textEdit_2")
        self.horizontalLayout_8.addWidget(self.textEdit_2)
        self.verticalLayout_13.addLayout(self.horizontalLayout_8)
        self.FunctionBtnLayout.addLayout(self.verticalLayout_13)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.verticalLayout_12 = QtWidgets.QVBoxLayout()
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.KeyboardControlBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.KeyboardControlBtn.sizePolicy().hasHeightForWidth())
        self.KeyboardControlBtn.setSizePolicy(sizePolicy)
        self.KeyboardControlBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(11)
        font.setBold(False)
        font.setWeight(50)
        self.KeyboardControlBtn.setFont(font)
        self.KeyboardControlBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.KeyboardControlBtn.setObjectName("KeyboardControlBtn")
        self.verticalLayout_12.addWidget(self.KeyboardControlBtn)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.KeyboardControl_SetSpeedBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.KeyboardControl_SetSpeedBtn.setStyleSheet("background-color: rgb(227, 227, 227);")
        self.KeyboardControl_SetSpeedBtn.setObjectName("KeyboardControl_SetSpeedBtn")
        self.horizontalLayout_10.addWidget(self.KeyboardControl_SetSpeedBtn)
        self.KeyBoardControl_speed = QtWidgets.QSlider(self.scrollAreaWidgetContents_2)
        self.KeyBoardControl_speed.setMaximum(100)
        self.KeyBoardControl_speed.setProperty("value", 1)
        self.KeyBoardControl_speed.setOrientation(QtCore.Qt.Horizontal)
        self.KeyBoardControl_speed.setObjectName("KeyBoardControl_speed")
        self.horizontalLayout_10.addWidget(self.KeyBoardControl_speed)
        self.SetSpeed_label = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        self.SetSpeed_label.setObjectName("SetSpeed_label")
        self.horizontalLayout_10.addWidget(self.SetSpeed_label)
        self.verticalLayout_12.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_7.addLayout(self.verticalLayout_12)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(-1, -1, 0, -1)
        self.gridLayout.setObjectName("gridLayout")
        self.KeyUp = QtWidgets.QGraphicsView(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.KeyUp.sizePolicy().hasHeightForWidth())
        self.KeyUp.setSizePolicy(sizePolicy)
        self.KeyUp.setMaximumSize(QtCore.QSize(20, 20))
        self.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.KeyUp.setObjectName("KeyUp")
        self.gridLayout.addWidget(self.KeyUp, 0, 1, 1, 1)
        self.KeyRight = QtWidgets.QGraphicsView(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.KeyRight.sizePolicy().hasHeightForWidth())
        self.KeyRight.setSizePolicy(sizePolicy)
        self.KeyRight.setMaximumSize(QtCore.QSize(20, 20))
        self.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.KeyRight.setObjectName("KeyRight")
        self.gridLayout.addWidget(self.KeyRight, 1, 2, 1, 1)
        self.KeyDown = QtWidgets.QGraphicsView(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.KeyDown.sizePolicy().hasHeightForWidth())
        self.KeyDown.setSizePolicy(sizePolicy)
        self.KeyDown.setMaximumSize(QtCore.QSize(20, 20))
        self.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.KeyDown.setObjectName("KeyDown")
        self.gridLayout.addWidget(self.KeyDown, 1, 1, 1, 1)
        self.KeyLeft = QtWidgets.QGraphicsView(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.KeyLeft.sizePolicy().hasHeightForWidth())
        self.KeyLeft.setSizePolicy(sizePolicy)
        self.KeyLeft.setMaximumSize(QtCore.QSize(20, 20))
        self.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.KeyLeft.setObjectName("KeyLeft")
        self.gridLayout.addWidget(self.KeyLeft, 1, 0, 1, 1)
        self.horizontalLayout_7.addLayout(self.gridLayout)
        self.FunctionBtnLayout.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.verticalLayout_14 = QtWidgets.QVBoxLayout()
        self.verticalLayout_14.setObjectName("verticalLayout_14")
        self.VisionBuildMapBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.VisionBuildMapBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.VisionBuildMapBtn.setFont(font)
        self.VisionBuildMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.VisionBuildMapBtn.setObjectName("VisionBuildMapBtn")
        self.verticalLayout_14.addWidget(self.VisionBuildMapBtn)
        self.pushButton = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.pushButton.setStyleSheet("background-color: rgb(220, 220, 220);")
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout_14.addWidget(self.pushButton)
        self.horizontalLayout_6.addLayout(self.verticalLayout_14)
        self.VisionBuildMap = QtWidgets.QVBoxLayout()
        self.VisionBuildMap.setObjectName("VisionBuildMap")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setStyleSheet("background-color: rgb(255, 255, 255,80%);")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_3.addWidget(self.label_2)
        self.BuildMapID = QtWidgets.QSpinBox(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.BuildMapID.sizePolicy().hasHeightForWidth())
        self.BuildMapID.setSizePolicy(sizePolicy)
        self.BuildMapID.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.BuildMapID.setProperty("value", 1)
        self.BuildMapID.setObjectName("BuildMapID")
        self.horizontalLayout_3.addWidget(self.BuildMapID)
        self.VisionBuildMap.addLayout(self.horizontalLayout_3)
        self.VisionBuildMapStopBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.VisionBuildMapStopBtn.sizePolicy().hasHeightForWidth())
        self.VisionBuildMapStopBtn.setSizePolicy(sizePolicy)
        self.VisionBuildMapStopBtn.setStyleSheet("background-color: rgb(220, 220, 220);")
        self.VisionBuildMapStopBtn.setObjectName("VisionBuildMapStopBtn")
        self.VisionBuildMap.addWidget(self.VisionBuildMapStopBtn)
        self.horizontalLayout_6.addLayout(self.VisionBuildMap)
        self.FunctionBtnLayout.addLayout(self.horizontalLayout_6)
        self.VisionUseMap = QtWidgets.QHBoxLayout()
        self.VisionUseMap.setObjectName("VisionUseMap")
        self.VisionUseMapBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.VisionUseMapBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.VisionUseMapBtn.setFont(font)
        self.VisionUseMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.VisionUseMapBtn.setObjectName("VisionUseMapBtn")
        self.VisionUseMap.addWidget(self.VisionUseMapBtn)
        self.verticalLayout_10 = QtWidgets.QVBoxLayout()
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_3 = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        self.label_3.setStyleSheet("background-color: rgb(255, 255, 255,80%);")
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_4.addWidget(self.label_3)
        self.UseMapID = QtWidgets.QSpinBox(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.UseMapID.sizePolicy().hasHeightForWidth())
        self.UseMapID.setSizePolicy(sizePolicy)
        self.UseMapID.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.UseMapID.setProperty("value", 1)
        self.UseMapID.setObjectName("UseMapID")
        self.horizontalLayout_4.addWidget(self.UseMapID)
        self.verticalLayout_10.addLayout(self.horizontalLayout_4)
        self.VisionUseMapStopBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.VisionUseMapStopBtn.setStyleSheet("background-color: rgb(221, 221, 221);")
        self.VisionUseMapStopBtn.setObjectName("VisionUseMapStopBtn")
        self.verticalLayout_10.addWidget(self.VisionUseMapStopBtn)
        self.VisionUseMap.addLayout(self.verticalLayout_10)
        self.FunctionBtnLayout.addLayout(self.VisionUseMap)
        self.WayPoints = QtWidgets.QHBoxLayout()
        self.WayPoints.setObjectName("WayPoints")
        self.WayPointsBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.WayPointsBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.WayPointsBtn.setFont(font)
        self.WayPointsBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.WayPointsBtn.setObjectName("WayPointsBtn")
        self.WayPoints.addWidget(self.WayPointsBtn)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.AddWayPointBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.AddWayPointBtn.sizePolicy().hasHeightForWidth())
        self.AddWayPointBtn.setSizePolicy(sizePolicy)
        self.AddWayPointBtn.setStyleSheet("background-color: rgb(227, 227, 227);\n"
"border-color: rgb(0, 0, 0);")
        self.AddWayPointBtn.setObjectName("AddWayPointBtn")
        self.horizontalLayout.addWidget(self.AddWayPointBtn)
        self.DeleteWayPointBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.DeleteWayPointBtn.sizePolicy().hasHeightForWidth())
        self.DeleteWayPointBtn.setSizePolicy(sizePolicy)
        self.DeleteWayPointBtn.setStyleSheet("background-color: rgb(227, 227, 227);\n"
"border-color: rgb(0, 0, 0);")
        self.DeleteWayPointBtn.setObjectName("DeleteWayPointBtn")
        self.horizontalLayout.addWidget(self.DeleteWayPointBtn)
        self.verticalLayout_8.addLayout(self.horizontalLayout)
        self.WayPointList = QtWidgets.QLineEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.WayPointList.sizePolicy().hasHeightForWidth())
        self.WayPointList.setSizePolicy(sizePolicy)
        self.WayPointList.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.WayPointList.setObjectName("WayPointList")
        self.verticalLayout_8.addWidget(self.WayPointList)
        self.WayPoints.addLayout(self.verticalLayout_8)
        self.FunctionBtnLayout.addLayout(self.WayPoints)
        self.SaveMap = QtWidgets.QHBoxLayout()
        self.SaveMap.setObjectName("SaveMap")
        self.SaveMapBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.SaveMapBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.SaveMapBtn.setFont(font)
        self.SaveMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.SaveMapBtn.setObjectName("SaveMapBtn")
        self.SaveMap.addWidget(self.SaveMapBtn)
        self.verticalLayout_11 = QtWidgets.QVBoxLayout()
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.lineEdit = QtWidgets.QLineEdit(self.scrollAreaWidgetContents_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit.sizePolicy().hasHeightForWidth())
        self.lineEdit.setSizePolicy(sizePolicy)
        self.lineEdit.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.lineEdit.setObjectName("lineEdit")
        self.verticalLayout_11.addWidget(self.lineEdit)
        self.SaveMap.addLayout(self.verticalLayout_11)
        self.FunctionBtnLayout.addLayout(self.SaveMap)
        self.ShowMapLayout = QtWidgets.QVBoxLayout()
        self.ShowMapLayout.setObjectName("ShowMapLayout")
        self.ShowMapBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.ShowMapBtn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        self.ShowMapBtn.setFont(font)
        self.ShowMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);\n"
"\n"
"\n"
"")
        self.ShowMapBtn.setObjectName("ShowMapBtn")
        self.ShowMapLayout.addWidget(self.ShowMapBtn)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.ShowMap_AddBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.ShowMap_AddBtn.setStyleSheet("background-color: rgb(229, 229, 229);")
        self.ShowMap_AddBtn.setObjectName("ShowMap_AddBtn")
        self.horizontalLayout_2.addWidget(self.ShowMap_AddBtn)
        self.ShowMap_DelBtn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.ShowMap_DelBtn.setStyleSheet("background-color: rgb(229, 229, 229);")
        self.ShowMap_DelBtn.setObjectName("ShowMap_DelBtn")
        self.horizontalLayout_2.addWidget(self.ShowMap_DelBtn)
        self.ShowMapLayout.addLayout(self.horizontalLayout_2)
        self.FunctionBtnLayout.addLayout(self.ShowMapLayout)
        self.Btnnnnn = QtWidgets.QPushButton(self.scrollAreaWidgetContents_2)
        self.Btnnnnn.setMinimumSize(QtCore.QSize(0, 30))
        font = QtGui.QFont()
        font.setFamily("Algerian")
        font.setPointSize(12)
        font.setBold(False)
        font.setWeight(50)
        self.Btnnnnn.setFont(font)
        self.Btnnnnn.setStyleSheet("background-color: rgb(112, 155, 255);")
        self.Btnnnnn.setObjectName("Btnnnnn")
        self.FunctionBtnLayout.addWidget(self.Btnnnnn)
        self.MiddleLayout.addLayout(self.FunctionBtnLayout)
        self.TabLayout = QtWidgets.QHBoxLayout()
        self.TabLayout.setObjectName("TabLayout")
        self.tabWidget = QtWidgets.QTabWidget(self.scrollAreaWidgetContents_2)
        self.tabWidget.setStyleSheet("background-color:transparent;\n"
"border-color: rgb(255, 0, 0);")
        self.tabWidget.setObjectName("tabWidget")
        self.Console = QtWidgets.QWidget()
        self.Console.setObjectName("Console")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.Console)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.console_1 = QtWidgets.QTextEdit(self.Console)
        self.console_1.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"")
        self.console_1.setObjectName("console_1")
        self.verticalLayout_7.addWidget(self.console_1)
        self.tabWidget.addTab(self.Console, "")
        self.lidarMap = QtWidgets.QWidget()
        self.lidarMap.setObjectName("lidarMap")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.lidarMap)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.LidarMap = LidarMap(self.lidarMap)
        self.LidarMap.setObjectName("LidarMap")
        self.verticalLayout_5.addWidget(self.LidarMap)
        self.tabWidget.addTab(self.lidarMap, "")
        self.globalMap = QtWidgets.QWidget()
        self.globalMap.setObjectName("globalMap")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.globalMap)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.GlobalMap = GlobalMap(self.globalMap)
        self.GlobalMap.setObjectName("GlobalMap")
        self.verticalLayout_9.addWidget(self.GlobalMap)
        self.tabWidget.addTab(self.globalMap, "")
        self.TabLayout.addWidget(self.tabWidget)
        self.MiddleLayout.addLayout(self.TabLayout)
        self.verticalLayout_6.addLayout(self.MiddleLayout)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents_2)
        self.verticalLayout_4.addWidget(self.scrollArea)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 828, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.title.setText(_translate("MainWindow", "ROVER HMI"))
        self.warring.setText(_translate("MainWindow", "Warring"))
        self.RoverMainOnOffBtn.setText(_translate("MainWindow", "Main"))
        self.VisionOnOffBtn.setText(_translate("MainWindow", "Vision"))
        self.LidarOnOffBtn.setText(_translate("MainWindow", "Lidar"))
        self.AlgorithmOnOffBtn.setText(_translate("MainWindow", "Algorithm"))
        self.StopAllBtn.setText(_translate("MainWindow", "Stop All"))
        self.label_5.setText(_translate("MainWindow", "LiDAR USB"))
        self.label.setText(_translate("MainWindow", "Vision Status"))
        self.label_6.setText(_translate("MainWindow", "Current Speed(pwm)"))
        self.label_4.setText(_translate("MainWindow", "Future"))
        self.KeyboardControlBtn.setText(_translate("MainWindow", "Keyboard Control (WASD)"))
        self.KeyboardControl_SetSpeedBtn.setText(_translate("MainWindow", "Set Speed(0~100)"))
        self.SetSpeed_label.setText(_translate("MainWindow", "0"))
        self.VisionBuildMapBtn.setText(_translate("MainWindow", "Vision Build Map"))
        self.pushButton.setText(_translate("MainWindow", "Show Map List"))
        self.label_2.setText(_translate("MainWindow", "MapID"))
        self.VisionBuildMapStopBtn.setText(_translate("MainWindow", "Build Map Stop"))
        self.VisionUseMapBtn.setText(_translate("MainWindow", "Vision Use Map"))
        self.label_3.setText(_translate("MainWindow", "MapID"))
        self.VisionUseMapStopBtn.setText(_translate("MainWindow", "Use Map Stop"))
        self.WayPointsBtn.setText(_translate("MainWindow", "WayPoints"))
        self.AddWayPointBtn.setText(_translate("MainWindow", "Add"))
        self.DeleteWayPointBtn.setText(_translate("MainWindow", "Delete"))
        self.SaveMapBtn.setText(_translate("MainWindow", "Save Map"))
        self.ShowMapBtn.setText(_translate("MainWindow", "Show data and build Map"))
        self.ShowMap_AddBtn.setText(_translate("MainWindow", "Add current"))
        self.ShowMap_DelBtn.setText(_translate("MainWindow", "Delete last"))
        self.Btnnnnn.setText(_translate("MainWindow", "Future Work"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Console), _translate("MainWindow", "Console"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.lidarMap), _translate("MainWindow", "LiDAR"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.globalMap), _translate("MainWindow", "Vision and LiDAR Map"))

from globalmap import GlobalMap
from lidarmap import LidarMap
