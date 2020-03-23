import math
import multiprocessing
import os
import queue
import sys
import threading
import time
import traceback

import keyboard
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph


import gui.rover_ui_file as GUI
import gui.rover_calibration as C_GUI
import rover_socket
import rover_shared_variable

class Pens:
    '''pyqtgraph Pen'''
    def __init__(self):
        self.blue_point_pen = pyqtgraph.mkPen(color=(0, 0, 255), width=2)
        self.transparent_pen = pyqtgraph.mkPen(color=(255, 255, 255, 0))
        self.route_pen = pyqtgraph.mkPen(color=(255, 0, 0), width=2, style=QtCore.Qt.DotLine)
        self.fitted_route_pen = pyqtgraph.mkPen(color=(0, 0, 255), width=3)

        self.dict = {
            "trans":self.transparent_pen,
            "blue":self.blue_point_pen,
            "route":self.route_pen,
            "froute":self.fitted_route_pen,
            "or": pyqtgraph.mkPen(color=(250, 100, 0),width=5),
            "db": pyqtgraph.mkPen(color=(0, 0, 100),width=5),
            "b": pyqtgraph.mkPen(color=(0, 0, 250),width=5),
            "g": pyqtgraph.mkPen(color=(0, 255, 0), width=5),
        }

class Arrow:
    def __init__(self, x=0, y=0, length=90, side_len=30, angle=0, color="or"):
        '''
        color = ['or', 'db]
        angle = radians
                  \  
                   \  side length
        length      \  
        -------------+
                    /
                   /
                  /
        '''
        self.Pens = Pens()
        self.side_angle = 50
        self.length = length
        self.side_len = side_len
        self.updatePos(x, y, angle)

        self.arror_plot = pyqtgraph.PlotDataItem(
            self.arror_pos_x, 
            self.arror_pos_y, 
            pen=self.Pens.dict["db"],
        )

    def updatePos(self, x, y, angle):
        self.angle = angle
        self.x = x
        self.y = y
        self.tail_x = self.x + self.length*np.cos(self.angle + np.pi)
        self.tail_y = self.y + self.length*np.sin(self.angle + np.pi)
        self.side_up_x = self.x + self.side_len*np.cos(np.pi - np.radians(self.side_angle) + self.angle)
        self.side_up_y = self.y + self.side_len*np.sin(np.pi - np.radians(self.side_angle) + self.angle)
        self.side_down_x = self.x + self.side_len*np.cos(np.pi + np.radians(self.side_angle) + self.angle)
        self.side_down_y = self.y + self.side_len*np.sin(np.pi + np.radians(self.side_angle) + self.angle)

        self.arror_pos_x = [self.side_up_x, self.x, self.tail_x, self.x, self.side_down_x]
        self.arror_pos_y = [self.side_up_y, self.y, self.tail_y, self.y, self.side_down_y]

    def updatePos2(self, x, y, angle):
        self.angle = angle
        self.tail_x = x
        self.tail_y = y
        self.x = self.tail_x + self.length*np.cos(self.angle)
        self.y = self.tail_y + self.length*np.sin(self.angle)
        self.side_up_x = self.x + self.side_len*np.cos(np.pi - np.radians(self.side_angle) + self.angle)
        self.side_up_y = self.y + self.side_len*np.sin(np.pi - np.radians(self.side_angle) + self.angle)
        self.side_down_x = self.x + self.side_len*np.cos(np.pi + np.radians(self.side_angle) + self.angle)
        self.side_down_y = self.y + self.side_len*np.sin(np.pi + np.radians(self.side_angle) + self.angle)

        self.arror_pos_x = [self.side_up_x, self.x, self.tail_x, self.x, self.side_down_x]
        self.arror_pos_y = [self.side_up_y, self.y, self.tail_y, self.y, self.side_down_y]


    def setPos(self, x, y, angle):
        '''Point at head'''
        self.updatePos(x, y, angle)
        self.arror_plot.setData(self.arror_pos_x, self.arror_pos_y)

    def setPos2(self, x, y, angle):
        '''Point at tail'''
        self.updatePos2(x, y, angle)
        self.arror_plot.setData(self.arror_pos_x, self.arror_pos_y)
    
    def setStyle(self, length=90, side_length=30):
        self.length = length
        self.side_length = side_length



class ROVER_gui():
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        self.MainWindow = QtWidgets.QMainWindow()

        self.gui = GUI.Ui_MainWindow()
        self.gui.setupUi(self.MainWindow)
        self.pen = Pens()

        self.SV = rover_shared_variable.SharedVariables()
        self.ROV = self.SV.ROV
        self.VI = self.SV.VI
        self.LI = self.SV.LI
        self.MAP = self.SV.MAP
        self.CAL = self.SV.CAL
        self.CC = self.SV.CC
        self.LOBS = self.SV.LOBS
        self.GOBS = self.SV.GOBS
        self.AS = self.SV.AS
        self.CF = self.SV.CF
        self.PT = self.SV.PT
        self.GUI = self.SV.GUI

        self.GUI.gui = self.gui


        self.current_speed = 0
        self.show_vision_status_dict = {0:"Booting", 1:"Waiting for command", 2:"Loading data",
            3:":Locating, please move around", 4:":Working normally",
            5:":Lost current position"}


        self.animation_run, \
        self.GUI.calibration_run, self.keyboard_control_run \
            = False, False, False

        self.check_status_rover_run_list = [
            self.ROV.rover_run, self.VI.vision_run, self.LI.lidar_run
        ]

        self.check_status_rover_Btn_list = [
            self.gui.RoverMainOnOffBtn,
            self.gui.VisionOnOffBtn,
            self.gui.LidarOnOffBtn
        ]

        self.check_status_func_run_list = [
            self.animation_run,
            self.VI.vision_build_map_mode, 
            self.VI.vision_use_map_mode,
            self.GUI.calibration_run,
            self.keyboard_control_run
        ]

        self.check_status_func_Btn_list = [
            self.gui.ShowMapBtn,
            self.gui.VisionBuildMapBtn,
            self.gui.VisionUseMapBtn,
            self.gui.CalibrationBtn,
            self.gui.KeyboardControlBtn
        ]


        ### Lidar Plot
        self.lidar_plot_widget = pyqtgraph.PlotWidget(background='w')
        self.lidar_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size = 10,
            brush=(0, 0, 255),
            pen=self.pen.dict["trans"]
        )
        self.lidar_plot_widget.setXRange(-4000, 4000)
        self.lidar_plot_widget.setYRange(-4000, 4000)
        self.lidar_plot_widget.addItem(self.lidar_plot)
        self.gui.verticalLayout_17.addWidget(self.lidar_plot_widget)

        ### Plot Global Map
        self.global_plot_widget = pyqtgraph.PlotWidget(background='w')
        self.gobs_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size = 10,
            brush=(0, 0, 0),
            pen=self.pen.dict["trans"]
        )
        self.lobs_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size = 10,
            brush=(0, 0, 255),
            pen=self.pen.dict["trans"]
        )
        self.end_point_plot = pyqtgraph.ScatterPlotItem(
            symbol='x',
            size=20,
            brush=(255, 0, 0)
        )
        self.rover_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.rover_size,
            size=20,
            brush=(0, 255 ,0)
        )
        self.forward_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            # size=self.SV.AS.obstacle_size,
            size = 20,
            brush=(0, 200, 0),
            pen=self.pen.dict["trans"],
        )
        self.backward_plot = pyqtgraph.ScatterPlotItem(
            symbol='o',
            size = 20,
            brush=(200, 0, 0),
            pen=self.pen.dict["trans"],
        )
        self.route_plot = self.global_plot_widget.plot(pen=self.pen.dict["route"])
        self.fitted_route_plot = self.global_plot_widget.plot(pen=self.pen.dict["froute"])
        self.global_plot_widget.addItem(self.gobs_plot)
        self.global_plot_widget.addItem(self.lobs_plot)
        self.global_plot_widget.addItem(self.end_point_plot)
        self.global_plot_widget.addItem(self.rover_plot)
        self.global_plot_widget.addItem(self.forward_plot)
        self.global_plot_widget.addItem(self.backward_plot)
        self.gui.verticalLayout_19.addWidget(self.global_plot_widget)

        self.gui.StopAllBtn.clicked.connect(self.StopAllBtn_click)
        self.gui.KeyboardControlBtn.clicked.connect(self.KeyboardControlBtn_click)
        self.gui.VisionBuildMapBtn.clicked.connect(self.VisionBuildMapBtn_click)
        self.gui.VisionBuildMapStopBtn.clicked.connect(self.VisionBuildMapStopBtn_click)
        self.gui.VisionUseMapBtn.clicked.connect(self.VisionUseMapBtn_click)
        self.gui.VisionUseMapStopBtn.clicked.connect(self.VisionUseMapStopBtn_click)
        self.gui.ShowMapBtn.clicked.connect(self.showMap)
        self.gui.ShowMap_AddBtn.clicked.connect(self.ShowMap_AddBtn_click)
        self.gui.KeyboardControl_SetSpeedBtn.clicked.connect(self.KeyboardControl_SetSpeedBtn_click)
        self.gui.KeyBoardControl_speed.valueChanged.connect(self.KeyBoardControl_speed_value_change)
        self.gui.CalibrationBtn.clicked.connect(self.calibration)
        self.gui.SaveMapBtn.clicked.connect(self.SaveMapBtn_click)


        self.gui_test_connection_client = rover_socket.UDP_client(50011, ip=self.ROV.rover_ip)
        self.gui_command_client = rover_socket.UDP_client(50012, ip=self.ROV.rover_ip)
        self.gui_rov_client = rover_socket.UDP_client(50013, ip=self.ROV.rover_ip)
        self.gui_vi_client = rover_socket.UDP_client(50014, ip=self.ROV.rover_ip)
        self.gui_li_client = rover_socket.UDP_client(50015, ip=self.ROV.rover_ip)
        self.gui_cal_client = rover_socket.UDP_client(50016, ip=self.ROV.rover_ip)
        self.gui_cc_client = rover_socket.UDP_client(50017, ip=self.ROV.rover_ip)
        self.gui_lobs_client = rover_socket.UDP_client(50018, ip=self.ROV.rover_ip)
        self.gui_gobs_client = rover_socket.UDP_client(50019, ip=self.ROV.rover_ip)
        self.gui_as_client = rover_socket.UDP_client(50020, ip=self.ROV.rover_ip)
        self.gui_cf_client = rover_socket.UDP_client(50021, ip=self.ROV.rover_ip)
        self.gui_pt_client = rover_socket.UDP_client(50022, ip=self.ROV.rover_ip)

        # self.gui_test_connection_client
        # self.gui_command_client
        # self.gui_rov_client
        # self.gui_vi_client
        # self.gui_li_client
        # self.gui_cal_client
        # self.gui_cc_client
        # self.gui_lobs_client
        # self.gui_gobs_client
        # self.gui_as_client
        # self.gui_cf_client
        # self.gui_pt_client


        self.get_data_retry = 0
        self.get_data_timer = QtCore.QTimer()
        self.get_data_timer.timeout.connect(self.getDataFromRover)
        self.get_data_timer.start(90)
        self.get_rover_status_retry = 0

        self.check_status_timer = QtCore.QTimer()
        self.check_status_timer.timeout.connect(self.checkStatus)
        self.check_status_timer.start(100)

        self.showMap()

        self.gui.tabWidget.setCurrentIndex(0)

        self.MainWindow.show()
        sys.exit(app.exec_())


    def showMessage(self, message):
        self.gui.console_1.append(str(message))
        self.gui.MessageBox_Edit.setText(str(message))


    def checkStatus(self):
        for i, j in zip([self.ROV.rover_run, self.VI.vision_run, self.LI.lidar_run], 
                    range(len(self.check_status_rover_run_list))):
            if i:
                 self.check_status_rover_Btn_list[j].setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.check_status_rover_Btn_list[j].setStyleSheet("background-color: rgb(255, 0, 0);")

        for i, j in zip([self.animation_run, self.VI.vision_build_map_mode, self.VI.vision_use_map_mode,
                    self.GUI.calibration_run, self.keyboard_control_run], 
                    range(len(self.check_status_func_run_list))):
            if i:
                self.check_status_func_Btn_list[j].setStyleSheet("background-color: rgb(0, 255, 93);")
            else:
                self.check_status_func_Btn_list[j].setStyleSheet("background-color: rgb(112, 155, 255);")



        ### Different action if no connection with ROVER
        if self.get_rover_status_retry < 50:
            ### Show Vision module Status
            if self.VI.vision_status in self.show_vision_status_dict:
                self.gui.VisionStatus_text.setText("{} : {}".format(
                    self.VI.vision_status, self.show_vision_status_dict[self.VI.vision_status]
                ))
            else:
                self.gui.VisionStatus_text.setText("{} : unknown".format(self.VI.vision_status))

            self.gui.LidarUSB_text.setText("USB Port: " + str(self.LI.lidar_USB_port) \
                + "\n Status: " + str(self.LI.lidar_state))

            self.gui.CurrentSpeed_text.setText(str(self.CC.car_control_move))
        else:
            self.LI.lidar_run, self.ROV.rover_run, self.VI.vision_run = False, False, False
            self.gui.CurrentSpeed_text.setText("No connection")
            self.gui.LidarUSB_text.setText("No connection")
            self.gui.VisionStatus_text.setText("No connection")


    def getDataFromRover(self):
        '''Get data from rover and turn into map'''
        self.gui_test_connection_client.send_object(1) # Send anything to make sure connection always open
        self.gui_rov_client.send_object(1)
        self.gui_vi_client.send_object(1)
        self.gui_li_client.send_object(1)
        self.gui_cal_client.send_object(1)
        self.gui_cc_client.send_object(1)
        self.gui_lobs_client.send_object(1)
        self.gui_gobs_client.send_object(1)
        self.gui_as_client.send_object(1)
        self.gui_cf_client.send_object(1)
        self.gui_pt_client.send_object(1)

        test_communication = self.gui_test_connection_client.recv_object(16)
        if test_communication is not None:
            self.get_rover_status_retry = 0
        temp = self.gui_rov_client.recv_object(256)
        if temp is not None:
            self.ROV = temp
        temp = self.gui_vi_client.recv_object(512)
        if temp is not None:
            self.VI = temp
        temp = self.gui_li_client.recv_object(65536)
        if temp is not None:
            self.LI = temp
        temp = self.gui_cal_client.recv_object(512)
        if temp is not None:
            self.CAL = temp
        temp = self.gui_cc_client.recv_object(512)
        if temp is not None:
            self.CC = temp
        temp = self.gui_lobs_client.recv_object(65536)
        if temp is not None:
            self.LOBS = temp
        temp = self.gui_gobs_client.recv_object(65536)
        if temp is not None:
            self.GOBS = temp
        temp = self.gui_as_client.recv_object(65536)
        if temp is not None:
            self.AS = temp
        temp = self.gui_cf_client.recv_object(65536)
        if temp is not None:
            self.CF = temp
        temp = self.gui_pt_client.recv_object(2048)
        if temp is not None:
            self.PT = temp
        else:
            self.get_rover_status_retry += 1

    def calibration(self):
        '''Call out Calibration Map'''
        if not hasattr(self, "CAL_GUI"):
            self.CAL_GUI = CalibrationUI(self.SV, self.gui_command_client)

        if not self.VI.vision_idle and self.VI.vision_run and self.LI.lidar_run:
            if not self.GUI.calibration_run and not self.CAL_GUI.calibration_MainWindow.isVisible():
                self.CAL_GUI.show_window()
                self.GUI.calibration_run = True
            else:
                self.CAL_GUI.close_window()
                self.GUI.calibration_run = False
        else:
            self.showMessage("For calibration, lidar, vision should be 'On' and \
                vision should be either build map mode or use map mode")



    def showMap(self):

        def plot_lidar_map():
            self.lidar_plot.setData(self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)

        def plot_global_map():
            self.route_plot.setData(self.AS.route_x, self.AS.route_y)
            self.gobs_plot.setData(self.GOBS.global_obstacle_x, self.GOBS.global_obstacle_y)
            self.end_point_plot.setData([self.AS.end_x], [self.AS.end_y])
            self.rover_plot.setData([self.VI.vision_x], [self.VI.vision_y])
            self.fitted_route_plot.setData(self.CF.fitted_route_x, self.CF.fitted_route_y)
            

        # print("show map")
        if not self.animation_run:
            self.lidar_plot_timer = QtCore.QTimer()
            self.lidar_plot_timer.timeout.connect(plot_lidar_map)
            self.lidar_plot_timer.start(100)

            self.global_plot_timer = QtCore.QTimer()
            self.global_plot_timer.timeout.connect(plot_global_map)
            self.global_plot_timer.start(100)

            self.showMessage("Show Map Start")
            self.animation_run = True
        else:

            self.lidar_plot_timer.stop()
            self.showMessage("Show Map Stop")
            self.animation_run = False


    def ShowMap_AddBtn_click(self):
        self.gui_command_client.send_list(["mbg"])

    def SaveMapBtn_click(self):
        try:
            name = QtWidgets.QFileDialog.getSaveFileName(self, \
                'Save File', "", "Rover Map Files (*.rovermap);;All Files (*)")
            if name != ("", ""):
                file = open(name[0],'w')
                np.save(file, arr=self.GOBS.global_obstacle, allow_pickle=False)
                file.close()
        except TypeError:
            traceback.print_exc()

    def StopAllBtn_click(self):
        self.showMessage("Stop ALL")

        if self.animation_run:
            self.lidar_plot_timer.stop()

            self.gui.GlobalMap.global_map_axes.clear()
            self.animation_run = False

        if self.keyboard_control_run:
            self.KeyboardControlTimer.stop()
            self.gui_command_client.send_list(['gkcc', 400, 405])


        


    def KeyboardControlBtn_click(self):

        def controller():
            forward_pwm = self.CC.car_control_forward_pwm + self.gui.KeyBoardControl_speed.value()
            backward_pwm = self.CC.car_control_backward_pwm - self.gui.KeyBoardControl_speed.value()
            s_pwm = self.CC.car_control_stop_pwm
            left_pwm = 409
            right_pwm = 320
            middle_pwm = self.CC.car_control_steer
            if keyboard.is_pressed('w') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', forward_pwm, middle_pwm])
            elif keyboard.is_pressed('w') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', forward_pwm, left_pwm])
            elif keyboard.is_pressed('w') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', forward_pwm, right_pwm])
            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', backward_pwm, middle_pwm])
            elif keyboard.is_pressed('s') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', backward_pwm, left_pwm])
            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui_command_client.send_list(['gkcc', backward_pwm, right_pwm])
            elif keyboard.is_pressed('d') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui_command_client.send_list(['gkcc', s_pwm, right_pwm])
            elif keyboard.is_pressed('a') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui_command_client.send_list(['gkcc', s_pwm, left_pwm])
            else:
                self.gui_command_client.send_list(["gkcc", s_pwm, middle_pwm])


            self.gui.KeyUp.setStyleSheet("background-color: rgb(0, 255, 0);") \
                if keyboard.is_pressed('w') else \
                    self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")

            self.gui.KeyDown.setStyleSheet("background-color: rgb(0, 255, 0);") \
                if keyboard.is_pressed('s') else \
                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
 
            self.gui.KeyRight.setStyleSheet("background-color: rgb(0, 255, 0);") \
                if keyboard.is_pressed('d') else \
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

            self.gui.KeyLeft.setStyleSheet("background-color: rgb(0 255, 0);") \
                if keyboard.is_pressed('a') else \
                    self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")



        self.KeyboardControlTimer = QtCore.QTimer()
        self.KeyboardControlTimer.timeout.connect(controller)

        if not self.keyboard_control_run:
            self.showMessage('Keyboard Control Start')
            self.KeyboardControlTimer.start(100)
            self.keyboard_control_run = True
        else:
            self.KeyboardControlTimer.stop()
            self.showMessage('Keyboard Control Stop')
            self.keyboard_control_run = False
                    

    def KeyboardControl_SetSpeedBtn_click(self):
        if self.ROV.rover_run:
            self.gui_command_client.send_list(["gss", self.gui.KeyBoardControl_speed.value()])
            self.showMessage("Set Car Speed " + str(self.gui.KeyBoardControl_speed.value()))
        else:
            self.showMessage("Warring : ROVER is not running")

    def KeyBoardControl_speed_value_change(self):
        self.gui.SetSpeed_label.setText(str(self.gui.KeyBoardControl_speed.value()))

    def WayPointBtn_click(self):
        self.showMessage("Way Point mode start !")


    def VisionUseMapBtn_click(self):
        if self.VI.vision_idle:
            self.gui_command_client.send_list(['gum', self.gui.UseMapID.value()])
            self.showMessage('Vision start use map')
        else:
            if self.VI.vision_run:
                self.showMessage('Vision module is busy')
            else:
                self.showMessage('Vision is not working')

    def VisionBuildMapBtn_click(self):
        if self.VI.vision_idle:
            self.gui_command_client.send_list(['gbm', self.gui.BuildMapID.value()])
            self.showMessage("Vision start building map")
        else:
            if self.VI.vision_run:
                self.showMessage('Vision module is busy')
            else:
                self.showMessage('Vision is not working')

    def VisionBuildMapStopBtn_click(self):
        if not self.VI.vision_idle and self.VI.vision_build_map_mode:
            self.gui_command_client.send_list(['gbms'])
            self.showMessage("Vision module is reseting please wait until ROVER reseting")
        else:
            self.showMessage("Vision is either idling or in use map mode")

    def VisionUseMapStopBtn_click(self):
        if not self.VI.vision_idle and self.VI.vision_use_map_mode:
            self.gui_command_client.send_list(['gums'])
            self.showMessage("Vision module is reseting please wait until ROVER reseting")
        else:
            self.showMessage("Vision is either idling or in build map mode")





class CalibrationUI:
    def __init__(self, SharedVariables, gui_command_client):
        self.SV = SharedVariables
        self.GOBS = self.SV.GOBS
        self.LOBS = self.SV.LOBS
        self.CAL = self.SV.CAL
        self.LI = self.SV.LI
        self.VI = self.SV.VI
        self.GUI = self.SV.GUI

        self.gui_command_client = gui_command_client


        self.calibration_MainWindow = QtWidgets.QMainWindow()
        self.calibration_gui = C_GUI.Ui_MainWindow()
        self.calibration_gui.setupUi(self.calibration_MainWindow)

        self.temp_calibrate_x = self.CAL.calibrate_x
        self.temp_calibrate_y = self.CAL.calibrate_y
        self.temp_calibrate_angle = self.CAL.calibrate_angle
        self.temp_calibrate_x_multi = self.CAL.calibrate_x_multi
        self.temp_calibrate_y_multi = self.CAL.calibrate_y_multi
        self.temp_calibrate_angle_multi = self.CAL.calibrate_angle_multi
        # self.temp_lobs_x = self.LOBS.local_obstacle_x
        # self.temp_lobs_y = self.LOBS.local_obstacle_y
        # self.temp_angle = self.VI.vision_angle


        self.calibration_gui.XCalibration_constant_spinBox.valueChanged.connect(self.x_spin)
        self.calibration_gui.YCalibration_constant_spinBox.valueChanged.connect(self.y_spin)
        self.calibration_gui.AngleCalibration_constant_spinBox.valueChanged.connect(self.angle_spin)
        self.calibration_gui.XCalibration_spinBox.valueChanged.connect(self.x_spin_multi)
        self.calibration_gui.YCalibration_spinBox.valueChanged.connect(self.y_spin_multi)
        self.calibration_gui.AngleCalibration_spinBox.valueChanged.connect(self.angle_spin_multi)
        self.calibration_gui.XCalibration_constant_slider.valueChanged.connect(self.x_slider)
        self.calibration_gui.YCalibration_constant_slider.valueChanged.connect(self.y_slider)
        self.calibration_gui.AngleCalibration_constant_slider.valueChanged.connect(self.angle_slider)
        self.calibration_gui.XCalibration_slider.valueChanged.connect(self.x_slider_multi)
        self.calibration_gui.YCalibration_slider.valueChanged.connect(self.y_slider_multi)
        self.calibration_gui.AngleCalibration_slider.valueChanged.connect(self.angle_slider_multi)
        self.calibration_MainWindow.closeEvent = self.closeEvent
        self.calibration_gui.ExitBtn.clicked.connect(self.calibration_MainWindow.close)
        self.calibration_gui.ConfirmBtn.clicked.connect(self.ConfirmBtn_click)
        self.calibration_gui.ResetBtn.clicked.connect(self.ResetBtn_click)
        self.calibration_gui.ExportBtn.clicked.connect(self.ExportBtn_click)
        self.calibration_gui.ImportBtn.clicked.connect(self.ImportBtn_click)

        self.initial_line, = self.calibration_gui.Calibration.calibration_map_axes.plot(self.SV.local_obstacle_x, self.SV.local_obstacle_y, 'g.')
        self.current_line, = self.calibration_gui.Calibration.calibration_map_axes.plot(self.SV.local_obstacle_x, self.SV.local_obstacle_y, 'b.')
        self.calibrate_line, = self.calibration_gui.Calibration.calibration_map_axes.plot(self.SV.local_obstacle_x, self.SV.local_obstacle_y, 'r.')
        self.position_line, = self.calibration_gui.Calibration.calibration_map_axes.plot(self.SV.vision_data[0], self.SV.vision_data[1], 'bo')
        self.calibrate_position_line, = self.calibration_gui.Calibration.calibration_map_axes.plot(self.SV.vision_data[0], self.SV.vision_data[1], 'ro')

    def show_window(self):
        self.calibration_MainWindow.show()
        self.calibration_animation = FuncAnimation(self.calibration_gui.Calibration.figure, self.animation, blit=True, interval=50)

    def close_window(self):
        self.calibration_MainWindow.close()


    def x_spin(self):
        self.temp_calibrate_x = self.calibration_gui.XCalibration_constant_spinBox.value()
        self.calibration_gui.XCalibration_constant_label.setText("X Calibration : {}".format(self.temp_calibrate_x))
        self.calibration_gui.XCalibration_constant_slider.setValue(self.temp_calibrate_x)

    def x_spin_multi(self):
        self.temp_calibrate_x_multi = self.calibration_gui.XCalibration_spinBox.value() / 100
        self.calibration_gui.XCalibration_label.setText("X Calibration multiply: {} %".format(self.calibration_gui.XCalibration_spinBox.value()))
        self.calibration_gui.XCalibration_slider.setValue(self.calibration_gui.XCalibration_spinBox.value())

    def x_slider(self):
        self.temp_calibrate_x = self.calibration_gui.XCalibration_constant_slider.value()
        self.calibration_gui.XCalibration_constant_label.setText("X Calibration : {}".format(self.temp_calibrate_x))
        self.calibration_gui.XCalibration_constant_spinBox.setValue(self.temp_calibrate_x)

    def x_slider_multi(self):
        self.temp_calibrate_x_multi = self.calibration_gui.XCalibration_slider.value() / 100
        self.calibration_gui.XCalibration_label.setText("X Calibration multiply: {} %".format(self.calibration_gui.XCalibration_slider.value()))
        self.calibration_gui.XCalibration_spinBox.setValue(self.calibration_gui.XCalibration_slider.value())

    def y_spin(self):
        self.temp_calibrate_y = self.calibration_gui.YCalibration_constant_spinBox.value()
        self.calibration_gui.YCalibration_constant_label.setText("Y Calibration : {}".format(self.temp_calibrate_y))
        self.calibration_gui.YCalibration_constant_slider.setValue(self.temp_calibrate_y)

    def y_spin_multi(self):
        self.temp_calibrate_y_multi = self.calibration_gui.YCalibration_spinBox.value() / 100
        self.calibration_gui.YCalibration_label.setText("Y Calibration multiply : {} %".format(self.calibration_gui.YCalibration_spinBox.value()))
        self.calibration_gui.YCalibration_slider.setValue(self.calibration_gui.YCalibration_spinBox.value())

    def y_slider(self):
        self.temp_calibrate_y = self.calibration_gui.YCalibration_constant_slider.value()
        self.calibration_gui.YCalibration_constant_label.setText("Y Calibration : {}".format(self.temp_calibrate_y))
        self.calibration_gui.YCalibration_constant_spinBox.setValue(self.temp_calibrate_y)

    def y_slider_multi(self):
        self.temp_calibrate_y_multi = self.calibration_gui.YCalibration_slider.value() / 100
        self.calibration_gui.YCalibration_label.setText("Y Calibration multiply: {} %".format(self.calibration_gui.YCalibration_slider.value()))
        self.calibration_gui.YCalibration_spinBox.setValue(self.calibration_gui.YCalibration_slider.value())

    def angle_spin(self):
        self.temp_calibrate_angle = self.calibration_gui.AngleCalibration_constant_spinBox.value()
        self.calibration_gui.AngleCalibration_constant_label.setText("Angle Calibration : {}".format(self.temp_calibrate_angle))
        self.calibration_gui.AngleCalibration_constant_slider.setValue(self.temp_calibrate_angle)

    def angle_spin_multi(self):
        self.temp_calibrate_angle_multi = self.calibration_gui.AngleCalibration_spinBox.value() / 100
        self.calibration_gui.AngleCalibration_label.setText("Angle Calibration multiply : {} %".format(self.calibration_gui.AngleCalibration_spinBox.value()))
        self.calibration_gui.AngleCalibration_slider.setValue(self.calibration_gui.AngleCalibration_spinBox.value())

    def angle_slider(self):
        self.temp_calibrate_angle = self.calibration_gui.AngleCalibration_constant_slider.value()
        self.calibration_gui.AngleCalibration_constant_label.setText("Angle Calibration : {}".format(self.temp_calibrate_angle))
        self.calibration_gui.AngleCalibration_constant_spinBox.setValue(self.temp_calibrate_angle)

    def angle_slider_multi(self):
        self.temp_calibrate_angle_multi = self.calibration_gui.AngleCalibration_slider.value() / 100
        self.calibration_gui.AngleCalibration_label.setText("Angle Calibration multiply: {} %".format(self.calibration_gui.AngleCalibration_slider.value()))
        self.calibration_gui.AngleCalibration_spinBox.setValue(self.calibration_gui.AngleCalibration_slider.value())

    def ConfirmBtn_click(self):
        self.gui_command_client.send_list([
            "gcal", self.temp_calibrate_x, self.temp_calibrate_x_multi,
            self.temp_calibrate_y, self.temp_calibrate_y_multi,
            self.temp_calibrate_angle, self.temp_calibrate_angle_multi,
            self.CAL.calibrate_difference_between_lidar_and_vision
        ])


    def ResetBtn_click(self):
        self.initial_line.set_data(self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)

    def closeEvent(self, event):
        # self.gui.showMessage("Calibration stop")
        self.calibration_animation._stop()
        self.GUI.calibration_run = False
        event.accept()

    def animation(self, i):
        self.calibration_gui.VisionData_label.setText(
            "Vx: {} Vy: {} Vangle: {}".format(self.VI.vision_x, self.VI.vision_y, self.VI.vision_angle)
        )

        temp_vision_x = self.VI.vision_x * self.temp_calibrate_x_multi + self.temp_calibrate_x
        temp_vision_y = self.VI.vision_y * self.temp_calibrate_y_multi + self.temp_calibrate_y
        temp_vision_angle = self.VI.vision_angle * self.temp_calibrate_angle_multi + self.temp_calibrate_angle
        temp_vision_angle_radian = np.radians(temp_vision_angle)
        temp_local_obstacle_x = np.round(np.cos(self.LI.lidar_angle - \
                temp_vision_angle_radian + 0.5*np.pi)* \
                self.LI.lidar_radius + temp_vision_x, 0)
        temp_local_obstacle_y = np.round(np.sin(self.LI.lidar_angle - \
                temp_vision_angle_radian + 0.5*np.pi)*\
                self.LI.lidar_radius + temp_vision_y, 0)

        self.current_line.set_data(self.LOBS.local_obstacle_x, self.LOBS.local_obstacle_y)
        self.calibrate_line.set_data(temp_local_obstacle_x, temp_local_obstacle_y)
        self.position_line.set_data(self.VI.vision_data[0], self.VI.vision_data[1])
        self.calibrate_position_line.set_data(temp_vision_x, temp_vision_y)
        self.calibration_gui.Calibration.calibration_map_axes.autoscale(True)

        return self.current_line, self.calibrate_line, self.position_line, self.calibrate_position_line, self.initial_line,

    def ExportBtn_click(self):
        try:
            name = QtWidgets.QFileDialog.getSaveFileName(self.calibration_MainWindow, \
                'Save File', "", "Rover Calibration Files (*.calibration);;Text Files (*.txt);;All Files (*)")
            if name != ("", ""):
                file = open(name[0],'w')
                text = "x{}xm{}y{}ym{}a{}am{}".format(self.CAL.calibrate_x, self.CAL.calibrate_x_multi, \
                    self.CAL.calibrate_y, self.CAL.calibrate_y_multi, self.CAL.calibrate_angle, self.CAL.calibrate_angle_multi)
                file.write(text)
                file.close()
        except TypeError:
            traceback.print_exc()

    def ImportBtn_click(self):
        name = QtWidgets.QFileDialog.getOpenFileName(self.calibration_MainWindow, \
            'Open File', "", "Rover Calibration Files (*.calibration);;Text Files (*.txt);;All Files (*)")
        if name != ("", ""):
            file = open(name[0],'r')
            text = file.read()
            self.temp_calibrate_x = int(text[text.index("x") + 1 : text.index("xm")])
            self.temp_calibrate_x_multi = float(text[text.index("xm") + 2 : text.index("y")])
            self.temp_calibrate_y = int(text[text.index("y") + 1 : text.index("ym")])
            self.temp_calibrate_y_multi = float(text[text.index("ym") + 2 : text.index("a")])
            self.temp_calibrate_angle = int(text[text.index("a") + 1 : text.index("am")])
            self.temp_calibrate_angle_multi = float(text[text.index("am") + 2 : len(text)])
            self.calibration_gui.XCalibration_constant_slider.setValue(self.temp_calibrate_x)
            self.calibration_gui.YCalibration_constant_slider.setValue(self.temp_calibrate_y)
            self.calibration_gui.AngleCalibration_constant_slider.setValue(self.temp_calibrate_angle)
            self.calibration_gui.XCalibration_slider.setValue(self.temp_calibrate_x_multi * 100)
            self.calibration_gui.YCalibration_slider.setValue(self.temp_calibrate_y_multi * 100)
            self.calibration_gui.AngleCalibration_slider.setValue(self.temp_calibrate_angle_multi * 100)
            # print(
            #     self.temp_calibrate_x, self.temp_calibrate_x_multi,
            #     self.temp_calibrate_y, self.temp_calibrate_y_multi,
            #     self.temp_calibrate_angle, self.temp_calibrate_angle_multi
            #     )

            file.close()









if __name__ == "__main__":
    ROVER_gui()
