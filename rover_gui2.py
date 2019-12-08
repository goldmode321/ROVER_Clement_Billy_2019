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
import numpy
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph


import gui.rover_ui_file as GUI
import gui.rover_calibration as C_GUI
import rover_socket
import rover_shared_variable


class Calculator:
    def __init__(self, SharedVariables):
        self.GUI = SharedVariables.GUI
        self.CAL = SharedVariables.CAL
        self.LOBS = SharedVariables.LOBS
        self.GOBS = SharedVariables.GOBS
        self.VI = SharedVariables.VI

    def show_message(self, message):
        self.GUI.gui.MessageBox_Edit.setText(message)
        self.GUI.gui.console_1.append(message)

    def get_x_axis_limit(self):
        ''' return value set for set_xlim()'''
        return numpy.concatenate((self.LOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).min(), \
            numpy.concatenate((self.LOBS.local_obstacle_x, self.GOBS.global_obstacle_x)).max()

    def get_y_axis_limit(self):
        ''' return value set for set_ylim() '''
        return numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).min(), \
                numpy.concatenate((self.LOBS.local_obstacle_y, self.GOBS.global_obstacle_y)).max()


    def get_calibrate_temp_local_obstacle(
                        self, vision_x, vision_y, vision_angle_radian, 
                        lidar_radius, lidar_angle
                        ):
        return numpy.cos(numpy.array(lidar_angle) - vision_angle_radian + 0.5*math.pi)*\
                    numpy.array(lidar_radius) + vision_x, \
                numpy.sin(numpy.array(lidar_angle) - vision_angle_radian + 0.5*math.pi)*\
                    numpy.array(lidar_radius) + vision_y

    def get_calibrate_temp_vision_xy_angle(self, temp_calibrate_x, temp_calibrate_y,
                        temp_calibrate_x_multi, temp_calibrate_y_multi, 
                        temp_calibrate_angle, temp_calibrate_angle_multi):
        return self.VI.vision_data[0] * temp_calibrate_x_multi + temp_calibrate_x, \
            self.VI.vision_data[1] * temp_calibrate_y_multi + temp_calibrate_y, \
                math.radians(self.VI.vision_data[2] * temp_calibrate_angle_multi + temp_calibrate_angle),\


class ROVER_gui():
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        self.MainWindow = QtWidgets.QMainWindow()

        self.gui = GUI.Ui_MainWindow()
        self.gui.setupUi(self.MainWindow)

        self.SV = rover_shared_variable.SharedVariables()
        self.VI = self.SV.VI
        self.LI = self.SV.LI
        self.MAP = self.SV.MAP
        self.CAL = self.SV.CAL
        self.CC = self.SV.CC
        self.LOBS = self.SV.LOBS
        self.GOBS = self.SV.GOBS
        self.GUI = self.SV.GUI

        self.GUI.gui = self.gui
        self.CALCULATOR = Calculator()


        self.current_speed = 0
        self.show_vision_status_dict = {0:"Booting", 1:"Waiting for command", 2:"Loading data",
            3:":Locating, please move around", 4:":Working normally",
            5:":Lost current position"}

        self.rover_run, self.lidar_server_run, self.vision_server_run, \
        self.animation_run, self.vision_build_map_mode, self.vision_use_map_mode, \
        self.CAL.calibration_run, self.keyboard_control_run, self.vision_idle \
            = False, False, False, False, False, False, False, False, False

        self.check_status_rover_run_list = [
            self.rover_run, self.vision_server_run, self.lidar_server_run
        ]

        self.check_status_rover_Btn_list = [
            self.gui.RoverMainOnOffBtn,
            self.gui.VisionOnOffBtn,
            self.gui.LidarOnOffBtn
        ]

        self.check_status_func_run_list = [
            self.animation_run,
            self.vision_build_map_mode, 
            self.vision_use_map_mode,
            self.CAL.calibration_run,
            self.keyboard_control_run
        ]

        self.check_status_func_Btn_list = [
            self.gui.ShowMapBtn,
            self.gui.VisionBuildMapBtn,
            self.gui.VisionUseMapBtn,
            self.gui.CalibrationBtn,
            self.gui.KeyboardControlBtn
        ]

        self.gui.StopAllBtn.clicked.connect(self.StopAllBtn_click)
        self.gui.KeyboardControlBtn.clicked.connect(self.KeyboardControlBtn_click)
        self.gui.VisionBuildMapBtn.clicked.connect(self.VisionBuildMapBtn_click)
        self.gui.VisionBuildMapStopBtn.clicked.connect(self.VisionBuildMapStopBtn_click)
        self.gui.VisionUseMapBtn.clicked.connect(self.VisionUseMapBtn_click)
        self.gui.VisionUseMapStopBtn.clicked.connect(self.VisionUseMapStopBtn_click)
        self.gui.ShowMapBtn.clicked.connect(self.show_map)
        self.gui.ShowMap_AddBtn.clicked.connect(self.ShowMap_AddBtn_click)
        self.gui.KeyboardControl_SetSpeedBtn.clicked.connect(self.KeyboardControl_SetSpeedBtn_click)
        self.gui.KeyBoardControl_speed.valueChanged.connect(self.KeyBoardControl_speed_value_change)
        self.gui.CalibrationBtn.clicked.connect(self.calibration)
        self.gui.SaveMapBtn.clicked.connect(self.SaveMapBtn_click)
        self.gui.ImportMapBtn.clicked.connect(self.ImportMapBtn_click)

        self.gui_get_lidar_vision_client = rover_socket.UDP_client(50010, 0, '192.168.5.2')
        self.gui_get_rover_status_client = rover_socket.UDP_client(50012, 0, '192.168.5.2')
        self.gui_rover_command_client = rover_socket.UDP_client(50013, 0, '192.168.5.2')
        self.gui_get_vision_udp_client = rover_socket.UDP_client(50015, ip="192.168.5.2")
        self.gui_get_lidar_udp_client = rover_socket.UDP_client(50016, ip="192.168.5.2")
        self.gui_get_map_udp_client = rover_socket.UDP_client(50017, ip="192.168.5.2")
        self.gui_get_calibration_udp_client = rover_socket.UDP_client(50018, ip="192.168.5.2")
        self.gui_get_car_control_udp_client = rover_socket.UDP_client(50019, ip="192.168.5.2")
        self.gui_get_local_obstacle_udp_client = rover_socket.UDP_client(50020, 0, "192.168.5.2")
        self.gui_get_global_obstacle_udp_client = rover_socket.UDP_client(50021, 0, "192.168.5.2")

        self.get_data_retry = 0
        self.get_data_timer = QtCore.QTimer()
        self.get_data_timer.timeout.connect(self.get_data_from_rover)
        self.get_data_timer.start(40)
        self.get_rover_status_retry = 0
        self.get_rover_status_timer = QtCore.QTimer()
        self.get_rover_status_timer.timeout.connect(self.get_rover_status)
        self.get_rover_status_timer.start(40)
        self.check_status_timer = QtCore.QTimer()
        self.check_status_timer.timeout.connect(self.check_status)
        self.check_status_timer.start(100)

        self.show_map()

        self.gui.tabWidget.setCurrentIndex(0)
        # self.gui.tabWidget.currentChanged.connect(self.show_map)


        self.MainWindow.show()
        sys.exit(app.exec_())


    def check_status(self):
        for i, j in zip([self.rover_run, self.vision_server_run, self.lidar_server_run], 
                    range(len(self.check_status_rover_run_list))):
            if i:
                 self.check_status_rover_Btn_list[j].setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.check_status_rover_Btn_list[j].setStyleSheet("background-color: rgb(255, 0, 0);")

        for i, j in zip([self.animation_run, self.vision_build_map_mode, self.vision_use_map_mode,
                    self.CAL.calibration_run, self.keyboard_control_run], 
                    range(len(self.check_status_func_run_list))):
            if i:
                self.check_status_func_Btn_list[j].setStyleSheet("background-color: rgb(0, 255, 93);")
            else:
                self.check_status_func_Btn_list[j].setStyleSheet("background-color: rgb(112, 155, 255);")


    def get_rover_status(self):
        '''Specialize for getting rover status'''
        def show_vision_status():
            '''Explain vision status meanning'''
            if self.VI.vision_status in self.show_vision_status_dict:
                self.gui.VisionStatus_text.setText("{} : {}".format(
                    self.VI.vision_status, self.show_vision_status_dict[self.VI.vision_status]
                    ))
            else:
                self.gui.VisionStatus_text.setText("{} : unknown".format(self.VI.vision_status))


        self.gui_get_rover_status_client.send_list([1])
        temp_rover_status_receive = self.gui_get_rover_status_client.recv_list()
        if temp_rover_status_receive is not None:
            self.gui.LidarUSB_text.setText("USB Port: " + str(temp_rover_status_receive[0]) \
                + "\n Status: " + str(temp_rover_status_receive[1]))

            self.lidar_server_run, self.VI.vision_status, self.vision_server_run, \
            self.vision_idle, self.rover_run, self.current_speed, \
            self.vision_build_map_mode, self.vision_use_map_mode = \
                temp_rover_status_receive[2:10]

            self.get_rover_status_retry = 0
            show_vision_status()
            self.gui.CurrentSpeed_text.setText(str(self.current_speed))
        else:
            self.get_rover_status_retry+=1
            if self.get_rover_status_retry > 50:
                self.lidar_server_run, self.rover_run, self.vision_server_run = \
                    False, False, False
                self.gui.CurrentSpeed_text.setText("No connection")
                self.gui.LidarUSB_text.setText("No connection")
                self.gui.VisionStatus_text.setText("No connection")


    def get_data_from_rover(self):
        '''Get data from rover and turn into map'''
        self.gui_get_lidar_vision_client.send_list([1]) # Send anything to make sure connection always open
        temp_lidar_vision_receive = self.gui_get_lidar_vision_client.recv_list(16)
        if temp_lidar_vision_receive is not None:
            self.gui_get_vision_udp_client.recv_object(512)
            self.gui_get_lidar_udp_client.recv_object(8192)
            self.gui_get_map_udp_client.recv_object(512)
            self.gui_get_calibration_udp_client.recv_object(512)
            self.gui_get_car_control_udp_client.recv_object(512)
            self.gui_get_local_obstacle_udp_client.recv_object(8192)
            self.gui_get_global_obstacle_udp_client.recv_object(65536)

            self.SV.lidar_data = numpy.asarray(temp_lidar_vision_receive[0])
            self.SV.lidar_angle = numpy.radians(self.SV.lidar_data[:, 1]* (-1)) + 0.0*math.pi
            self.SV.lidar_radius = self.SV.lidar_data[:, 2]
            self.SV.vision_data = temp_lidar_vision_receive[1]

            if self.SV.vision_data[3] == 4:
                self.CALCULATOR.calculate_vision_xy_angle()
                self.CALCULATOR.calculate_local_obstacle()
                self.CALCULATOR.calculate_arrow()


    def calibration(self):
        if not hasattr(self, "CAL_GUI"):
            self.CAL_GUI = CalibrationUI(self.SV, self.CALCULATOR)

        if not self.vision_idle and self.vision_server_run and self.lidar_server_run:
            if not self.SV.calibration_run and not self.CAL_GUI.calibration_MainWindow.isVisible():
                self.CAL_GUI.show_window()
                self.SV.calibration_run = True
            else:
                self.CAL_GUI.close_window()
                self.SV.calibration_run = False
        else:
            self.CALCULATOR.show_message("For calibration, lidar, vision should be 'On' and \
                vision should be either build map mode or use map mode")


    def show_map(self):

        def plot_lidar_map(i):
            self.lidar_plot.set_xdata(self.SV.lidar_angle)
            self.lidar_plot.set_ydata(self.SV.lidar_radius)
            return self.lidar_plot,

        def plot_global_map(i):
            self.gui.GlobalMap.global_map_axes.set_xlim(self.CALCULATOR.get_x_axis_limit())
            self.gui.GlobalMap.global_map_axes.set_ylim(self.CALCULATOR.get_y_axis_limit())
            self.global_map_plot_vision.set_data(self.SV.vision_x, self.SV.vision_y)
            self.global_map_plot_local_obstacle.set_data(self.SV.local_obstacle_x, self.SV.local_obstacle_y)
            self.global_map_plot_arrow.set_data(self.SV.arrow_x, self.SV.arrow_y)
            self.global_map_plot_global_obstacle.set_data(self.SV.global_obstacle_x, self.SV.global_obstacle_y)

            return self.global_map_plot_local_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow, self.global_map_plot_global_obstacle,

        # print("show map")
        if not self.animation_run:
            self.lidar_plot, = self.gui.LidarMap.lidar_axes.plot(0, 0, 'b.')

            self.global_map_plot_arrow, = self.gui.GlobalMap.global_map_axes.plot([], [], 'g', linewidth=3)
            self.global_map_plot_vision, = self.gui.GlobalMap.global_map_axes.plot([], [], 'ro', linewidth=200)
            self.global_map_plot_local_obstacle, = self.gui.GlobalMap.global_map_axes.plot([], [], 'b.')
            self.global_map_plot_global_obstacle, = self.gui.GlobalMap.global_map_axes.plot([], [], 'k.')

            self.lidar_animation = FuncAnimation(self.gui.LidarMap.figure, plot_lidar_map, blit=True, interval=50)
            self.global_animation = FuncAnimation(self.gui.GlobalMap.fig, plot_global_map, blit=True, interval=50)
            self.CALCULATOR.show_message("Show Map Start")
            self.animation_run = True
        else:
            self.lidar_animation._stop()
            self.global_animation._stop()
            self.CALCULATOR.show_message("Show Map Stop")
            self.animation_run = False


    def ShowMap_AddBtn_click(self):
        self.CALCULATOR.get_global_obstacle()


    def SaveMapBtn_click(self):
        try:
            if self.SV.global_obstacle != numpy.array([]):
                name = QtWidgets.QFileDialog.getSaveFileName(self.MainWindow, \
                    'Save File', "", "Numpy Files (*.npz);;All Files (*)")
                if name != ("", ""):
                    numpy.savez(name[0], self.gui.BuildMapID.value(), self.SV.global_obstacle)
            else:
                self.CALCULATOR.show_message("Global Map is None")

        except TypeError:
            traceback.print_exc()


    def ImportMapBtn_click(self):
        try:
            name = QtWidgets.QFileDialog.getOpenFileName(self.MainWindow, \
                'Open File', "", "Numpy Files (*.npz)")
            if name != ("", ""):
                temp_info = numpy.load(name[0], allow_pickle=False)
                self.gui.BuildMapID.setValue(temp_info['arr_0'])
                self.SV.global_obstacle = temp_info['arr_1']
                self.SV.local_obstacle_x = temp_info['arr_1'][:, 0]
                self.SV.local_obstacle_y = temp_info['arr_1'][:, 1]
        except:
            traceback.print_exc()

    def StopAllBtn_click(self):
        self.CALCULATOR.show_message("Stop ALL")

        if self.animation_run:
            self.lidar_animation._stop()
            self.global_animation._stop()
            self.gui.GlobalMap.global_map_axes.clear()
            self.animation_run = False

        if self.keyboard_control_run:
            self.KeyboardControlTimer.stop()
            self.gui_keyboard_control_client.close()


    def KeyboardControlBtn_click(self):

        def controller():
            if keyboard.is_pressed('w') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['w'])
            elif keyboard.is_pressed('w') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['wa'])
            elif keyboard.is_pressed('w') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['wd'])
            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['s'])
            elif keyboard.is_pressed('s') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['sa'])
            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui_keyboard_control_client.send_list(['sd'])
            elif keyboard.is_pressed('d') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui_keyboard_control_client.send_list(['d'])
            elif keyboard.is_pressed('a') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui_keyboard_control_client.send_list(['a'])


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



        self.gui_keyboard_control_client = rover_socket.UDP_client(50011, 0, '192.168.5.2')
        self.KeyboardControlTimer = QtCore.QTimer()
        self.KeyboardControlTimer.timeout.connect(controller)

        if not self.keyboard_control_run:
            self.CALCULATOR.show_message('Keyboard Control Start')
            self.KeyboardControlTimer.start(50)
            self.keyboard_control_run = True
        else:
            self.KeyboardControlTimer.stop()
            self.gui_keyboard_control_client.close()
            self.CALCULATOR.show_message('Keyboard Control Stop')
            self.keyboard_control_run = False


    def KeyboardControl_SetSpeedBtn_click(self):
        if self.rover_run:
            self.gui_rover_command_client.send_list(["gss", self.gui.KeyBoardControl_speed.value()])
            self.CALCULATOR.show_message("Set Car Speed " + str(self.gui.KeyBoardControl_speed.value()))
        else:
            self.CALCULATOR.show_message("Warring : ROVER is not running")


    def KeyBoardControl_speed_value_change(self):
        self.gui.SetSpeed_label.setText(str(self.gui.KeyBoardControl_speed.value()))


    def WayPointBtn_click(self):
        self.CALCULATOR.show_message("Way Point mode start !")


    def VisionUseMapBtn_click(self):
        if self.vision_idle:
            self.gui_rover_command_client.send_list(['gum', self.gui.UseMapID.value()])
            self.CALCULATOR.show_message('Vision start use map')
        else:
            if self.vision_server_run:
                self.CALCULATOR.show_message('Vision module is busy')
            else:
                self.CALCULATOR.show_message('Vision is not working')


    def VisionBuildMapBtn_click(self):
        if self.vision_idle:
            self.gui_rover_command_client.send_list(['gbm', self.gui.BuildMapID.value()])
            self.CALCULATOR.show_message("Vision start building map")
        else:
            if self.vision_server_run:
                self.CALCULATOR.show_message('Vision module is busy')
            else:
                self.CALCULATOR.show_message('Vision is not working')


    def VisionBuildMapStopBtn_click(self):
        if not self.vision_idle and self.vision_build_map_mode:
            self.gui_rover_command_client.send_list(['gbms'])
            self.CALCULATOR.show_message("Vision module is reseting please wait until ROVER reseting")
        else:
            self.CALCULATOR.show_message("Vision is either idling or in use map mode")


    def VisionUseMapStopBtn_click(self):
        if not self.vision_idle and self.vision_use_map_mode:
            self.gui_rover_command_client.send_list(['gums'])
            self.CALCULATOR.show_message("Vision module is reseting please wait until ROVER reseting")
        else:
            self.CALCULATOR.show_message("Vision is either idling or in build map mode")





class CalibrationUI():
    def __init__(self, SharedVariables_class, calculator_class):
        self.SV = SharedVariables_class
        self.CALCULATOR = calculator_class


        self.calibration_MainWindow = QtWidgets.QMainWindow()
        self.calibration_gui = C_GUI.Ui_MainWindow()
        self.calibration_gui.setupUi(self.calibration_MainWindow)

        self.temp_calibrate_x = self.SV.calibrate_x
        self.temp_calibrate_y = self.SV.calibrate_y
        self.temp_calibrate_angle = self.SV.calibrate_angle
        self.temp_calibrate_x_multi = self.SV.calibrate_x_multi
        self.temp_calibrate_y_multi = self.SV.calibrate_y_multi
        self.temp_calibrate_angle_multi = self.SV.calibrate_angle_multi


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
        self.SV.calibrate_x = self.temp_calibrate_x
        self.SV.calibrate_y = self.temp_calibrate_y
        self.SV.calibrate_angle = self.temp_calibrate_angle
        self.SV.calibrate_x_multi = self.temp_calibrate_x_multi
        self.SV.calibrate_y_multi = self.temp_calibrate_y_multi
        self.SV.calibrate_angle_multi = self.temp_calibrate_angle_multi
        self.CALCULATOR.show_message("Confirm calibration : x : {} , y : {} , angle : {}  \
            x multi : {} , y multi : {} , angle multi : {}" \
            .format(self.SV.calibrate_x, self.SV.calibrate_y, self.SV.calibrate_angle, \
                self.SV.calibrate_x_multi, self.SV.calibrate_y_multi, self.SV.calibrate_angle_multi))

    def ResetBtn_click(self):
        self.initial_line.set_data(self.SV.local_obstacle_x, self.SV.local_obstacle_y)

    def closeEvent(self, event):
        self.CALCULATOR.show_message("Calibration stop")
        self.calibration_animation._stop()
        self.SV.calibration_run = False
        event.accept()

    def animation(self, i):
        self.calibration_gui.VisionData_label.setText(str(self.SV.vision_data))

        temp_vision_x, temp_vision_y, temp_vision_angle_radian = self.CALCULATOR.get_calibrate_temp_vision_xy_angle(
                                    self.temp_calibrate_x, self.temp_calibrate_y, 
                                    self.temp_calibrate_x_multi, self.temp_calibrate_y_multi,
                                    self.temp_calibrate_angle, self.temp_calibrate_angle_multi
                                )
        temp_local_obstacle_x, temp_local_obstacle_y = self.CALCULATOR.get_calibrate_temp_local_obstacle(
                                                temp_vision_x, temp_vision_y, 
                                                temp_vision_angle_radian,
                                                self.SV.lidar_radius, self.SV.lidar_angle
                                            )

        self.current_line.set_data(self.SV.local_obstacle_x, self.SV.local_obstacle_y)
        self.calibrate_line.set_data(temp_local_obstacle_x, temp_local_obstacle_y)
        self.position_line.set_data(self.SV.vision_data[0], self.SV.vision_data[1])
        self.calibrate_position_line.set_data(temp_vision_x, temp_vision_y)
        self.calibration_gui.Calibration.calibration_map_axes.autoscale(True)

        return self.current_line, self.calibrate_line, self.position_line, self.calibrate_position_line, self.initial_line,

    def ExportBtn_click(self):
        try:
            name = QtWidgets.QFileDialog.getSaveFileName(self.calibration_MainWindow, \
                'Save File', "", "Rover Calibration Files (*.calibration);;Text Files (*.txt);;All Files (*)")
            if name != ("", ""):
                file = open(name[0],'w')
                text = "x{}xm{}y{}ym{}a{}am{}".format(self.SV.calibrate_x, self.SV.calibrate_x_multi, \
                    self.SV.calibrate_y, self.SV.calibrate_y_multi, self.SV.calibrate_angle, self.SV.calibrate_angle_multi)
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
