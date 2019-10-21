import sys
import os
import threading
from PyQt5 import QtWidgets, QtCore, QtGui
import gui.rover_ui_file as GUI
import keyboard
import time
import rover_socket
import multiprocessing
import queue
import math
import numpy
import traceback
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation

class ROVER_gui():
    def __init__(self):

        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        MainWindow = QtWidgets.QMainWindow()

        self.keyboard_control_run = False
        self.animation_run = False

        self.rover_run = False

        self.current_speed = 0

        self.lidar_data = [[0,0,0],[1,1,1]]
        self.lidar_angle = [0]
        self.lidar_radius = [0]
        self.lidar_server_run = False

        self.vision_server_run = False
        self.vision_status = -1
        self.vision_data = [0, 0, 0, self.vision_status]
        self.vision_idle = False
        self.vision_build_map_mode = False
        self.vision_use_map_mode = False

        self.local_obstacle_x = numpy.array([0, 1, 2])
        self.local_obstacle_y = numpy.array([0, 1, 2])
        self.global_obstacle_x = numpy.array([])
        self.global_obstacle_y = numpy.array([])
        self.vision_angle_radian = 0
        self.arrow_x = [0]
        self.arrow_y = [0]



        self.gui = GUI.Ui_MainWindow()
        self.gui.setupUi(MainWindow)



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


        self.gui_get_lidar_vision_client = rover_socket.UDP_client(50010, 0, '192.168.5.2')
        self.gui_get_rover_status_client = rover_socket.UDP_client(50012, 0, '192.168.5.2')
        self.gui_rover_command_client = rover_socket.UDP_client(50013, 0, '192.168.5.2')

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





        MainWindow.show()
        # self.show_map()
        sys.exit(app.exec_())


    def check_status(self):
        if self.rover_run:
            self.gui.RoverMainOnOffBtn.setStyleSheet("background-color: rgb(0, 255, 0);")
        else:
            self.gui.RoverMainOnOffBtn.setStyleSheet("background-color: rgb(255, 0, 0);")

        if self.vision_server_run:
            self.gui.VisionOnOffBtn.setStyleSheet("background-color: rgb(0, 255, 0);")
        else:
            self.gui.VisionOnOffBtn.setStyleSheet("background-color: rgb(255, 0, 0);")

        if self.lidar_server_run:
            self.gui.LidarOnOffBtn.setStyleSheet("background-color: rgb(0, 255, 0);")
        else:
            self.gui.LidarOnOffBtn.setStyleSheet("background-color: rgb(255, 0, 0);")

        if self.animation_run:
            self.gui.ShowMapBtn.setStyleSheet("background-color: rgb(0, 255, 93);")
        else:
            self.gui.ShowMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")
        
        if self.vision_build_map_mode:
            self.gui.VisionBuildMapBtn.setStyleSheet("background-color: rgb(0, 255, 93);")
        else:
            self.gui.VisionBuildMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")

        if self.vision_use_map_mode:
            self.gui.VisionUseMapBtn.setStyleSheet("background-color: rgb(0, 255, 93);")
        else:
            self.gui.VisionUseMapBtn.setStyleSheet("background-color: rgb(112, 155, 255);")    


    def get_rover_status(self):
        '''Specialize for getting rover status'''
        def show_vision_status():
            '''Explain vision status meanning'''
            if self.vision_status == 0:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Booting")
            elif self.vision_status == 1:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Waiting for command")
            elif self.vision_status == 2:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Loading data")
            elif self.vision_status == 3:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Locating, please move around")
            elif self.vision_status == 4:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Working normally")
            elif self.vision_status == 5:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ":Lost current position")
            else:
                self.gui.VisionStatus_text.setText(str(self.vision_status) + ": unkonwn")

        self.gui_get_rover_status_client.send_list([1])
        temp_rover_status_receive = self.gui_get_rover_status_client.recv_list()
        if temp_rover_status_receive is not None:
            self.gui.LidarUSB_text.setText("USB Port: " + str(temp_rover_status_receive[0]) \
                + "\n Status: " + str(temp_rover_status_receive[1]))
            self.lidar_server_run = temp_rover_status_receive[2]
            self.vision_status = temp_rover_status_receive[3]
            self.vision_server_run = temp_rover_status_receive[4]
            self.vision_idle = temp_rover_status_receive[5]
            self.rover_run = temp_rover_status_receive[6]
            self.current_speed = temp_rover_status_receive[7]
            self.vision_build_map_mode = temp_rover_status_receive[8]
            self.vision_use_map_mode = temp_rover_status_receive[9]
            self.get_rover_status_retry = 0
            show_vision_status()
            self.gui.CurrentSpeed_text.setText(str(self.current_speed))
        else:
            self.get_rover_status_retry+=1
            if self.get_rover_status_retry > 50:
                self.lidar_server_run = False
                self.rover_run = False
                self.vision_server_run = False
                self.gui.CurrentSpeed_text.setText("No connection")
                self.gui.LidarUSB_text.setText("No connection")
                self.gui.VisionStatus_text.setText("No connection")




    def get_data_from_rover(self):
        '''Get data from rover and turn into map'''
        self.gui_get_lidar_vision_client.send_list([1]) # Send anything to make sure connection always open
        temp_lidar_vision_receive = self.gui_get_lidar_vision_client.recv_list(32768)
        if temp_lidar_vision_receive is not None:
            self.lidar_data = temp_lidar_vision_receive[0]
            self.lidar_angle = [math.radians(-i[1]) + 0.0*math.pi for i in self.lidar_data]
            self.lidar_radius = [i[2] for i in self.lidar_data]
            self.vision_data = temp_lidar_vision_receive[1]
            # self.gui.console_1.append(str(temp_lidar_vision_receive[1]))
            if self.vision_data[3] == 4:
                self.vision_angle_radian = math.radians(self.vision_data[2])
                self.local_obstacle_x = numpy.cos(numpy.array(self.lidar_angle) - self.vision_angle_radian + 0.5*math.pi)*\
                    numpy.array(self.lidar_radius) + self.vision_data[0]
                self.local_obstacle_y = numpy.sin(numpy.array(self.lidar_angle) - self.vision_angle_radian + 0.5*math.pi)*\
                    numpy.array(self.lidar_radius) + self.vision_data[1]

                self.arrow_x = [self.vision_data[0], self.vision_data[0] + 200*math.cos(-self.vision_angle_radian+0.5*math.pi)]
                self.arrow_y = [self.vision_data[1], self.vision_data[1] + 200*math.sin(-self.vision_angle_radian+0.5*math.pi)]






    def show_map(self):

        def plot_lidar_map(i):
            self.lidar_plot.set_xdata(self.lidar_angle)
            self.lidar_plot.set_ydata(self.lidar_radius)
            # self.gui.LidarMap.fig.canvas.update()


            return self.lidar_plot,


        def plot_global_map(i):
            self.gui.GlobalMap.global_map_axes.set_xlim(numpy.concatenate((\
                self.local_obstacle_x, self.global_obstacle_x)).min(), numpy.concatenate((\
                    self.local_obstacle_x, self.global_obstacle_x)).max())
            self.gui.GlobalMap.global_map_axes.set_ylim(numpy.concatenate((\
                self.local_obstacle_y, self.global_obstacle_y)).min(), numpy.concatenate((\
                    self.local_obstacle_y, self.global_obstacle_y)).max())
            self.global_map_plot_vision.set_xdata(self.vision_data[0])
            self.global_map_plot_vision.set_ydata(self.vision_data[1])
            self.global_map_plot_local_obstacle.set_xdata(self.local_obstacle_x)
            self.global_map_plot_local_obstacle.set_ydata(self.local_obstacle_y)
            self.global_map_plot_arrow.set_xdata(self.arrow_x)
            self.global_map_plot_arrow.set_ydata(self.arrow_y)
            self.global_map_plot_global_obstacle.set_xdata(self.global_obstacle_x)
            self.global_map_plot_global_obstacle.set_ydata(self.global_obstacle_y)
            # self.gui.GlobalMap.fig.canvas.update()

            # return self.global_map_plot_local_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow, self.global_map_plot_initial_position,
            # return self.global_map_plot_local_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow,
            # return self.global_map_plot_local_obstacle,
            return self.global_map_plot_local_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow, self.global_map_plot_global_obstacle,

        # print("show map")
        if not self.animation_run:
            self.lidar_plot, = self.gui.LidarMap.lidar_axes.plot(0, 0, 'b.')

            self.global_map_plot_arrow, = self.gui.GlobalMap.global_map_axes.plot([], [], 'g', linewidth=3)

            # self.global_map_plot_vision, = self.gui.GlobalMap.global_map_axes.plot(self.vision_data[0], self.vision_data[1], 'ro', linewidth=200)
            # self.global_map_plot_local_obstacle, = self.gui.GlobalMap.global_map_axes.plot(self.local_obstacle_x, self.local_obstacle_y, 'b.')
            self.global_map_plot_vision, = self.gui.GlobalMap.global_map_axes.plot([], [], 'ro', linewidth=200)
            # self.global_map_plot_initial_position, = self.gui.GlobalMap.global_map_axes.plot([], [], '^', linewidth=200)
            self.global_map_plot_local_obstacle, = self.gui.GlobalMap.global_map_axes.plot([], [], 'b.')
            self.global_map_plot_global_obstacle, = self.gui.GlobalMap.global_map_axes.plot([], [], 'k.')

            # self.global_map_plot_arrow, = self.gui.GlobalMap.global_map_axes.plot(self.arrow_x, self.arrow_y, 'g', linewidth=3)

            self.lidar_animation = FuncAnimation(self.gui.LidarMap.figure, plot_lidar_map, blit=True, interval=50)
            self.global_animation = FuncAnimation(self.gui.GlobalMap.fig, plot_global_map, blit=True, interval=50)
            self.gui.console_1.append("Show Map Start")
            self.animation_run = True
        else:
            self.lidar_animation._stop()
            self.global_animation._stop()
            self.gui.console_1.append("Show Map Stop")
            self.animation_run = False


    def ShowMap_AddBtn_click(self):
        self.global_obstacle_x = numpy.concatenate((self.local_obstacle_x, self.global_obstacle_x))
        self.global_obstacle_y = numpy.concatenate((self.local_obstacle_y, self.global_obstacle_y))

    def StopAllBtn_click(self):
        self.gui.console_1.append('Stop ALL')


        if self.animation_run:
            self.lidar_animation._stop()
            self.global_animation._stop()
            self.gui.GlobalMap.global_map_axes.clear()
            self.animation_run = False

        if self.keyboard_control_run:
            self.gui.console_1.append('Stopped Keyboard control!')
            self.KeyboardControlTimer.stop()
            self.gui_keyboard_control_client.close()

        


    def KeyboardControlBtn_click(self):

        def controller():
            if keyboard.is_pressed('w') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui.KeyUp.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['w'])


            elif keyboard.is_pressed('w') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui.KeyUp.setStyleSheet("background-color: rgb(0, 255, 0);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['wa'])


            elif keyboard.is_pressed('w') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui.KeyUp.setStyleSheet("background-color: rgb(0, 255, 0);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['wd'])


            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui.KeyDown.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['s'])


            elif keyboard.is_pressed('s') and keyboard.is_pressed('a') and not keyboard.is_pressed('d'):
                self.gui.KeyDown.setStyleSheet("background-color: rgb(0, 255, 0);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['sa'])


            elif keyboard.is_pressed('s') and not keyboard.is_pressed('a') and keyboard.is_pressed('d'):
                self.gui.KeyDown.setStyleSheet("background-color: rgb(0, 255, 0);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['sd'])


            elif keyboard.is_pressed('d') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui.KeyRight.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['d'])


            elif keyboard.is_pressed('a') and not keyboard.is_pressed('w') and not keyboard.is_pressed('s'):
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(0, 255, 0);")

                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")

                self.gui_keyboard_control_client.send_list(['a'])


            else:
                self.gui.KeyUp.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyDown.setStyleSheet("background-color: rgb(255, 255, 255);")
                self.gui.KeyRight.setStyleSheet("background-color: rgb(255, 255, 255);")               
                self.gui.KeyLeft.setStyleSheet("background-color: rgb(255, 255, 255);")

        self.gui_keyboard_control_client = rover_socket.UDP_client(50011, 0, '192.168.5.2')
        self.KeyboardControlTimer = QtCore.QTimer()
        self.KeyboardControlTimer.timeout.connect(controller)            

        if not self.keyboard_control_run:
            self.gui.console_1.append('Keyboard Control Start')
            self.KeyboardControlTimer.start(50)
            self.keyboard_control_run = True
        else:
            self.KeyboardControlTimer.stop()
            self.gui_keyboard_control_client.close()
            self.gui.console_1.append('Keyboard Control Stop')
            self.keyboard_control_run = False
                    

    def KeyboardControl_SetSpeedBtn_click(self):
        if self.rover_run:
            self.gui_rover_command_client.send_list(["gss", self.gui.KeyBoardControl_speed.value()])
            self.gui.console_1.append("Set Car Speed")
        else:
            self.gui.console_1.append("Warring : Rover is not running")

    def KeyBoardControl_speed_value_change(self):
        self.gui.SetSpeed_label.setText(str(self.gui.KeyBoardControl_speed.value()))

    def WayPointBtn_click(self):
        self.gui.console_1.append('Way point mode start')


    def VisionUseMapBtn_click(self):
        if self.vision_idle:
            self.gui_rover_command_client.send_list(['gum', self.gui.UseMapID.value()])
            self.gui.console_1.append('Vision start use map')
        else:
            self.gui.console_1.append('Vision module is busy')


    def VisionBuildMapBtn_click(self):
        if self.vision_idle:
            self.gui_rover_command_client.send_list(['gbm', self.gui.BuildMapID.value()])
            self.gui.console_1.append('Vision start building map')
        else:
            self.gui.console_1.append('Vision module is busy')

    def VisionBuildMapStopBtn_click(self):
        if not self.vision_idle and self.vision_build_map_mode:
            self.gui_rover_command_client.send_list(['gbms'])
        else:
            self.gui.console_1.append("Vision is either idling or in use map mode")

    def VisionUseMapStopBtn_click(self):
        if not self.vision_idle and self.vision_use_map_mode:
            self.gui_rover_command_client.send_list(['gums'])
        else:
            self.gui.console_1.append("Vision is either idling or in build map mode")



if __name__ == "__main__":
    ROVER_gui()