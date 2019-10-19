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
        self.lidar_data = [[0,0,0],[1,1,1]]
        self.lidar_angle = [0]
        self.lidar_angle2 = [0]
        self.lidar_radius = [0]
        self.vision_data = [0, 0, 0, -1]
        self.local_obstacle_x = [0]
        self.local_obstacle_y = [0]
        self.vision_angle_radian = 0
        self.arrow_x = [0]
        self.arrow_y = [0]



        self.gui = GUI.Ui_MainWindow()
        self.gui.setupUi(MainWindow)




        self.gui.StopAllBtn.clicked.connect(self.StopAllBtn_click)
        self.gui.KeyboardControlBtn.clicked.connect(self.KeyboardControlBtn_click)
        self.gui.VisionBuildMapBtn.clicked.connect(self.BuildMapBtn_click)
        self.gui.VisionUseMapBtn.clicked.connect(self.GetLidarDataBtn_click)
        self.gui.ShowMapBtn.clicked.connect(self.show_map)


        self.gui_get_lidar_vision_client = rover_socket.UDP_client(50010, 0, '192.168.5.2')
        self.get_data_timer = QtCore.QTimer()
        self.get_data_timer.timeout.connect(self.get_data_from_rover)
        self.get_data_timer.start(40)

        self.gui.tabWidget.setCurrentIndex(0)
        # self.gui.tabWidget.currentChanged.connect(self.show_map)




        MainWindow.show()
        # self.show_map()
        sys.exit(app.exec_())



    def get_data_from_rover(self):
        
        self.gui_get_lidar_vision_client.send_list([1]) # Send anything to make sure connection always open
        temp_receive = self.gui_get_lidar_vision_client.recv_list(32768)
        if temp_receive is not None:
            self.lidar_data = temp_receive[0]
            self.lidar_angle = [math.radians(-i[1]) + 0.0*math.pi for i in self.lidar_data]
            self.lidar_radius = [i[2] for i in self.lidar_data]
            self.vision_data = temp_receive[1]
            # self.gui.console_1.append(str(temp_receive[1]))
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
            self.gui.GlobalMap.global_map_axes.set_xlim(min(self.local_obstacle_x), max(self.local_obstacle_x))
            self.gui.GlobalMap.global_map_axes.set_ylim(min(self.local_obstacle_y), max(self.local_obstacle_y))
            self.global_map_plot_vision.set_xdata(self.vision_data[0])
            self.global_map_plot_vision.set_ydata(self.vision_data[1])
            self.global_map_plot_obstacle.set_xdata(self.local_obstacle_x)
            self.global_map_plot_obstacle.set_ydata(self.local_obstacle_y)
            self.global_map_plot_arrow.set_xdata(self.arrow_x)
            self.global_map_plot_arrow.set_ydata(self.arrow_y)
            # self.gui.GlobalMap.fig.canvas.update()

            # return self.global_map_plot_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow, self.global_map_plot_initial_position,
            return self.global_map_plot_obstacle, self.global_map_plot_vision, self.global_map_plot_arrow,
            # return self.global_map_plot_obstacle,

        
        if not self.animation_run:
            self.lidar_plot, = self.gui.LidarMap.lidar_axes.plot(0, 0, 'b.')


            # self.global_map_plot_vision, = self.gui.GlobalMap.global_map_axes.plot(self.vision_data[0], self.vision_data[1], 'ro', linewidth=200)
            # self.global_map_plot_obstacle, = self.gui.GlobalMap.global_map_axes.plot(self.local_obstacle_x, self.local_obstacle_y, 'b.')
            self.global_map_plot_vision, = self.gui.GlobalMap.global_map_axes.plot([], [], 'ro', linewidth=200)
            # self.global_map_plot_initial_position, = self.gui.GlobalMap.global_map_axes.plot([], [], '^', linewidth=200)
            self.global_map_plot_obstacle, = self.gui.GlobalMap.global_map_axes.plot([], [], 'b.')
            self.global_map_plot_arrow, = self.gui.GlobalMap.global_map_axes.plot([], [], 'g', linewidth=3)
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
                    
            
    def WayPointBtn_click(self):
        self.gui.console_1.append('Way point mode start')


    def GetLidarDataBtn_click(self):
        self.gui.console_1.append('Start getting Lidar\'s data')


    def BuildMapBtn_click(self):
        self.gui.console_1.append('Start building map')




if __name__ == "__main__":
    ROVER_gui()