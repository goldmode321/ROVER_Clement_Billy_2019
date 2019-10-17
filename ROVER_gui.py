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
from matplotlib.animation import FuncAnimation

class ROVER_gui():
    def __init__(self):

        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        MainWindow = QtWidgets.QMainWindow()

        self.Keyboard_Control_Mode = False
        self.get_data_from_rover_run = False
        self.lidar_data = [[0,0,0],[1,1,1]]
        self.lidar_angle = [0]
        self.lidar_angle2 = [0]
        self.lidar_radius = [0]
        self.vision_data = [0, 0, 0, -1]
        self.local_obstacle_x = [0]
        self.local_obstacle_y = [0]
        self.vision_angle_radian = 0


        # self.lidar_data_test = multiprocessing.Queue()
        # self.lidar_angle_test = multiprocessing.Queue()
        # self.lidar_radius_test = multiprocessing.Queue()

        self.show_map_timer = None
        # get_data_from_rover_thread = threading.Thread(target=self.get_data_from_rover, daemon=True)
        # get_data_from_rover_thread.start()
        # get_data_from_rover_process = multiprocessing.Process(target=self.get_data_from_rover, args=(self.lidar_data_test,\
        #     self.lidar_angle_test, self.lidar_radius_test), daemon=True)
        # get_data_from_rover_process.start()


        self.gui = GUI.Ui_MainWindow()
        self.gui.setupUi(MainWindow)

        self.canvas = self.gui.LidarMap
        # vertical_layout = QtWidgets.QVBoxLayout()
        # vertical_layout.addWidget(self.canvas)

        self.gui.StopAllBtn.clicked.connect(self.StopAllBtn_click)
        self.gui.KeyboardControlBtn.clicked.connect(self.KeyboardControlBtn_click)
        self.gui.VisionBuildMapBtn.clicked.connect(self.BuildMapBtn_click)
        self.gui.VisionUseMapBtn.clicked.connect(self.GetLidarDataBtn_click)
        self.gui.ShowMapBtn.clicked.connect(self.show_map)
        

        self.gui_keyboard_control_client = rover_socket.UDP_client(50011, 0, '192.168.5.2')


        self.lidar_plot, = self.canvas.lidar_axes.plot(0, 0, '.')
        self.animation_run = False

        self.gui.tabWidget.setCurrentIndex(1)
        # self.gui.tabWidget.currentChanged.connect(self.show_map)

        # self.gui.LidarMap.plot(self.lidar_angle, self.lidar_radius)
        # self.gui.LidarMap.show_plot()

        MainWindow.show()
        sys.exit(app.exec_())



    def show_map(self):

        # self.lidar_angle = [math.radians(-i[1]) + 0.5*math.pi for i in self.lidar_data]
        # self.lidar_radius = [i[2] for i in self.lidar_data]
        def plot_map(i):
            self.lidar_plot.set_xdata(self.lidar_angle)
            self.lidar_plot.set_ydata(self.lidar_radius)
            self.canvas.fig.canvas.update()

            return self.lidar_plot,
            # self.gui.LidarMap.clear()
            # self.gui.LidarMap.plot(self.lidar_angle, self.lidar_radius)
            # self.gui.LidarMap.show_plot()
            # self.gui.LidarMap.canvas.start_event_loop(0.01)

            # self.gui.GlobalMap.clear()
            # self.gui.GlobalMap.plot(self.local_obstacle_x, self.local_obstacle_y, 'bo')
            # self.gui.GlobalMap.plot(self.vision_data[0], self.vision_data[1], 'ro')
            # self.gui.GlobalMap.plot_arrow(self.vision_data[0], self.vision_data[1], 200*math.cos(-self.vision_angle_radian+0.5*math.pi), 200*math.sin(-self.vision_angle_radian+0.5*math.pi), width=30)
            # self.gui.GlobalMap.draw()
            # self.gui.GlobalMap.canvas.start_event_loop(0.01)


        def get_data_from_rover():
            
            self.gui_get_lidar_vision_client.send_list([1]) # Send anything to make sure connection always open
            temp_receive = self.gui_get_lidar_vision_client.recv_list(32768)
            if temp_receive is not None:
                self.lidar_data = temp_receive[0]
                self.lidar_angle = [math.radians(-i[1]) + 0.5*math.pi for i in self.lidar_data]
                self.lidar_radius = [i[2] for i in self.lidar_data]
                self.vision_data = temp_receive[1]
                if self.vision_data[3] == 4:
                    self.vision_angle_radian = math.radians(self.vision_data[2])
                    self.local_obstacle_x = numpy.cos(numpy.array(self.lidar_angle)-self.vision_angle_radian)*\
                        numpy.array(self.lidar_radius) + self.vision_data[0]
                    self.local_obstacle_y = numpy.sin(numpy.array(self.lidar_angle)-self.vision_angle_radian)*\
                        numpy.array(self.lidar_radius) + self.vision_data[1]


        if not self.animation_run:
            self.gui_get_lidar_vision_client = rover_socket.UDP_client(50010, 0, '192.168.5.2')
            self.get_data_timer = QtCore.QTimer()
            self.get_data_timer.timeout.connect(get_data_from_rover)
            self.get_data_timer.start(40)
            self.animation = FuncAnimation(self.canvas.figure, plot_map, blit=True, interval=50)
            self.gui.console_1.append("Show Map Start")
            self.animation_run = True
        else:
            self.get_data_timer.stop()
            self.animation._stop()
            self.gui.console_1.append("Show Map Stop")
            self.gui_get_lidar_vision_client.close()
            self.animation_run = False




    def StopAllBtn_click(self):
        self.gui.console_1.append('Stop ALL')

        if self.get_data_from_rover_run:
            self.get_data_from_rover_run = False

        if self.animation_run:
            self.animation._stop()
            self.get_data_timer.stop()
            self.gui_get_lidar_vision_client.close()
            self.animation_run = False

        if self.Keyboard_Control_Mode:
            self.gui.console_1.append('Stopped Keyboard control!')
            self.KeyboardControlTimer.stop()

        


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

        self.KeyboardControlTimer = QtCore.QTimer()
        self.KeyboardControlTimer.timeout.connect(controller)            

        if not self.Keyboard_Control_Mode:
            self.gui.console_1.append('Keyboard Control Start')
            self.KeyboardControlTimer.start(50)
            self.Keyboard_Control_Mode = True
        else:
            self.KeyboardControlTimer.stop()
            self.gui.console_1.append('Keyboard Control Stop')
            self.Keyboard_Control_Mode = False
                    
            
    def WayPointBtn_click(self):
        self.gui.console_1.append('Way point mode start')


    def GetLidarDataBtn_click(self):
        self.gui.console_1.append('Start getting Lidar\'s data')


    def BuildMapBtn_click(self):
        self.gui.console_1.append('Start building map')


    # def get_data_from_rover(self):
    #     self.gui_get_lidar_vision_client = rover_socket.UDP_client(50010, 0, '192.168.5.2')
    #     self.get_data_from_rover_run = True
    #     while self.get_data_from_rover_run:
    #         self.gui_get_lidar_vision_client.send_list([1]) # Send anything to make sure connection always open
    #         temp_receive = self.gui_get_lidar_vision_client.recv_list(32768)
    #         if temp_receive is not None:
    #             self.lidar_data = temp_receive[0]
    #             self.lidar_angle = [math.radians(-i[1]) + 0.5*math.pi for i in self.lidar_data]
    #             self.lidar_radius = [i[2] for i in self.lidar_data]
    #             self.vision_data = temp_receive[1]
    #             if self.vision_data[3] == 4:
    #                 self.vision_angle_radian = math.radians(self.vision_data[2])
    #                 self.local_obstacle_x = numpy.cos(numpy.array(self.lidar_angle)-self.vision_angle_radian)*\
    #                     numpy.array(self.lidar_radius) + self.vision_data[0]
    #                 self.local_obstacle_y = numpy.sin(numpy.array(self.lidar_angle)-self.vision_angle_radian)*\
    #                     numpy.array(self.lidar_radius) + self.vision_data[1]

    #         time.sleep(0.04)

# class gui_background_communication():

if __name__ == "__main__":
    ROVER_gui()