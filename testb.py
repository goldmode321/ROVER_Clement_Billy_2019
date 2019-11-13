# import time
# import rover_socket

# gui_server = rover_socket.UDP_server(50010, 0, '192.168.5.2')
# run = True
# while run:
#     gui_server.send_list([1])
#     gui_receive = gui_server.recv_list(32768)
#     print(gui_receive)
#     time.sleep(0.1)
    # if gui_server.addr is not None:
        # print(gui_receive)
        # gui_server.send_list_back()





# import multiprocessing
# import time

# class process1(multiprocessing.Process):
#     def __init__(self, queue_process1_run):
#         super().__init__(self, daemon=True)
#         self.queue_process1_run = queue_process1_run


#     def run(self):
#         self.queue_process1_run.get()
#         while run:

# import rplidar
# import time
# import traceback
# import logging
# import threading
# import rover_socket


# class Lidar():


#     def __init__(self):

#         ###############
#         self.lidar = None
#         self.lidar_client = None
#         self.lidar_thread_client = None
#         self.lidar_run_flag = False
#         self.lidar_data = []
#         self.lidar_connect = False
#         ###############
#         try:
#             self.lidar_thread_client = rover_socket.UDP_server(50010, 0, '192.168.5.2')
#             self.lidar_scan_port()
#             if self.lidar_connect:
#                 self.lidar_run_flag = True
#                 self.lidar_main()
#             else:
#                 print(("LiDAR is not initialized"))



#         except:
#             traceback.print_exc()
#             logging.exception("Got error\n")
#             if self.lidar_thread_client != None:
#                 self.lidar_thread_client.close()


#     def lidar_main(self):
#         self.lidar_run_background()
#         while self.lidar_run_flag:
#             try:
#                 command = input("exit to exit")
#                 if command == 'exit':
#                     self.lidar_run_flag = False
#                     self.stop()



#             except:
#                 traceback.print_exc()
#                 logging.exception('Got error : ')
#                 self.lidar_run_flag = False






#     def lidar_run_background(self):
#         thread = threading.Thread(target = self.get_lidar_data ,daemon = True)
#         thread.start()


# #=============================================#
# #                   Liberary                  #
# #=============================================#


#     def lidar_scan_port(self):
#         retry = 0
#         self.lidar_scanning_flag = True
#         self.initial_scanning_port_num = 0
#         logging.info('Scanning RPLidar port')
#         while self.lidar_scanning_flag:
#             try:
#                 self.lidar_USB_port = '/dev/ttyUSB'+str(self.initial_scanning_port_num)
#                 logging.info('Scanning '+self.lidar_USB_port)
#                 self.lidar = rplidar.RPLidar(self.lidar_USB_port)
#                 time.sleep(0.1)
#                 self.lidar_state = self.get_status()
#                 if self.lidar_state[0] == 'Good':
#                     logging.info(self.lidar_state)
#                     self.lidar_scanning_flag = False
#                     self.lidar_connect = True
#                     logging.info("lidar initialize successuflly")
#                 else:
#                     print(self.lidar_state)
#                     print('1 or more undefined problems , plase check RPLiDAR')
#                     logging.warning(str(self.lidar_state)+' ; 1 or more undefined problems , please check RPLiDAR')


#             except rplidar.RPLidarException:
#                 # print("Not this one , system continue scanning")
#                 logging.info('Not this one , system continue scanning')
#                 self.initial_scanning_port_num += 1
#                 if retry < 5:
#                     if self.initial_scanning_port_num > 5:
#                         self.initial_scanning_port_num = 0
#                         retry += 1
#                         logging.warning('Rescanning RPLiDAR port')
#                 else:
#                     self.lidar_scanning_flag = False

#             except:
#                 traceback.print_exc()
#                 logging.exception("Got error\n")

#     def get_status(self):
#         return self.lidar.get_health()


#     def stop(self):
#         self.lidar.stop()  
#         self.lidar.disconnect()

#     def reconnect(self):
#         self.lidar.stop()
#         self.lidar = rplidar.RPLidar(self.lidar_USB_port)

#     def get_lidar_object(self):
#         return self.lidar

#     def get_lidar_data(self):
#         print(321)
#         try:
#             for scan in self.lidar.iter_scans():
#                 # self.lidar_data = scan
#                 self.lidar_thread_client.recv_list()
#                 self.lidar_data = [list(scan) for scan in scan if scan[2] > 450]
#                 self.lidar_thread_client.send_list_back([self.lidar_data])
#                 print(self.lidar_data)
#         except:
#             traceback.print_exc()
#             self.reconnect()
#             self.get_lidar_data()

# if __name__ == "__main__":
#     Lidar()





# coding=utf-8
# -*- coding: utf-8 -*-
 
# import sys
# import os
# import random
# import matplotlib
# # Make sure that we are using QT5
# matplotlib.use('Qt5Agg')
# from PyQt5 import QtCore, QtWidgets
# from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout,QHBoxLayout, QSizePolicy, QMessageBox, QWidget
# import numpy as np
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
# from matplotlib.figure import Figure
# from matplotlib.animation import FuncAnimation
 
 
# from PyQt5.QtWidgets import *
# from PyQt5 import QtCore
# from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget
 
# progname = os.path.basename(sys.argv[0])
# progversion = "0.1"
 
# global n_drops
# global scat
# global rain_drops
# n_drops = 100
 
# class MyMplCanvas(FigureCanvas):
#     def __init__(self, parent=None, width=5, height=4, dpi=100):
#         self.fig = Figure(figsize=(width, height), dpi=dpi)
#         self.axes = self.fig.add_subplot(111, projection = 'polar')
 
#         self.ax = self.fig.add_subplot(111)
#         self.ax.set_xlim(0, 100)
#         self.ax.set_ylim(0, 100)
#         x = range(1, 7)
#         y = (22, 30, 30, 29, 32, 31)
#         self.ax.set_xticklabels([i+100 for i in x])
#         self.ax.set_yticklabels(y)
#         self.ax.grid(True)
 
#         self.compute_initial_figure()
#         FigureCanvas.__init__(self, self.fig)
#         self.setParent(parent)
#         FigureCanvas.setSizePolicy(self,
#                                    QtWidgets.QSizePolicy.Expanding,
#                                    QtWidgets.QSizePolicy.Expanding)
#         FigureCanvas.updateGeometry(self)
 
#     def compute_initial_figure(self):
#         pass
 
# class ApplicationWindow(QtWidgets.QMainWindow):
#     def __init__(self):
#         QtWidgets.QMainWindow.__init__(self)
#         self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
#         self.setWindowTitle("application main window")
#         self.main_widget = QtWidgets.QWidget(self)
 
#         vbox = QtWidgets.QVBoxLayout(self.main_widget)
 
#         self.canvas =  MyMplCanvas( self.main_widget,width=5, height=4, dpi=100) ###attention###
#         vbox.addWidget(self.canvas)
 
#         hbox = QtWidgets.QHBoxLayout(self.main_widget)
#         self.start_button = QPushButton("start", self)
#         self.stop_button = QPushButton("stop", self)
#         self.exit_button = QPushButton("exit", self)
 
#         self.start_button.clicked.connect(self.on_start)
#         self.stop_button.clicked.connect(self.on_stop)
#         self.exit_button.clicked.connect(self.on_exit)
 
#         hbox.addWidget(self.start_button)
#         hbox.addWidget(self.stop_button)
#         hbox.addWidget(self.exit_button)
 
#         vbox.addLayout(hbox)
#         self.setLayout(vbox)
 
#         self.main_widget.setFocus()
#         self.setCentralWidget(self.main_widget)

#         global n_drops
#         global scat
#         global rain_drops
#         rain_drops = np.zeros(n_drops, dtype=[('position', float, 2)])
#         self.scat= self.canvas.axes.scatter(rain_drops['position'][:, 0], rain_drops['position'][:, 1],s=10,lw=0.5)
#         print(self.scat)

#         data = np.random.uniform(0,100,(5,2))
#         # self.plotting = self.canvas.axes.plot(data)
#         # self.line, = self.canvas.axes.plot(data[:,0], data[:,1], '.')
#         self.line, = self.canvas.axes.plot(0, 50, '.')
#         # self.plotting = self.canvas.axes.plot(data)
#         print(self.line)
#         self.canvas.axes.cla()
#         # self.canvas.ax.set_xlim(0, 100)
#         self.canvas.ax.set_ylim(0, 100)
 
#     def update_line(self, i):
#         global n_drops
#         global scat
#         global rain_drops
#         rain_drops['position']= np.random.uniform(0, 100, (n_drops, 2))

#         data = np.random.uniform(0,100,(5,2))
#         random_radius = np.random.uniform(0,100,200)
#         random_angle = np.random.uniform(0, 2*np.pi, 200)

#         # self.scat.set_offsets(rain_drops['position'])

#         self.line.set_xdata(random_angle)
#         self.line.set_ydata(random_radius)
#         # self.line.set_xdata(data[:,1])
#         # self.canvas.axes.draw_artist(self.line)
#         # self.canvas.axes.draw_artist(self.plotx)
#         # self.plotting[1].set_ydata(data[:, 0])
#         # self.plotting.cla()
#         # self.plotting.plot(data)
#         self.canvas.fig.canvas.update()
#         # self.canvas.fig.canvas.flush_events()

#         # return self.scat,

#         return self.line,
#         # return self.plotting,
 
 
#     def on_start(self):
#         def init_fun():

#             self.canvas.axes.clear()
#             # self.canvas.ax.set_xlim(0, 100)
#             self.canvas.ax.set_ylim(0, 100)
#             self.line.set_xdata(0)
#             self.line.set_ydata(0)
#             self.canvas.fig.canvas.update()
#             # self.canvas.axes.draw_artist(self.line)
#             # self.canvas.axes.plot(0,0)
#             print('init_fun')
#             return self.line,
#         self.ani = FuncAnimation(self.canvas.fig, self.update_line, blit=True, interval=25)
#         # self.ani = FuncAnimation(self.canvas.figure, self.update_line,
#                                 #  blit=True, interval=25)
#     def on_stop(self):
#         self.ani._stop()
 
#     def on_exit(self):
#         self.close()
 
# if __name__ == "__main__":
#     App = QApplication(sys.argv)
#     aw = ApplicationWindow()
#     aw.show()
#     App.exit()

#     sys.exit(App.exec_())
# ————————————————
# 版权声明：本文为CSDN博主「小小飞行器」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
# 原文链接：https://blog.csdn.net/oFresh/article/details/61924876





# from PyQt5.QtWidgets import (QWidget, QApplication, QPushButton, 
#         QLabel, QHBoxLayout, QSizePolicy)
# from PyQt5.QtGui import QColor
# from PyQt5.QtCore import QPropertyAnimation, pyqtProperty
# import sys


# class MyLabel(QLabel):
    
#     def __init__(self, text):
#         super().__init__(text)

#     def _set_color(self, col):
        
#         palette = self.palette()
#         palette.setColor(self.foregroundRole(), col)
#         self.setPalette(palette)

#     color = pyqtProperty(QColor, fset=_set_color)


# class Example(QWidget):

#     def __init__(self):
#         super().__init__()


#         self.initUI()
        
        
#     def initUI(self):     

#         hbox = QHBoxLayout(self)
        
#         self.button = QPushButton("Start", self)
#         self.button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
#         hbox.addWidget(self.button)
        
#         hbox.addSpacing(40)

#         self.label = MyLabel("Summer")
#         font = self.label.font()
#         font.setPointSize(35)
#         self.label.setFont(font)
#         hbox.addWidget(self.label)

#         self.anim = QPropertyAnimation(self.label, b"color")
#         self.anim.setDuration(2500)
#         self.anim.setLoopCount(2)
#         self.anim.setStartValue(QColor(0, 0, 0))
#         self.anim.setEndValue(QColor(255, 255, 255))

#         self.button.clicked.connect(self.anim.start)
        
#         self.setGeometry(300, 300, 380, 250)
#         self.setWindowTitle('Color anim')
#         self.show()    
        
# if __name__ == "__main__":
    
#     app = QApplication([])
#     ex = Example()
#     ex.show()
#     app.exec_()

# import time
# class aaa():
#     def f1(self, i):
#         print(i)
# class bbb(aaa):
#     def __init__(self):
#         self.bbb1 = 10
#         self.f1(3)
# class ccc(bbb):
#     def __init__(self):
#         bbb.__init__(self)
#         print(self.bbb1)


# class info():
#     def __init__(self):
#         self.a = 1
#         self.b = 2
#         self.c = 3
# class a1(info):
#     super().__init__()


import time

def f1(a=None):
    a = a if not a is None else 1
    return a

global b
b = 2
def f2(z = 1):
    return z
# class c:
#     def __init__(self):
#         self.x = 2
#     def re(self):
#         return self.x
# cc = c()

################
def func(a):
    b = a if a%2==0 else 0
#################
def func2(a):
    if a%2==0:
        b=a
    else:
        b=0

##########################
def fun3():
    a=1
    a2=1
    a3=1
    a4=1
    a5=1
    a6=1
    a7=1
    a8=1
    a9=1
    a10=1

#ooooooooooooooooooooooooooooooooo
def fun4():
    a,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10=1,1,1,1,1,1,1,1,1,1,1

#XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
def fun5():
    a=list()
    for i in range(10):
        a.append(i)

global dic1
dic1 = {0:0,1:0,2:0,3:0,4:0,5:0}
#OOOOOOOOOOOOOOOOOOOOOOOOO
def fun6(a):
    if a%5 in dic1:
        b = dic1[a%5]
#XXXXXXXXXXXXXXXXXXXXX
def fun7(a):
    if a%5==0:
        b = 0
    elif a%5==1:
        b=0
    elif a%5==2:
        b=0
    elif a%5==3:
        b=0
    elif a%5==4:
        b=0
    elif a%5==5:
        b=0

global l1
l1 = [1,2,3,4,5]
#XXXXXXXXXXXXXXXXXXX
def fun8():
    a = l1[0]
    a1 = l1[1]
    a2 = l1[2]
    a3 = l1[3]
    a4 = l1[4]
#OOOOOOOOOOOOOOOOOOO
def fun9():
    a,a1,a2,a3,a4=l1



a = 0
for j in range(5):
    st = time.time()
    for i in range(1000000):
        # func(a)
        # a += 1
        # b = f2(8)
        # cc.re()
        # fun4()
        # fun5()
        # fun7(a)
        # fun8()
        fun9()
    et = time.time()
    print(et-st)