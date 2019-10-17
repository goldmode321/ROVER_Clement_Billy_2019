# ------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------
from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi

from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
#import matplotlib.pyplot as plt
import test_ui as tui
import numpy as np
import random
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
class MatplotlibWidget():

    def __init__(self):

#        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
#        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        MainWindow = QtWidgets.QMainWindow()
        self.gui = tui.Ui_MainWindow()
        self.gui.setupUi(MainWindow)
        self.MplWidget = self.gui.MplWidget

        MainWindow.setWindowTitle("PyQt5 & Matplotlib Example GUI")

#        self.gui.pushButton_generate_random_signal.clicked.connect(self.update_graph)

#        self.data, = self.MplWidget.canvas.axes.plot(0,0)
#        self.gui.pushButton_generate_random_signal.clicked.connect(self.update_graph)
        self.gui.pushButton_generate_random_signal.clicked.connect(self.stop)
        self.timer1 = QtCore.QTimer()
        self.timer1.timeout.connect(self.update_graph)
        self.timer1.start(100)

#        MainWindow.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))
        MainWindow.show()

        sys.exit(app.exec_())

    def stop(self):
#        self.MplWidget.canvas.flush_events()
        self.timer1.stop()

    def update_graph(self):

        fs = 500
        f = random.randint(1, 100)
        ts = 1/fs
        length_of_signal = 10000
        t = np.linspace(0,1,length_of_signal)

        cosinus_signal = 1000*np.cos(2*np.pi*f*t)
        sinus_signal = np.sin(2*np.pi*f*t)

        self.MplWidget.canvas.axes.cla()
#        self.data.set_ydata(cosinus_signal)
#        self.MplWidget.canvas.draw_artist(self.data)
#        self.MplWidget.canvas2.axes.cla()
        self.MplWidget.canvas2.axes.plot(t, cosinus_signal)
        self.MplWidget.canvas.axes.plot(t, cosinus_signal)
#        self.MplWidget.canvas.axes.plot(t, sinus_signal)
#        self.MplWidget.canvas.axes.legend(('cosinus', 'sinus'),loc='upper right')
#        self.MplWidget.canvas.axes.set_title('Cosinus - Sinus Signal')
        self.MplWidget.canvas.draw()
        self.MplWidget.canvas2.draw()
#        self.MplWidget.canvas.update()
#        self.MplWidget.canvas2.update()
#        self.MplWidget.canvas.flush_events()
#        self.MplWidget.canvas2.flush_events()
#        plt.pause(0.001)
        self.MplWidget.canvas.start_event_loop(0.001)
        self.MplWidget.canvas2.start_event_loop(0.01)


MatplotlibWidget()
#app.exec_()
