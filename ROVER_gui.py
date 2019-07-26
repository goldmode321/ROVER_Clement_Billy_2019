import sys
import os
from PyQt5 import QtWidgets, QtCore, QtGui
import gui.ui_rover as ui_rover
import keyboard
import time

class ROVER_gui():
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        MainWindow = QtWidgets.QMainWindow()
        self.gui = ui_rover.Ui_MainWindow()
        self.gui.setupUi(MainWindow)


        self.gui.EmergencyStopBtn.clicked.connect(self.EmergencyStopBtn_click)
        self.gui.KeyboardControlBtn.clicked.connect(self.KeyboardControlBtn_click)
        self.gui.BuildMapBtn.clicked.connect(self.BuildMapBtn_click)
        self.gui.GetLidarDataBtn.clicked.connect(self.GetLidarDataBtn_click)
        self.gui.BuildMapBtn.clicked.connect(self.BuildMapBtn_click)
        
        
        


        self.Keyboard_Control_Mode = False

        MainWindow.show()
        sys.exit(app.exec_())




    def EmergencyStopBtn_click(self):
        pass

    def KeyboardControlBtn_click(self):

        def controller():
            if keyboard.is_pressed('up'):
                self.gui.Key_up.setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.gui.Key_up.setStyleSheet("background-color: rgb(255, 255, 255);")
            
            if keyboard.is_pressed('down'):
                self.gui.Key_down.setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.gui.Key_down.setStyleSheet("background-color: rgb(255, 255, 255);")

            if keyboard.is_pressed('left'):
                self.gui.Key_left.setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.gui.Key_left.setStyleSheet("background-color: rgb(255, 255, 255);")
            
            if keyboard.is_pressed('right'):
                self.gui.Key_right.setStyleSheet("background-color: rgb(0, 255, 0);")
            else:
                self.gui.Key_right.setStyleSheet("background-color: rgb(255, 255, 255);")

        self.KeyboardControlTimer = QtCore.QTimer()
        self.KeyboardControlTimer.timeout.connect(controller)            

        if self.Keyboard_Control_Mode == False:
            self.gui.console_1.setText('KeyBoard Control')
            self.KeyboardControlTimer.start(10)
            self.Keyboard_Control_Mode = True
        else:
            self.KeyboardControlTimer.stop()
            self.Keyboard_Control_Mode = False
                    
            
    def WayPointBtn_click(self):
        pass


    def GetLidarDataBtn_click(self):
        pass

    def BuildMapBtn_click(self):
        pass


if __name__ == "__main__":
    ROVER_gui()