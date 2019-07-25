# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rover.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

import gui.ui_test as ui 
import sys
import os
from PyQt5 import QtWidgets, QtCore, QtGui

import time

class test_gui():
    def __init__(self):
        os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
        app = QtWidgets.QApplication(sys.argv)
        app.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)



        MainWindow = QtWidgets.QMainWindow()
        
        self.gui = ui.Ui_MainWindow()
        self.gui.setupUi(MainWindow)
        self.gui.pushButton.clicked.connect(self.btn_click)
        self.i = 0
        

        
        #GUI Start
        MainWindow.show()
        
        sys.exit(app.exec_())


    def btn_click(self):
        self.i = 0
        # while self.i < 255:
        #     self.i += 1
        #     print('button click')
        #     self.gui.tb1.setText('button click '+str(self.i))
        #     self.gui.tb1.setStyleSheet("background-color: rgb( 0 , {} ,0); ".format(self.i))
        #     QtCore.QTimer.start(100)

        def change_color():
            self.i += 1
            # print('button click')
            self.gui.tb1.setText('button click '+str(self.i))
            self.gui.tb1.setStyleSheet("background-color: rgb( 0 , {} ,0); ".format(self.i))
            if self.i > 100:
                self.Qtimer.stop()    
        
        self.Qtimer = QtCore.QTimer()
        self.Qtimer.timeout.connect(change_color)
        self.Qtimer.start(100)
        print('timer stop')
        

if __name__ == "__main__":
    test_gui()

