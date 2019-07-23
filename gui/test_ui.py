# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'rover.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

import gui.ui_test as ui 
from PyQt5 import QtWidgets, QtCore, QtGui

class test_gui():
    def __init__(self):
        MainWindow = QtWidgets.QMainWindow()
        self.gui = ui.Ui_MainWindow()
        self.gui.pushButton.clicked.connect(self.btn_click)
        self.i = 0
        #GUI Start
        MainWindow.show()


    def btn_click(self):
        self.i += 1
        print('button click')
        self.gui.tb1.setText('button click '+str(self.i))




