# ------------------------------------------------------
# -------------------- mplwidget.py --------------------
# ------------------------------------------------------
#from PyQt5.QtWidgets import*
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure
from PyQt5 import QtWidgets

#class MplWidget():
#class MplWidget(QWidget):
class MplWidget(QtWidgets.QWidget):

    def __init__(self, parent = None):
#        self.qwidget = QtWidgets.QWidget(parent)
#        QWidget.__init__(self, parent)
        QtWidgets.QWidget.__init__(self, parent)
        self.canvas = FigureCanvas(Figure())
        self.canvas2 = FigureCanvas(Figure())

#        vertical_layout = QVBoxLayout()
        vertical_layout = QtWidgets.QVBoxLayout()
        vertical_layout.addWidget(self.canvas)

        self.canvas.axes = self.canvas.figure.add_subplot(2,1,1, projection='polar')
        self.canvas2.axes = self.canvas.figure.add_subplot(2,1,2)
        self.setLayout(vertical_layout)
