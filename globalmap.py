from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvas
from PyQt5 import QtWidgets

class GlobalMap(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        self.canvas = FigureCanvas(Figure())
        vertical_layout = QtWidgets.QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        self.canvas.axes = self.canvas.figure.add_subplot(1,1,1)
        self.setLayout(vertical_layout)
    
    def clear(self):
        self.canvas.axes.clear()
    
    def plot(self, x=[0], y=[0], style='bo'):
        self.canvas.axes.plot(x, y, style)

    def plot_arrow(self, x=0, y=0, dx=1, dy=1, width=30):
        self.canvas.axes.arrow(x, y, dx, dy, width=width)

    def draw(self):
        self.canvas.draw()

