from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets

class LidarMap(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()

        self.lidar_axes = self.fig.add_subplot(111, projection='polar')
        self.lidar_axes.set_ylim(0, 2000)

        FigureCanvas.__init__(self, self.fig)

        QtWidgets.QWidget.__init__(self, parent, figure=self.figure)
        self.setParent(parent)
        # vertical_layout = QtWidgets.QVBoxLayout()
        # vertical_layout.addWidget(self.fig)
        FigureCanvas.updateGeometry(self)


# class LidarMap(QtWidgets.QWidget):
#     def __init__(self, parent=None):
#         QtWidgets.QWidget.__init__(self, parent)
#         self.canvas = FigureCanvas(Figure())
#         vertical_layout = QtWidgets.QVBoxLayout()
#         vertical_layout.addWidget(self.canvas)
#         self.canvas.axes = self.canvas.figure.add_subplot(1, 1, 1, projection='polar')
#         self.setLayout(vertical_layout)

#     def clear(self):
#         self.canvas.axes.clear()
    
#     def plot(self, lidar_angle=[0], lidar_radius=[0]):
#         self.canvas.axes.plot(lidar_angle, lidar_radius, '.')
#         self.canvas.axes.set_ylim(0, 2000)
    
#     def legend(self, legend=[""]):
#         self.canvas.axes.legend((x for x in legend), loc='upper right')

#     def show_plot(self):
#         self.canvas.draw()
