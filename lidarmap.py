from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets

class LidarMap(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()
        self.lidar_axes = self.fig.add_subplot(111, projection='polar')
        self.lidar_axes.set_ylim(0, 2000)
        self.lidar_axes.set_theta_zero_location("N")


        FigureCanvas.__init__(self, self.fig)
        QtWidgets.QWidget.__init__(self, parent, figure=self.figure)
        self.setParent(parent)
        FigureCanvas.updateGeometry(self)

