from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets

class Calibration(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure()
        self.calibration_map_axes = self.fig.add_subplot(111)
        self.calibration_map_axes.autoscale(True)
        # self.fig.legend("Initial", "Current", "Calibrated")
        # self.global_map_axes.autoscale()



        FigureCanvas.__init__(self, self.fig)
        QtWidgets.QWidget.__init__(self, parent, figure=self.figure)
        self.setParent(parent)
        FigureCanvas.updateGeometry(self)

