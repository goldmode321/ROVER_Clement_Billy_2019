from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets

class GlobalMap(FigureCanvas):
    def __init__(self, parent=None):
        self.parent = parent
        self.FigureCanvas = FigureCanvas


        self.fig = Figure()
        self.global_map_axes = self.fig.add_subplot(111)
        # self.global_map_axes.autoscale()



        FigureCanvas.__init__(self, self.fig)
        QtWidgets.QWidget.__init__(self, parent, figure=self.figure)
        self.setParent(parent)
        FigureCanvas.updateGeometry(self)

    # def reinitialize(self):
        
        # self.fig = Figure()
        # self.global_map_axes = self.fig.add_subplot(111)
        # self.setParent(self.parent)
        # FigureCanvas.updateGeometry(self)
