from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QPropertyAnimation, pyqtProperty
import sys

class Warring(QLabel):

    def __init__(self):
        super().__init__()

    def _set_color(self, col):
        
        palette = self.palette()
        palette.setColor(self.foregroundRole(), col)
        self.setPalette(palette)

    color = pyqtProperty(QColor, fset=_set_color)

