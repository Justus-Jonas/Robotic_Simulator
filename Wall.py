import numpy as np
from Artifact import Artifact
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Wall(Artifact):
    def __init__(self, x_pos, y_pos, width, height):
        super(Wall, self).__init__()
        self.rsize = np.array([width, height])
        self.pos = np.array([x_pos, y_pos])
        self.normal = np.array([0, 0])

    '''Wall is stationary, be it any structure
    But we need it as a generic Artefact's obj,
    therefore velocity component.'''
    def getVelocity(self):
        return np.array([0,0])

    '''
    1 -> Square
    2 -> Circle (currently for Robot)
    3 -> add further wall shapes as need be
    '''
    def getShape(self):
        return 1
    
    def drawArtifact(self, painter):
        start = np.array([self.pos[0] - self.rsize[0]/2, self.pos[1] - self.rsize[1]/2])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        painter.setPen(pen)
        painter.drawRect(start[0], start[1], self.rsize[0], self.rsize[1])