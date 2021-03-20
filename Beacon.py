import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class Beacon:
    counter = 0
    @staticmethod
    def generateID():
        index = Beacon.counter + 1
        Beacon.counter += 1
        return index

    def __init__(self, startPos, genId=True):
        if genId:
            self.id = Beacon.generateID()
        else:
            self.id = -1

        self.size = 10
        self.rsize = np.array([self.size, self.size])
        self.pos = np.copy(startPos)
        self.isDetected = False

    def Draw(self, pen):
        originBeacon = np.array([self.pos[0] - self.rsize[0], self.pos[1] - self.rsize[0]])
        if not self.isDetected:
            brush = QBrush(Qt.darkBlue)
        else: 
            brush = QBrush(Qt.darkGreen)
        pen.setBrush(brush)
        pen.drawEllipse(originBeacon[0], originBeacon[1], self.rsize[0]*2, self.rsize[0]*2)
        pen.drawText(self.pos[0], self.pos[1], str(self.id))