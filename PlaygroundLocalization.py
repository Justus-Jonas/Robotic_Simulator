from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from RobotLocalization import LRobot
from Beacon import Beacon
from KalmanFilter import KalmanFilter
import time
import VecUtils
import sys
import numpy as np
import Mapper as Map

class PlaygroundLocalization(QMainWindow):
    SCREEN_WIDTH = 1080
    SCREEN_HEIGHT = 720
    MAX_SPEED = 5

    def __init__(self):
        self.initGround()
        super(PlaygroundLocalization, self).__init__()
        self.initUIComponents()

    def initUIComponents(self):
        self.timer = QBasicTimer()
        self.lastFrameTime =0
        self.resize(self.SCREEN_WIDTH, self.SCREEN_HEIGHT)
        self.setWindowTitle('Mobile Robot Localization')
        self.show()
        self.timer.start(1000.0 / 60, self)

    def initGround(self):
        self.beacons = []
        self.lines = []
        self.timeCounter=0
        self.timeDelayDrawing = 2
        #beacons pos
        self.startPos,self.beacons,self.lines = Map.genAssignmentMap()
        self.pRobot = LRobot(self.startPos, Qt.cyan, True)
        self.cRobot = LRobot(self.startPos, Qt.blue, True)
        self.robot = LRobot(self.startPos)
        self.lastMean = np.array([self.robot.pos[0],self.robot.pos[1],self.robot.theta])
        self.lastCovariance = np.array([0.1,0.1,0.1])


    def keyPressEvent(self, event):
        key = event.key()

        if key == Qt.Key_W:
            self.robot.v +=1.0
        elif key == Qt.Key_S:
            self.robot.v -=1.0
        elif  key == Qt.Key_O:
            self.robot.rotation -=0.05
        elif key == Qt.Key_L:
            self.robot.rotation +=0.05
        elif key == Qt.Key_X:
            self.robot.rotation =0.0
            self.robot.v =0.0

        if key == Qt.Key_C:
            self.timeCounter = 0
            self.robot.closePaths()
            self.robot.Reset(startPos= self.startPos)
            self.cRobot.Reset(startPos= self.startPos)
            self.pRobot.Reset(startPos= self.startPos)
            self.lastMean = np.array([self.robot.pos[0], self.robot.pos[1], self.robot.theta])
            self.lastCovariance = np.array([0.1, 0.1, 0.1])
            return None

    def playgroundUpdateFlow(self,dt):

        self.timeCounter += dt

        #update beacons
        self.robot.features = []
        self.robot.beacons_pos_in_area = []
        for beacon in self.beacons:
            beacon.isDetected = self.robot.UpdateSensor(beacon)


        pMean,pCovariance,mean,covariance = KalmanFilter(dt,
                     self.robot.theta,
                     self.lastMean,
                     self.lastCovariance,
                     np.array([self.robot.v,self.robot.rotation]),
                     self.robot.GetMeasurement()
                     )
        self.lastMean = np.copy(mean)
        self.lastCovariance = np.copy(covariance)
        self.cRobot.UpdateTransform([mean[0], mean[1]], mean[2])
        print("estimated pos = "+str([mean[0],mean[1]]))

        if self.timeCounter > self.timeDelayDrawing:
            self.timeCounter =0
            self.robot.AddEstimatedPath(mean, covariance, True)
        else:
            self.robot.AddEstimatedPath(mean, covariance, False)

        # calculate transform 
        pos, theta = self.robot.CalculateTransform(dt)
        # apply them using that func
        self.robot.UpdateTransform(pos, theta)

    def Draw(self,qpainter):
        self.robot.Draw(qpainter)
        self.cRobot.Draw(qpainter)

        pen = QPen(Qt.black, 1.5, Qt.SolidLine)
        qpainter.setPen(pen)
        for line in self.lines:
            p1 = self.beacons[line[0]-1].pos
            p2 = self.beacons[line[1]-1].pos
            qpainter.drawLine(p1[0],p1[1],p2[0],p2[1])
        for beacon in self.beacons:
            beacon.Draw(qpainter)

        for beacon in self.robot.beacons_pos_in_area:
            qpainter.setPen(QPen(Qt.red, 1, Qt.DashLine))
            qpainter.drawLine(QPointF(beacon[0], beacon[1]), (QPointF(self.robot.pos[0], self.robot.pos[1])))

        pen = QPen(Qt.black, 1.5, Qt.SolidLine)
        qpainter.setPen(pen)


    def timerEvent(self, event):
        if event.timerId() == self.timer.timerId():
            currentTime = time.time()
            dt = (currentTime - self.lastFrameTime)
            self.lastFrameTime = currentTime
            self.playgroundUpdateFlow(dt)
            self.update()
        else:
            super(PlaygroundLocalization, self).timerEvent(event)

    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        self.Draw(qp)
        qp.end()

if __name__ == '__main__':
    QApplication.processEvents()

    app = QApplication([])
    game = PlaygroundLocalization()

    sys.exit(app.exec_())
