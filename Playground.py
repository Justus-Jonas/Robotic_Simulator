import sys
import random
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from Artifact import Artifact
from Sensor import Sensor, SensorComputation
from Wall import Wall
from Robot import Robot
import numpy as np
import math

class Playground(QMainWindow):
    SCREEN_WIDTH = 1080
    SCREEN_HEIGHT = 720

    def __init__(self):
        super(Playground, self).__init__()
        self.initGround()
        self.initUIComponents()

    def getCenter(self):
        return (self.SCREEN_WIDTH/2, self.SCREEN_HEIGHT/2)

    def initGround(self):
        self.walls = []
        self.robot = Robot()
        centerX, centerY = self.getCenter()
        # rectangle box
        # self.walls.append(Wall(centerX,centerY,100,100))
        self.walls.append(Wall(centerX-self.SCREEN_WIDTH/2, centerY, 20, self.SCREEN_HEIGHT))
        self.walls.append(Wall(centerX+self.SCREEN_WIDTH/2, centerY, 20, self.SCREEN_HEIGHT))
        self.walls.append(Wall(centerX, centerY-self.SCREEN_HEIGHT/2, self.SCREEN_WIDTH, 20))
        self.walls.append(Wall(centerX, centerY+self.SCREEN_HEIGHT/2, self.SCREEN_WIDTH, 20))

        self.doPress = False
        self.collisionFlag = False
        self.timer = QBasicTimer()
        self.prevPos = self.robot.pos

    def initUIComponents(self):
        layoutHorizontal = QHBoxLayout(self)
        layoutVertical = QVBoxLayout(self)
        layoutVertical.addLayout(layoutHorizontal)
        self.setLayout(layoutVertical)
        self.resize(1080, 720)
        self.setWindowTitle('D-D-D-D-D... DJ ROOMBA')
        self.show()
        self.lastFrameTime = time.time()
        self.timer.start(17, self)

    def fill_map(self, wall, maps):
        for x in range(wall.pos[0], wall.pos[0] + wall.rsize[0]):
            maps[x][wall.pos[1]] = True
        for y in range(wall.pos[1], wall.pos[1] + wall.rsize[1]):
            maps[wall.pos[0]][y] = True
        return maps

    def playgroundUpdateFlow(self, deltaTime):
        self.robot.updateRobot(deltaTime)
        self.collisionFlag = False
        self.robot.sensorDistances = np.zeros((len(self.robot.sensors),))
        maps = np.full((self.SCREEN_HEIGHT, self.SCREEN_WIDTH), None)
        compute = SensorComputation(0, maps, self.robot.len)
        for wall in self.walls:
            maps = self.fill_map(wall, maps)

        list_of_distances = compute.update_distance(math.degrees(self.robot.theta), maps, self.robot.pos[0], self.robot.pos[1])
        print(list_of_distances)

        if self.collisionFlag:
            self.robot.pos = self.prevPos
        
        self.prevPos = self.robot.pos

        return True
    
    '''
    Key events - 
    1) 'w': left wheel + 5
    2) 's': left wheel - 5
    3) 'o': right wheel + 5
    4) 'l': right wheel - 5
    5) 'x': both wheels = 0
    '''
    def keyPressEvent(self, event):
        key = event.key()

        prevVleft = self.robot.vleft
        prevVright = self.robot.vright

        if key == Qt.Key_W:
            self.doPress = True
            self.robot.vleft += 1

        elif key == Qt.Key_S:
            self.doPress = True
            self.robot.vleft += -1
        
        elif key == Qt.Key_O:
            self.doPress = True
            self.robot.vright += 1

        elif key == Qt.Key_L:
            self.doPress = True
            self.robot.vright += -1
        
        elif key == Qt.Key_X:
            self.robot.vright = 0
            self.robot.vleft = 0
        
        if self.doPress:
            self.doPress = False
        
    def paintEvent(self, e):
        painter = QPainter()
        painter.begin(self)

        # self.drawLines(painter)
        self.robot.drawArtifact(painter)
        for wall in self.walls:
            wall.drawArtifact(painter)

        painter.end()

    # def drawLines(self, qp):      
    #     pen = QPen(Qt.black, 1.5, Qt.SolidLine)
    #     qp.setPen(pen)
    #     qp.drawRect(4,4,self.SCREEN_WIDTH,self.SCREEN_HEIGHT)
       
    def timerEvent(self, event):
        if event.timerId() == self.timer.timerId():
            currentTime = time.time()
            dt = (currentTime - self.lastFrameTime)
            self.lastFrameTime = currentTime
            self.playgroundUpdateFlow(dt)
            self.update()
        else:
            super(MRS, self).timerEvent(event)


class Worker(QThread):
    def __init__(self, game):
        QThread.__init__(self)
        self.game = game

    FPS = 60
    gameIsRunning = True
    lastFrameTime = 0

    def run(self):
        while self.gameIsRunning:
            currTime = time.time()
            deltaTime = (currTime - self.lastFrameTime)
            self.lastFrameTime = currTime
            sleepTime = 1.0/self.FPS - deltaTime
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.game.playgroundUpdateFlow(deltaTime)

            self.game.updateRobot()

if __name__ == '__main__':
    QApplication.processEvents()

    app = QApplication([])
    game = Playground()
    
    sys.exit(app.exec_())
