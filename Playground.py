import sys
import random
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from Artifact import Artifact
from Wall import Wall
from Robot import Robot
import numpy as np

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

    def playgroundUpdateFlow(self, deltaTime):
        self.robot.updateRobot(deltaTime)
        self.collisionFlag = False
        self.robot.sensorDistances = np.zeros((len(self.robot.sensors),))
        for wall in self.walls:
            temp = self.robot.checkForCollision(wall)
            if self.collisionFlag == False:
                self.collisionFlag = temp
#            if temp:
#                # adjust theta
#                self.robot.theta =  # wall theta
#
#                # adjust velocity
#                vx = self.robot.getVelocity() * np.cos(self.robot.theta)
#                vy = self.robot.getVelocity() * np.sin(self.robot.theta)
#                v = np.sqrt(vx**2 + vy**2)
#                self.robot.vright = v
#                self.robot.vleft = v
        
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
            self.robot.vleft += 5

        elif key == Qt.Key_S:
            self.doPress = True
            self.robot.vleft += -5
        
        elif key == Qt.Key_O:
            self.doPress = True
            self.robot.vright += 5

        elif key == Qt.Key_L:
            self.doPress = True
            self.robot.vright += -5
        
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