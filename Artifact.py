import abc
import numpy as np
import VecUtils
import math
from PyQt5 import QtCore, QtGui, QtWidgets

class Artifact:
    idCounter = 0       # IDs in case we have more than one robot ahead!
    @staticmethod
    def idGenerator(self):
        i = self.idCounter + 1
        self.idCounter += 1
        return i

    #constructor
    def __init__(self, parent=None, assignId=True, collisionArtifactShape = 1):
        if assignId:
            self.id = self.idGenerator(self)
        else:
            self.id = -1

        self.shape = collisionArtifactShape #define shape
        self.pos = np.array([0,0])  #define position to initialize at
        self.forward = np.array([1, 0]) #right
        self.rsize = np.array([0,0])
    
    @abc.abstractmethod
    def drawArtifact(self, qp):
         return True
    
    @abc.abstractmethod
    def getShape(self):
        #Putting up square by default
        return self.shape

    # Abstract method for collision handling    
    @abc.abstractmethod
    def collisionHandling(self, obj, normX, normY, intersectionPoint):
        return True

    @abc.abstractmethod
    def checkForCollision(self, other):
        collisionFlag, normal, intersectionPoint = self.quickCheck(self, other)
        if collisionFlag:
            self.collisionHandling(other, normal[0], normal[1], intersectionPoint)
            other.collisionHandling(self, -normal[0], -normal[1], intersectionPoint)

        return collisionFlag
    
    def quickCheck(self, a, b):
        collisionFlag = True
        selfShape = a.getShape()
        otherShape = b.getShape()
        if selfShape == 2 and otherShape == 1:  # Rect and Circle
            collisionFlag, normal, intersectionPoint = self.circleRectIntersection(a.pos[0], a.pos[1], a.rsize[0], b.pos[0], b.pos[1], b.rsize[0], b.rsize[1])
        elif selfShape == 2 and otherShape ==2: # Circle and Circle
            isCollision,normal,contractPoint= self.circleCircle(a,b)
        
        normal = VecUtils.normalize(normal)
        self.normal - normal

        return collisionFlag, normal, intersectionPoint


    def circleRectIntersection(self, cx, cy, r, rx, ry, rw, rh):
        pointCX = cx
        pointCY = cy

        rLeft = rx - rw/2
        rRight= rx + rw/2
        rTop = ry - rh/2
        rBot = ry + rh/2

        if pointCX<rLeft:
            pointCX = rLeft
        elif pointCX > rRight:
            pointCX = rRight
        if pointCY < rTop:
            pointCY =rTop
        elif pointCY> rBot:
            pointCY = rBot
        self.point = np.array([pointCX,pointCY])
        dx = cx - pointCX
        dy = cy - pointCY

        d = np.array([dx,dy])
        temp = math.sqrt(dx**2 + dy**2)

        if temp <= r/2:
            return True, np.array([cx-pointCX,cy-pointCY]), np.array([pointCX,pointCY])
        else:
            return False, np.array([0,0]), np.array([0,0])

    # Adding the below, because the flow is on. It is not being used right now.
    def circleCircleIntersection(self,a,b):
        dx = a.pos[0] - b.pos[0]
        dy = a.pos[1] - b.pos[1]
        d = np.array([dx, dy])
        temp = math.sqrt(dx ** 2 + dy ** 2)

        if temp <= a.rsize[0] + b.rsize[0]:
            return True,np.array([b.pos[0]-(a.pos[0]+a.rsize[0]),b.pos[1]-(a.pos[1]+a.rsize[1])]),(a.pos + b.pos)/2
        else:
            return False,np.array([0,0]),np.array([0,0])
