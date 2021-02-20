import abc
import numpy as np
import VecUtils
import math
from PyQt5 import QtCore, QtGui, QtWidgets

class Artifact:
    idCounter = 0
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

        self.shape = collisionArtifactShape
        self.pos = np.array([0,0])
        self.forward = np.array([1, 0]) #right
        self.rsize = np.array([0,0])
    
    @abc.abstractmethod
    def drawArtifact(self, qp):
         return True
    
    @abc.abstractmethod
    def getShape(self):
        #Putting up square by default
        return self.shape
    
    @abc.abstractmethod
    def collisionHandling(self, obj, normX, normY, intersectionPoint):
        return True

    def AABB(self,obj):
        b1 = self.pos
        b1Width = self.rsize[0]
        b1Height = self.rsize[1]

        myVeloc= self.getVelocity()
        obVeloc= obj.getVelocity()

        b1vx = myVeloc[0] - obVeloc[0]
        b1vy = myVeloc[1] - obVeloc[1]

        b2 = obj.pos
        b2Width = obj.rsize[0]
        b2Height = obj.rsize[1]

        if b1vx > 0.0:
            xInvEntry = b2[0] - b1[0] + b1Width
            xInvExit = b2[0] + b2Width - b1[0]
        else:
            xInvEntry = b2[0] + b2Width - b1[0]
            xInvExit = b2[0] - (b1[0]+b1Width)

        if b1vy > 0.0:
            yInvEntry = b2[1] - (b1[1]+b1Height)
            yInvExit = b2[1]+b2Height - b1[1]
        else:
            yInvEntry = b2[1] + b2Height - b1[1]
            yInvExit = b2[1] - (b1[1]+b1Height)

        if b1vx == 0.0:
            xEntry = -math.inf
            xExit = math.inf
        else:
            xEntry = xInvEntry/b1vx
            xExit = xInvExit/b1vx

        if b1vy == 0.0:
            yEntry = -math.inf
            yExit = math.inf
        else:
            yEntry = yInvEntry/b1vy
            yExit = yInvExit/b1vy

        if xEntry > yEntry:
            entryTime = xEntry
        else:
            entryTime = yEntry

        if xExit < yExit:
            exitTime = xExit
        else:
            exitTime = yExit

        if entryTime > exitTime or xEntry < 0.0 and yEntry < 0.0 or xEntry > 1.0 or yEntry > 1.0:
            normalx = 0.0
            normaly = 0.0
            return 1.0,normalx,normaly
        else:
            if xEntry > yEntry:
                if xInvEntry <0:
                    normalx = 1.0
                    normaly = 0.0
                else:
                    normalx = -1.0
                    normaly = 0.0
            else:
                if yInvEntry <0:
                    normalx = 0.0
                    normaly = 1.0
                else:
                    normalx = 0.0
                    normaly = -1.0
            return entryTime,normalx,normaly

    @abc.abstractmethod
    def checkForCollision(self, other):
        collisionFlag, normal, intersectionPoint = self.quickCheck2(self, other)
        if collisionFlag:
            self.collisionHandling(other, normal[0], normal[1], intersectionPoint)
            other.collisionHandling(self, -normal[0], -normal[1], intersectionPoint)

        return collisionFlag
    
    def quickCheck2(self, a, b):
        collisionFlag = True
        selfShape = a.getShape()
        otherShape = b.getShape()
        if selfShape == 2 and otherShape == 1:
            collisionFlag, normal, intersectionPoint = self.circleRectIntersection(a.pos[0], a.pos[1], a.rsize[0], b.pos[0], b.pos[1], b.rsize[0], b.rsize[1])
        elif selfShape ==1 and otherShape ==1:
            isCollision,normal,contractPoint= self.rectRect(a,b)
        elif selfShape == 2 and otherShape ==2:
            isCollision,normal,contractPoint= self.circleCircle(a,b)
        
        normal = VecUtils.normalize(normal)
        self.normal - normal

        return collisionFlag, normal, intersectionPoint


    def circleRectIntersection(self, cx, cy, rad, rx, ry, rw, rh):
        px = cx
        py = cy

        rLeft = rx - rw/2
        rRight= rx + rw/2
        rTop = ry - rh/2
        rBot = ry + rh/2

        if px<rLeft:
            px = rLeft
        elif px > rRight:
            px = rRight
        if py < rTop:
            py =rTop
        elif py> rBot:
            py = rBot
        self.point = np.array([px,py])
        dx = cx - px
        dy = cy - py

        d = np.array([dx,dy])
        temp = math.sqrt(dx**2 + dy**2)
        #temp = dx**2 + dy**2

        #dRad = rad**2

        if temp <= rad/2:
            return True,np.array([cx-px,cy-py]),np.array([px,py])
        else:
            return False,np.array([0,0]),np.array([0,0])

    # Adding the two below, because the flow is on. They are not being used right now.
    def circleCircleIntersection(self,a,b):
        dx = a.pos[0] - b.pos[0]
        dy = a.pos[1] - b.pos[1]
        d = np.array([dx, dy])
        temp = math.sqrt(dx ** 2 + dy ** 2)

        if temp <= a.rsize[0] + b.rsize[0]:
            return True,np.array([b.pos[0]-(a.pos[0]+a.rsize[0]),b.pos[1]-(a.pos[1]+a.rsize[1])]),(a.pos + b.pos)/2
        else:
            return False,np.array([0,0]),np.array([0,0])

    def rectRectIntersection(self, a, b):
        temp = not(a.pos[0] + a.rsize[0] < b.pos[0]
            or a.pos[0] > b.pos[0] + b.rsize[0]
            or a.pos[1] + a.rsize[1] < b.pos[1]
            or a.pos[1] > b.pos[1] + b.rsize[1] ),np.array([0,0])

        if temp:
            colTime, normalx, normaly =a.AABB(b)
            if colTime <1:
                return True,np.array([normalx,normaly]),np.array([0,0]),(a.pos + b.pos)/2

        return False,np.array([0,0]),np.array([0,0])
    