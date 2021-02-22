import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import VecUtils
from Artifact import Artifact

class Robot(Artifact):

    '''
    2 -> Circle
    Add further if robot needs structural changes!
    '''
    def getShape(self):
        return 2

    def getVelocity(self):
        return self.forward * (self.vleft + self.vright)/2
    
    def robotBootup(self):
        self.len = 250
        self.point = np.array([0,0])
        self.setPos(500, 500)
        self.normal = np.array([0,0])
        #robot's personal properties
        self.ICC = self.pos
        self.forward = np.array([1., 0.])
        self.sensorThreshold = 200
        self.size = 100.0
        self.vleft = 0.0
        self.vright = 0.0
        self.vnet = (self.vleft + self.vright)/2
        self.rsize = np.array([self.size, self.size])
        self.rotation = 0
        self.R = 0
        x_axis = np.array([1,0])
        self.theta = np.arccos(np.dot(self.forward, x_axis) /
                    np.linalg.norm(self.forward) * np.linalg.norm(x_axis))

        self.sensorsBootup()
        self.sensorDistances = np.zeros((len(self.sensors),))

    def __init__(self, assignId=True):
        super(Robot, self).__init__(assignId=assignId)
        self.robotBootup()

    # init or reset sensors
    def sensorsBootup(self):
        x_axis = self.forward
        self.sensors = []
        offset_angle = 30
        for sensor in range(12):
            sensor_angle = np.pi*(offset_angle * sensor)/180
            temp = np.array([np.cos(sensor_angle) * x_axis[0] - np.sin(sensor_angle)* x_axis[1],
                np.sin(sensor_angle) * x_axis[0] + np.cos(sensor_angle) * x_axis[1] ])
            self.sensors.append(self.pos + temp*self.rsize[0]/2)
    
    # set robot's position
    def setPos(self, x, y):
        self.pos = np.array([x,y])
    
    def getLinesIntersectionPt(self, a1, a2, b1, b2):
        intersection_pt = np.array([0,0])
        ptB = a2-a1
        ptD = b2-b1

        dotProduct = ptB[0]*ptD[1] - ptB[1]*ptD[0]
        # Case: Lines are Perpendicular
        if dotProduct == 0: 
            return None
        
        # Case: Projection of a1a2 on b1b2
        ptC = b1-a1
        t = (ptC[0] * ptD[1] - ptC[1]*ptD[0]) / dotProduct
        if t>1 or t<0:
            return None
        u = (ptC[0]*ptB[1] - ptC[1]*ptB[0])/dotProduct
        if u>1 or u<0:
            return None

        intersection_pt = a1 + t*ptB
        return [intersection_pt[0], intersection_pt[1]]
    
    def getLineRectIntersectionPt(self, pt_s, pt_e, rx,ry,rw,rh):
        intersection_pts_list = []
        rect_tl = np.array([rx - rw / 2, ry - rh / 2])
        rect_tr = np.array([rx + rw / 2, ry - rh / 2])
        rect_bl = np.array([rx - rw / 2, ry + rh / 2])
        rect_br = np.array([rx + rw / 2, ry + rh / 2])

        top = self.getLinesIntersectionPt(pt_s, pt_e, rect_tl, rect_tr)
        if top != None:
            intersection_pts_list.append(top)
        
        right = self.getLinesIntersectionPt(pt_s, pt_e, rect_tr, rect_br)
        if right != None:
            intersection_pts_list.append(right)
        
        bottom = self.getLinesIntersectionPt(pt_s, pt_e, rect_br, rect_bl)
        if bottom != None:
            intersection_pts_list.append(bottom)

        left = self.getLinesIntersectionPt(pt_s, pt_e, rect_bl, rect_tl)
        if left != None:
            intersection_pts_list.append(left)

        if len(intersection_pts_list)==0:
            return False, np.zeros((2,))
        else:
            intersection_pts = np.copy(intersection_pts_list)
            # find sensor distance from the robot's local coordinate's - starting point.
            dist = np.linalg.norm(intersection_pts - pt_s)
            # find the shortest distance
            minIndex = np.argmin(dist)
            return True, intersection_pts[minIndex]
    
    def checkForCollision(self, other):
        for ind, sensor in enumerate(self.sensors):
            normVec = VecUtils.normalizationByDivision(sensor-self.pos)
            subSensorThreshold = sensor + normVec*self.sensorThreshold

            interactDecision, intersectionPoint = self.getLineRectIntersectionPt(sensor, subSensorThreshold, 
                                                            other.pos[0], other.pos[1], other.rsize[0], other.rsize[1])

            if interactDecision:
                self.sensorDistances[ind] = np.linalg.norm(intersectionPoint - sensor) 
                
        return super(Robot, self).checkForCollision(other)

    
    def collisionHandling(self, obj, normX, normY, intersectionPoint):
        projection = np.array([0, 0])

        if normX != 0 and normY != 0:
            vec = np.array([0,0])
            self.vleft = self.vright = 0
            return True
        else:
            if normX != 0:
                if normX > 0:
                    vec = np.array([0, -1])
                else:
                    vec = np.array([0, 1])
            else:
                if normY > 0:
                    vec = np.array([-1,0])
                else:
                    vec = np.array([1, 0])
        vecPlane = vec
        projection = np.multiply(self.forward, vecPlane) / (vecPlane[0]**2 + vecPlane[1] ** 2)* vecPlane
        
        if projection[0] != 0:
            if projection[0] > 0:
                vec = np.array([1, 0])
            else:
                vec = np.array([-1, 0])
        else:
            if projection[1] > 0:
                vec = np.array([0,1])
            else:
                vec = np.array([0, -1])
        n = VecUtils.normalizationByDivision(intersectionPoint - self.pos)

        self.forward = VecUtils.normalizationByDivision(vec)
        prevTheta = np.rad2deg(self.theta)

        if prevTheta < 0:
            self.theta = -self.theta
        
        return True

    def getArtifactBoundingBox(self):
        delta = self.pos - self.prevPos
        box = Artifact.Box(self.prevPos[0], self.prevPos[1], delta[0] + self.rsize[0], delta[1]+self.rsize[1])
        return box
    
    def updateRobot(self, deltaTime):
        x_axis = np.array([1, 0])
        deltaV = self.vright - self.vleft
        if deltaV != 0:

            self.rotation = deltaV / self.len
            self.R = self.len/2 * (self.vright + self.vleft)/deltaV
            self.ICC = np.array([self.pos[0] - self.R * np.sin(self.theta),
                                 self.pos[1] - self.R * np.cos(self.theta)])
                                 #self.pos[1] + self.R * np.cos(self.theta)])

            odt = self.rotation # * deltaTime
            rotationalMatrix = np.array([[np.cos(odt), np.sin(odt), 0],
                                         [-np.sin(odt), np.cos(odt),  0],
                                         [0,            0,           1]])
            originMatrix = np.array([self.pos[0] - self.ICC[0],
                                     self.pos[1] - self.ICC[1],
                                     self.theta])
            prevLocationMatrix = np.array([self.ICC[0],
                                           self.ICC[1],
                                           odt])
            matrix = np.dot(rotationalMatrix,originMatrix) + prevLocationMatrix

            self.pos = np.array([matrix[0],matrix[1]])
            self.theta = matrix[2]

            self.forward = np.array(
                [np.cos(self.theta)*x_axis[0] + np.sin(self.theta)*x_axis[1],
                    -np.sin(self.theta) * x_axis[0] + np.cos(self.theta) * x_axis[1],
                ])

        else:
            self.forward = np.array(
                [np.cos(self.theta) * x_axis[0] + np.sin(self.theta) * x_axis[1],
                 -np.sin(self.theta) * x_axis[0] + np.cos(self.theta) * x_axis[1],
                 ])
            f = np.copy(self.forward)
            f/= np.linalg.norm(self.forward)

            self.rotation =0
            self.ICC = self.pos
            self.pos = self.pos + f * (self.vleft+self.vright)/2

        self.sensorsBootup()
        return True

    def drawArtifact(self,qp):
        #body
        origin = np.array([self.pos[0] - self.rsize[0]/2,self.pos[1] - self.rsize[0]/2])
        pen = QPen(Qt.red, 1.5, Qt.SolidLine)
        qp.setPen(pen)
        qp.drawEllipse(origin[0], origin[1], self.rsize[0], self.rsize[0])

        pen2 = QPen(Qt.red, 0.5, Qt.SolidLine)
        qp.setPen(pen2)
        f = self.forward 
        qp.drawLine(self.pos[0], self.pos[1], self.pos[0]+f[0]*self.size/2, self.pos[1]+f[1]*self.size/2)

        #ICC debug
        pen3 = QPen(Qt.magenta, 1.5, Qt.SolidLine)
        qp.setPen(pen3)
        qp.drawEllipse(self.ICC[0] - self.size/8, self.ICC[1] - self.size/8, self.size/4, self.size/4)

        pen6 = QPen(Qt.green, 5, Qt.SolidLine)
        qp.setPen(pen6)
        qp.drawPoint(self.point[0],self.point[1])


        pen = QPen(Qt.black, 10, Qt.SolidLine)
        qp.setPen(pen)

        #draw texts

        pVec = np.array([self.forward[1],-self.forward[0]])
        lefTextPos = self.pos + pVec * 50

        pen = QPen(Qt.blue, 1, Qt.SolidLine)
        qp.setPen(pen)

        #qp.drawPoint(textPos[0], textPos[1])
        qp.drawText(QPointF(lefTextPos[0] ,lefTextPos[1] ),str(self.vleft))

        rightTextPos = self.pos + pVec * -50
        pen = QPen(Qt.red, 1, Qt.SolidLine)
        qp.setPen(pen)

        qp.drawText(QPointF(rightTextPos[0], rightTextPos[1]), str(self.vright))


        for i,sensor in enumerate(self.sensors) :
            pen6 = QPen(Qt.darkCyan, 0.01, Qt.SolidLine)
            qp.setPen(pen6)
            norVec = VecUtils.normalizationByDivision(sensor - self.pos) *self.sensorThreshold
            qp.drawText(QPointF(sensor[0] +norVec[0], sensor[1]+norVec[1]), str("{:.1f}".format(self.sensorDistances[i])))

        return True