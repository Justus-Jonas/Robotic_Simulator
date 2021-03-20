import VecUtils
import numpy as np
import math

A = np.identity(3)
C = np.identity(3)

def KalmanFilter(dt, theta, lastMean, lastCovariance, u, z):

    sigmaMotion = np.array([250, 10, 20])
    sigmaSensor = np.array([100,50,0.1])

    B = np.array([[dt*np.cos(theta), 0],[dt*np.sin(theta, 0)],[0, dt]])
    motion = np.identity(3)*sigmaMotion
    sensor = np.identity(3)*sigmaSensor

    #prediction stuff
    predMean = np.dot(A, lastMean) + np.dot(B, u)
    predCovariance = np.dot(np.dot(A, lastCovariance), A.T) + motion

    #correction stuff
    mulMat = np.dot(np.dot(C, predCovariance), C.T)
    inverseMat = np.linalg.inv(mulMat + Q)
    K = np.dot(np.dot(predCovariance, C.T), inverseMat)
    mean = predMean + np.dot(K, (z-np.dot(C, predMean)))
    covariance = np.dot((np.identity(3) - np.dot(K, C)), predCovariance)

    if math.isnan(mean[0]):
        print("gotta change something")

    return predMean, predCovariance, mean, covariance