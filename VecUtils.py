import numpy as np

SCREEN_WIDTH = 1080
SCREEN_HEIGHT = 720

def normalizationByDivision(vector):
    normVal = np.linalg.norm(vector)
    normVec = np.copy(vector)

    if normVal != 0:
        normVec[0] = normVec[0]/normVal
        normVec[1] = normVec[1]/normVal
        return normVec
    else:
        return vector

def normalize(vector):
    normVec = np.array([0,0])
    if vector[0] < 0:
        normVec[0] = -1
    elif vector[0] > 0:
        normVec[0] = 1
    else:
        normVec[0] = 0

    if vector[1] < 0:
        normVec[1] = -1
    elif vector[1] > 0:
        normVec[1] = 1
    else: 
        normVec[1] = 0
    return normVec
