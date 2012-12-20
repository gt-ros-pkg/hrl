import math
import random
import numpy as np
import scipy as scp
import scipy.ndimage as ni

def dist(point1, point2):
    d = (np.sum((point1 - point2)**2))**(.5)
    return d

def validation(aPoint, objects):
    collisionObjectKey = collisionDetection(aPoint, objects)
    isValidPoint = np.isnan(collisionObjectKey)
    return isValidPoint

def extension(point1, point2, extendPrct, objects):
    nSteps = 50.
    d = dist(point1, point2)
    extendDir = (point2 - point1) / d
    isValidPoint = True
    iStep = 0.
    aPoint = point1
    while (iStep < nSteps):
        iStep = iStep + 1
        newPoint = point1 + extendDir*d*iStep/nSteps*(extendPrct/100.)
        isValidPoint = validation(newPoint, objects)
        if (not(isValidPoint)):
            break
        aPoint = newPoint
    return aPoint

def collisionDetection(aPoint, objects):
    collisionObjectKey = float('nan')
    for key in objects:
        radius = objects[key][0]
        oPoint = np.array(objects[key][1:])
        if (dist(aPoint, oPoint) <= radius):
            collisionObjectKey = key
            break
    return collisionObjectKey
