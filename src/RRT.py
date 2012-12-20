import math
import random
import numpy as np
import scipy as scp
import scipy.ndimage as ni

# RRT function
def RRT(initPos, goalPos, objects, limits, distFunc, validationFunc, extensionFunc):

   ## Sample input ##
   # initPos = [0.0, 0.0, 0.0] #[x, y, z]
   # goalPos = [9.0, 0.0, 0.0] #[x, y, z]
   # objects = {1:[1, 3.0, 0.0, 0.0], 2:[5, 0.0, 7.0, 0.0]} #{key:[radius, x, y, z]}
   # limits = [[-10.0, -10.0, -10.0], [10.0, 10.0, 10.0]]

   ## Sample output ##
   #path = [[0.0, 0.0, 0.0],[0.0, 0.0, 7.0],[9.0, 0.0, 7.0],[9.0, 0.0, 0.0]] # List of x's, y's and z's

   ### Run RRT ###
   debugPrint = False

   # Convert input arguments to numpy arrays
   initPos = np.array(initPos)
   goalPos = np.array(goalPos)
   limits = np.array(limits)

   # Parameters
   maxNodeCnt = 5000
   maxIterationCnt = 1000
   extendPrct = 10.

   # Variables
   nodeCnt = 0
   treeF = [[-1, initPos]] # Forward growing tree - List of parent node index (-1 means root node) and node location
   treeB = [[-1, goalPos]]
   path = []

   # Loop until goal is found or max number of nodes is reached
   iterationCnt = 0
   while (iterationCnt < maxIterationCnt and nodeCnt < maxNodeCnt):
       iterationCnt = iterationCnt + 1
       if (debugPrint): print("Iteration: " + str(iterationCnt))

       # Grow trees randomly
       randPoint = randomPoint(objects, limits, distFunc, validationFunc) # Get random point
       treeF = growTree(treeF, randPoint, objects, extendPrct, distFunc, validationFunc, extensionFunc)
       if (debugPrint): print(treeF)

       randPoint = randomPoint(objects, limits, distFunc, validationFunc) # Get random point
       treeB = growTree(treeB, randPoint, objects, extendPrct, distFunc, validationFunc, extensionFunc)
       if (debugPrint): print(treeB)

       # Grow trees towards each other
       nodeIndB = random.randint(0, len(treeB)-1)
       pointB = treeB[nodeIndB][1]
       treeF = growTree(treeF, pointB, objects, 100., distFunc, validationFunc, extensionFunc)
       if (debugPrint): print(treeF)
       nodeIndF = len(treeF)-1
       pointF = treeF[nodeIndF][1]

       if (equalPoints(pointF, pointB, distFunc)):
           treeS = generateSolutionTree(treeF, nodeIndF, treeB, nodeIndB)
           break

       nodeIndF = random.randint(0, len(treeF)-1)
       pointF = treeF[nodeIndF][1]
       treeB = growTree(treeB, pointF, objects, 100., distFunc, validationFunc, extensionFunc)
       if (debugPrint): print(treeB)
       nodeIndB = len(treeB)-1
       pointB = treeB[nodeIndB][1]

       if (equalPoints(pointF, pointB, distFunc)):
           treeS = generateSolutionTree(treeF, nodeIndF, treeB, nodeIndB)
           break

       # Count nodes
       nodeCnt = len(treeF) + len(treeB)

       if (debugPrint): print(treeF)
       if (debugPrint): print(treeB)
       if (debugPrint): print("")

   # Generate path
   path = generatePath(treeS,len(treeS)-1)

   # Shorten path
   if (debugPrint):
       for node in path:
           print(node)
   path1 = shortenPath(path, objects, distFunc, extensionFunc)
   if (debugPrint):
       for node in path1:
           print(node)

   # Return path
   return path


## Helper functions##
def equalPoints(point1, point2, distFunc):
    maxDist = .0001
    equalFlag = (distFunc(point1, point2) <= maxDist)
    return equalFlag

def randomPoint(objects, limits, distFunc, validationFunc):
    isValidPoint = False
    while (not(isValidPoint)):
           newPoint = (limits[1] - limits[0]) * np.random.rand(len(limits[0])) + limits[0]
           isValidPoint = validationFunc(newPoint, objects)
    return newPoint

def closestNode(aPoint, aTree, distFunc):
    minDist = np.inf
    minInd = -1
    thisInd = -1
    for aNode in aTree:
        thisInd = thisInd + 1
        thisPoint = aNode[1]
        thisDist = distFunc(thisPoint, aPoint)
        if (thisDist < minDist):
            minDist = thisDist
            minInd = thisInd
    return minInd

def growTree(aTree, aPoint, objects, extendPrct, distFunc, validationFunc, extensionFunc):
    # Find closest node
    nodeInd = closestNode(aPoint, aTree, distFunc)

    # Extend tree forward
    treePoint = aTree[nodeInd][1]
    newTreePoint = extensionFunc(treePoint, aPoint, extendPrct, objects)
    if (not(equalPoints(newTreePoint, treePoint, distFunc))):
        aTree.append([nodeInd, newTreePoint])
    return aTree

def generateSolutionTree(treeF, nodeIndF, treeB, nodeIndB):
    treeS = []
    nodeInd = nodeIndF
    while (nodeInd != -1):
        treeS.insert(0, [np.nan, treeF[nodeInd][1]])
        nodeInd = treeF[nodeInd][0]

    newNodeInd = -1
    for node in treeS:
        node[0] = newNodeInd
        newNodeInd = newNodeInd + 1

    nodeInd = treeB[nodeIndB][0]
    while (nodeInd != -1):
        treeS.append([newNodeInd, treeB[nodeInd][1]])
        nodeInd = treeB[nodeInd][0]
        newNodeInd = newNodeInd + 1

    return treeS

def generatePath(tree, nodeInd):
    path = []
    while (nodeInd != -1):
        path.insert(0, tree[nodeInd][1].tolist())
        nodeInd = tree[nodeInd][0]

    return path

def shortenPath(path, objects, distFunc, extensionFunc):
    n = len(path)
    i = 0
    while (i < n - 2):
        j = n - 1
        while (j > i + 1):
            point = extensionFunc(np.array(path[i]), np.array(path[j]), 100, objects)
            if (equalPoints(point, path[j], distFunc)):
                k = j - 1
                while (k > i):
                    del path[k]
                    k = k - 1
                    n = n - 1
                break
            j = j - 1
        i = i + 1

    return path
