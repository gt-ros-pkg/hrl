import sys, os
import numpy as np
import math
import roslib; roslib.load_manifest('hrl_tactile_controller')
import hrl_lib.util as ut
import scipy.stats as ss

class GettingReachability():
    def __init__(self, d_robot):
        self.root_path = None

        self.aGoalMap  = []
        self.aStartMap = []

        self.fResolution = 0.1
        self.aX = np.zeros((1,1))
        self.aY = np.zeros((1,1))

        self.TOL = 0.00001

        if d_robot == '--planar_three_link_capsule':
            # limit
            self.lStartXlim = [0.13149376, 0.13149376]
            self.lStartYlim = [-0.4, 0.4]
            self.lStartZlim = [0.,0.]

            self.lGoalXlim = [0.3, 0.7]
            self.lGoalYlim = [-0.5, 0.5]
            self.lGoalZlim = [0.,0.]

            # Clutter environment info
            self.lClutterXlim = [0.2,0.85]
            self.lClutterYlim = [-1.2,1.2]

            # Robot info
            self.lRobotXlimInit = [-2.0, 0.2]
            self.fDia = 0.03

            # Resolution
            self.fPosResol = 0.1
            self.fAngResol = 0.3839724354387525 # 22 degree


        elif d_robot == '--three_link_planar_cody':
            # limit
            self.lStartXlim = [0.28, 0.28]
            self.lStartYlim = [-0.5, 0.0]
            self.lStartZlim = [0.,0.]

            self.lGoalXlim = [0.35, 0.7]
            self.lGoalYlim = [-0.5, 0.4]
            self.lGoalZlim = [0.,0.]

            # Clutter environment info
            self.lClutterXlim = [0.3,0.75]
            self.lClutterYlim = [-0.8,0.8]

            # Robot info
            self.lRobotXlimInit = [-2.0, 0.3]
            self.fDia = 0.05

            # Resolution
            self.fPosResol = 0.1
            self.fAngResol = 3.14/180.0*20.0 # degree

        elif d_robot == '--three_link_planar_pr2':
            # limit
            self.lStartXlim = [0.35, 0.35]
            self.lStartYlim = [-0.8, -0.0]
            self.lStartZlim = [0.,0.]

            self.lGoalXlim = [0.5, 1.0]
            self.lGoalYlim = [-0.7, 0.2]
            self.lGoalZlim = [0.,0.]

            # Clutter environment info
            self.lClutterXlim = [0.4,1.2]
            self.lClutterYlim = [-1.0,1.0]

            # Robot info
            self.lRobotXlimInit = [-1.0, 0.4]
            self.fDia = 0.05

            # Resolution
            self.fPosResol = 0.1
            self.fAngResol = 3.14/180.0*20.0 # degree

        pass


    def readData(self, root_path):

        print "*******************************************"
        print "Call Nonparametric Density Estimation Data"
        # Load Previous pkl     
        root_path = root_path.rstrip('/')
        pkl_file = root_path+'/ra_multi_map_multi_start.pkl'

        if os.path.isfile(pkl_file):
            data = ut.load_pickle(pkl_file)
            self.aGoalMap  = data['goalMap'] 
            self.aStartMap = data['startMap']

            self.fResolution = data['resolution']
            self.aX          = data['xRange']
            self.aY          = data['yRange']
            return
        else:
            print "Error: no save file!"
            sys.exit()

    def worldToGrid_2D(self, lWorld, lGridSize, lXlim, lYlim):

        nIndX = int(float(lGridSize[0])*(lWorld[0]-lXlim[0])/(lXlim[1]-lXlim[0]))
        nIndY = int(float(lGridSize[1])*(lWorld[1]-lYlim[0])/(lYlim[1]-lYlim[0]))

        return nIndX, nIndY

    def get_prob_start_goal(self, d_robot, lGoal):
        
        print "Get Probability given start-goal postioin pair"       
        # Get goal index
        aX       = self.aX
        aY       = self.aY
        aGoalMap = self.aGoalMap 

        nXMax = len(aX)
        nYMax = len(aY)

        lGridSize = [nXMax, nYMax]
        lXlim     = [aX[0], aX[-1]]
        lYlim     = [aY[0], aY[-1]]
        nIndX, nIndY = self.worldToGrid_2D(lGoal, lGridSize, lXlim, lYlim)

        # Check limit
        if nIndX > nXMax-1:
            nIndX = nXMax-1
        elif nIndX < 0:
            nIndX = 0

        if nIndY > nYMax-1:
            nIndY = nYMax-1
        elif nIndY < 0:
            nIndY = 0

        # Before Interpolation
        aX  = np.arange(self.lStartYlim[0], self.lStartYlim[1]+self.TOL, self.fPosResol)
        aY  = np.arange(-np.pi, np.pi+self.TOL, self.fAngResol)
        X,Y = np.meshgrid(aX,aY)      
        Z   = (self.aStartMap[nIndX][nIndY])

        if True:
            # New x,y
            aNewX = np.arange(self.lStartYlim[0], self.lStartYlim[1]+self.TOL, self.fPosResol/5.0)
            aNewY = np.arange(-np.pi, np.pi+self.TOL, self.fAngResol/5.0)
            newMeshX,newMeshY = np.meshgrid(aNewX, aNewY)      

            import scipy.interpolate as si
            
            rbfi = si.Rbf(X.T,Y.T,Z,smooth=0.00001)
            aNewStartMap = rbfi(newMeshX.T,newMeshY.T)
        else:
            newMeshX = X
            newMeshY = Y
            aNewStartMap = Z

        fProbMax = 0.0
        
        nYMax, nPhiMax = aNewStartMap.shape
        for nY in range(nYMax):
            for nPhi in range(nPhiMax):
                fProb = aNewStartMap[nY][nPhi]
                if fProb > fProbMax:

                    fProbMax    = fProb
                    nProbMaxY   = nY
                    nProbMaxPhi = nPhi
        if fProbMax == 0.0:
            print "Error: not available start!", lGoal
            sys.exit()
                
        fProbMaxY = aNewX[nProbMaxY]
        fProbMaxPhi = aNewY[nProbMaxPhi]


        #print "aNewStartMap: ", aNewStartMap.shape
        print "Max Start: ", fProbMax, fProbMaxY, fProbMaxPhi
        print "*********************************"

        return fProbMaxY, fProbMaxPhi
        
# @Input : goal list [x,y,z]
def  getInitPos(goal):

    d_robot = '--three_link_planar_pr2'

    gr = GettingReachability(d_robot)

    root_path = os.getcwd()
    gr.readData(root_path)

    # Left arm to Right arm conversion
    mGoal = np.matrix(goal).T
    mGoal[0,0] -= 0.015
    mGoal[1,0] -= 0.177    
    mGoal[1,0] *= -1.0

    print "Right Arm Frame Goal: ", mGoal.T
    Y, Phi = gr.get_prob_start_goal(d_robot, mGoal)
    print "Right Arm Frame Start: ",Y
    print "Right Arm Torso Frame Start: ", Y-0.177

    # Right arm to Left arm conversion
    Y = (-Y+0.177)
    Phi *= -1
    print "Left Arm Torso Frame Start: ", Y

    # Phi to torso frame orientation
    position    = [0.35+0.15, Y, goal[2]]
    orientation = [0.,0.,
                   math.atan2(goal[1]-position[1],goal[0]-position[0])+np.pi/2.0+Phi] 

    return position, orientation


