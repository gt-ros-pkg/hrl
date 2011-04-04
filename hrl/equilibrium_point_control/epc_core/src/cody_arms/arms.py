
import math
import numpy as np
import copy
import sys, time, os

import PyKDL as kdl
import create_IK_guess_dict as cgd

import roslib; roslib.load_manifest('epc_core')

import hrl_lib.transforms as tr
import hrl_lib.util as ut

#-------------- TF stuff ---------------
def link_tf_name(arm, link_number):
    if arm == 'right_arm':
        nm = 'r_arm'
    else:
        nm = 'l_arm'

    if link_number == 0:
        nm = 'torso_link'
    elif link_number == 7:
        nm = nm + '_ee'
    else:
        nm = nm + '_' + str(link_number)
    return nm


#-------------------- conversion from KDL ---------------------
def kdl_vec_to_np(kdl_vec):
    ''' kdl rotation matrix -> 3x3 numpy matrix.
    '''
    v = np.matrix([kdl_vec[0],kdl_vec[1],kdl_vec[2]]).T
    return v

def kdl_rot_to_np(kdl_rotation):
    ''' kdl rotation matrix -> 3x3 numpy matrix.
    '''
    m = kdl_rotation
    rot = np.matrix([[m[0,0],m[1,0],m[2,0]],
                     [m[0,1],m[1,1],m[2,1]],
                     [m[0,2],m[1,2],m[2,2]]])
    return rot

## 3x1 np vector -> KDL Vector
def np_vec_to_kdl(p):
    return kdl.Vector(p[0,0],p[1,0],p[2,0])

## 3x3 np rotation matrix -> KDL Rotation.
def np_rot_to_kdl(rot):
    return kdl.Rotation(rot[0,0],rot[1,0],rot[2,0],rot[0,1],rot[1,1],rot[2,1],rot[0,2],rot[1,2],rot[2,2])

class M3HrlRobot():
    def __init__(self, end_effector_length=0.11818):
        # create joint limit dicts
        self.joint_lim_dict = {}
        self.joint_lim_dict['right_arm'] = {'max':[math.radians(a) for a in [ 100.00, 60.,  77.5, 144., 122.,  65.,  65.]],
                                            'min':[math.radians(a) for a in [ -47.61,  0., -77.5,   0., -80., -65., -65.]]}
        self.joint_lim_dict['left_arm'] = {'max':[math.radians(a) for a in [ 100.00,   20.,  77.5, 144.,   80.,  65.,  65.]],
                                           'min':[math.radians(a) for a in [ -47.61, -122., -77.5,   0., -125., -65., -65.]]}

        self.setup_kdl_mekabot(end_effector_length)
        q_guess_pkl_l = os.environ['HOME']+'/svn/gt-ros-pkg/hrl/equilibrium_point_control/epc_core/src/cody_arms/q_guess_left_dict.pkl'
        q_guess_pkl_r = os.environ['HOME']+'/svn/gt-ros-pkg/hrl/equilibrium_point_control/epc_core/src/cody_arms/q_guess_right_dict.pkl'

        self.q_guess_dict_left = ut.load_pickle(q_guess_pkl_l)
        self.q_guess_dict_right = ut.load_pickle(q_guess_pkl_r)

    # KDL joint array to meka joint list. (7->7)
    # arm - 'left_arm' or 'right_arm'  
    def kdl_angles_to_meka(self, arm, q_jnt_arr):
        if q_jnt_arr == None:
            return None

        q_rad = [0. for i in range(7)]
        q_rad[0] = -q_jnt_arr[0]
        q_rad[1] = -q_jnt_arr[1]
        q_rad[2] = -q_jnt_arr[2]
        q_rad[3] = -q_jnt_arr[3]
        q_rad[4] = -q_jnt_arr[4]
        q_rad[5] = -q_jnt_arr[5]
        q_rad[6] = -q_jnt_arr[6]
        return q_rad

    # meka joint list to KDL joint array (7->7)
    # arm - 'left_arm' or 'right_arm'
    def meka_angles_to_kdl(self,arm,q_list):
        if q_list == None:
            return None

        n_joints = len(q_list)
        q = kdl.JntArray(n_joints)
        q[0] = -q_list[0]
        q[1] = -q_list[1]
        q[2] = -q_list[2]
        if n_joints > 3:
            q[3] = -q_list[3]
        if n_joints == 7:
            q[4] = -q_list[4]
            q[5] = -q_list[5]
            q[6] = -q_list[6]
        return q

    ## clamp joint angles to their physical limits.
    # @param arm - 'left_arm' or 'right_arm'
    # @param q - list of 7 joint angles.
    #
    # The joint limits for IK are larger that the physical
    # limits. First created this function to clamp prior to
    # computing the torque due to the virtual springs.
    def clamp_to_physical_joint_limits(self, arm, q):
        if arm == 'right_arm':
            q[0] = ut.bound(q[0],math.radians(197.6) ,math.radians(-47.6))
            q[1] = ut.bound(q[1],math.radians(122.15),math.radians(-20))
            q[2] = ut.bound(q[2],math.radians(77.5)  ,math.radians(-77.5))
            q[3] = ut.bound(q[3],math.radians(144.)  ,math.radians(0.))
            q[4] = ut.bound(q[4],math.radians(125.)  ,math.radians(-80.))
            q[5] = ut.bound(q[5],math.radians(45.)   ,math.radians(-45.))
            q[6] = ut.bound(q[6],math.radians(45.)   ,math.radians(-45.))
        elif arm == 'left_arm':
            q[0] = ut.bound(q[0],math.radians(197.6)  ,math.radians(-47.6))
            q[1] = ut.bound(q[1],math.radians(-122.15),math.radians(20))
            q[2] = ut.bound(q[2],math.radians(77.5)   ,math.radians(-77.5))
            q[3] = ut.bound(q[3],math.radians(144.)   ,math.radians(0.))
            q[4] = ut.bound(q[4],math.radians(-125.)  ,math.radians(80.))
            q[5] = ut.bound(q[5],math.radians(45.)    ,math.radians(-45.))
            q[6] = ut.bound(q[6],math.radians(45.)    ,math.radians(-45.))
        return q

    def within_joint_limits(self,arm,q):
        ''' angles list in radians
            returns True or False
            arm - 'left_arm' or 'right_arm'
        '''
        if q == None:
            return False

        if arm == 'right_arm':
            j0 = q[0]<math.radians(100) and q[0]>math.radians(-60)
            j1 = q[1]<math.radians(100) and q[1]>math.radians(-25)
            j2 = q[2]<math.radians(90)  and q[2]>math.radians(-90)
            j3 = q[3]<math.radians(140) and q[3]>math.radians(-5)
            j4 = q[4]<math.radians(135) and q[4]>math.radians(-90)
        elif arm == 'left_arm':
            j0 = q[0]<math.radians(100) and q[0]>math.radians(-60)
            j1 = q[1]<math.radians(25) and q[1]>math.radians(-100)
            j2 = q[2]<math.radians(90)  and q[2]>math.radians(-90)
            j3 = q[3]<math.radians(140) and q[3]>math.radians(-5)
            j4 = q[4]<math.radians(90) and q[4]>math.radians(-135)

        return (j0 and j1 and j2 and j3 and j4)

    def within_wrist_joint_limits(self,arm,q):
        ''' angles list in radians
            returns True or False
            arm - 'left_arm' or 'right_arm'
            check wrist joints j4, j5, j6 only
        '''
        if q == None:
            return False

        if arm == 'right_arm':
            j4 = q[4]<math.radians(135) and q[4]>math.radians(-90)
            j5 = q[5]<math.radians(40) and q[0]>math.radians(-40)
            j6 = q[6]<math.radians(40) and q[1]>math.radians(-40)

        elif arm == 'left_arm':
            j4 = q[4]<math.radians(90) and q[4]>math.radians(-135)
            j5 = q[5]<math.radians(40) and q[0]>math.radians(-40)
            j6 = q[6]<math.radians(40) and q[1]>math.radians(-40)

        return (j4 and j5 and j6)

    def within_physical_limits(self, arm, q, delta_list=[0.,0.,0.,0.,0.,0.,0.]):
        if arm == 'right_arm':
            max_l = [math.radians(a) for a in [ 100.00, 60.,  77.5, 144., 122.,  45.,  45.]]
            min_l = [math.radians(a) for a in [-47.61, -10., -77.5,   0., -80., -45., -45.]]
        elif arm == 'left_arm':
            max_l = [math.radians(a) for a in [ 100.00, 10.,  77.5, 144., 80.,  45.,  45.]]
            min_l = [math.radians(a) for a in [-47.61, -60., -77.5, 0., -122., -45., -45.]]
        qmax,qmin = [],[]
        for i in range(7):
            qmax.append(max_l[i]+delta_list[i])
            qmin.append(min_l[i]-delta_list[i])

        for i in range(7):
            if q[i]>qmax[i] or q[i]<qmin[i]:
                return False
        return True

    def setup_kdl_mekabot(self, end_effector_length=0.12818):
        """ Create KDL chain, fk and ik solvers for the left and right arms.
            
        """
        #RIGHT ARM SETUP
        kdl_rightarm = {}
        chain_upperarm_right = kdl.Chain()
        chain_upperarm_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,-0.18493,0.))))
        chain_upperarm_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,-0.03175,0.))))
        chain_upperarm_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.00635,0.,-0.27795))))

        chain_lowerarm_right = kdl.Chain()
        chain_lowerarm_right.addChain(chain_upperarm_right)
        chain_lowerarm_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,-0.27853))))

        chain_right = kdl.Chain()
        chain_right.addChain(chain_lowerarm_right)
        chain_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
#        chain_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-0.12818))))
        chain_right.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-end_effector_length))))

	  # chain_right2: 7-DOF chain with no end-effector offset
        chain_right2 = kdl.Chain()
        chain_right2.addChain(chain_lowerarm_right)
        chain_right2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_right2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_right2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,0.))))
        
        kdl_rightarm['chain_upperarm'] = chain_upperarm_right
        fk_upperarm_right = kdl.ChainFkSolverPos_recursive(chain_upperarm_right)
        kdl_rightarm['fk_upperarm'] = fk_upperarm_right

        kdl_rightarm['chain_lowerarm'] = chain_lowerarm_right
        kdl_rightarm['nJnts_lowerarm'] = chain_lowerarm_right.getNrOfJoints()
        fk_lowerarm_right = kdl.ChainFkSolverPos_recursive(chain_lowerarm_right)
        kdl_rightarm['fk_lowerarm'] = fk_lowerarm_right
        ik_lowerarm_v_right = kdl.ChainIkSolverVel_pinv(chain_lowerarm_right)
        kdl_rightarm['ik_lowerarm_v'] = ik_lowerarm_v_right

        ik_lowerarm_p_right = kdl.ChainIkSolverPos_NR(chain_lowerarm_right,fk_lowerarm_right,ik_lowerarm_v_right)
        kdl_rightarm['ik_lowerarm_p'] = ik_lowerarm_p_right

        kdl_rightarm['chain'] = chain_right
        kdl_rightarm['nJnts'] = chain_right.getNrOfJoints()

        fk_p_right = kdl.ChainFkSolverPos_recursive(chain_right)
        kdl_rightarm['fk_p'] = fk_p_right

        ik_v_right = kdl.ChainIkSolverVel_pinv(chain_right)
        kdl_rightarm['ik_v'] = ik_v_right

        #-------- extremely convoluted: KDl angles are -ve of meka angles, so max meka->min KDL. (Advait, July 6, 2009)
        #-------- a cleaner way of doing this is required.
        min_kdl_lim = self.meka_angles_to_kdl('right_arm',self.joint_lim_dict['right_arm']['max']) 
        max_kdl_lim = self.meka_angles_to_kdl('right_arm',self.joint_lim_dict['right_arm']['min']) 
        #ik_p_right = kdl.ChainIkSolverPos_NR_JL(chain_right,min_kdl_lim,max_kdl_lim,fk_p_right,ik_v_right)
        ik_p_right_nolim = kdl.ChainIkSolverPos_NR(chain_right,fk_p_right,ik_v_right)
        #kdl_rightarm['ik_p'] = ik_p_right
        kdl_rightarm['ik_p'] = ik_p_right_nolim
        kdl_rightarm['ik_p_nolim'] = ik_p_right_nolim

	#    7-DOF chain with no end-effector offset
        kdl_rightarm['chain2'] = chain_right2
        kdl_rightarm['nJnts2'] = chain_right2.getNrOfJoints()
        fk_p_right2 = kdl.ChainFkSolverPos_recursive(chain_right2)
        kdl_rightarm['fk_p2'] = fk_p_right2
        ik_v_right2 = kdl.ChainIkSolverVel_pinv(chain_right2)
        kdl_rightarm['ik_v2'] = ik_v_right2
        ik_p_right2 = kdl.ChainIkSolverPos_NR(chain_right2,fk_p_right2,ik_v_right2)
        kdl_rightarm['ik_p2'] = ik_p_right2

        kdl_rightarm['jacobian_solver'] = kdl.ChainJntToJacSolver(chain_right)

        #LEFT ARM SETUP
        kdl_leftarm = {}
        chain_upperarm_left = kdl.Chain()
        chain_upperarm_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.18493,0.))))
        chain_upperarm_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.03175,0.))))
        chain_upperarm_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.00635,0.,-0.27795))))

        chain_lowerarm_left = kdl.Chain()
        chain_lowerarm_left.addChain(chain_upperarm_left)
        chain_lowerarm_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,-0.27853))))

        chain_left = kdl.Chain()
        chain_left.addChain(chain_lowerarm_left)
        chain_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
#        chain_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-0.12818))))
        chain_left.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,-end_effector_length))))

        
	#    chain_left2: 7-DOF chain with no end-effector offset
        chain_left2 = kdl.Chain()
        chain_left2.addChain(chain_lowerarm_left)
        chain_left2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_left2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        chain_left2.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,0.))))
        

        kdl_leftarm['chain_upperarm'] = chain_upperarm_left
        kdl_leftarm['chain_lowerarm'] = chain_lowerarm_left
        fk_lowerarm_left = kdl.ChainFkSolverPos_recursive(chain_lowerarm_left)
        kdl_leftarm['fk_lowerarm'] = fk_lowerarm_left

        ik_lowerarm_v_left = kdl.ChainIkSolverVel_pinv(chain_lowerarm_left)
        kdl_leftarm['ik_lowerarm_v'] = ik_lowerarm_v_left
        ik_lowerarm_p_left = kdl.ChainIkSolverPos_NR(chain_lowerarm_left,fk_lowerarm_left,ik_lowerarm_v_left)
        kdl_leftarm['ik_lowerarm_p'] = ik_lowerarm_p_left

        kdl_leftarm['chain'] = chain_left
        kdl_leftarm['nJnts'] = chain_left.getNrOfJoints()

        fk_p_left = kdl.ChainFkSolverPos_recursive(chain_left)
        kdl_leftarm['fk_p'] = fk_p_left

        ik_v_left = kdl.ChainIkSolverVel_pinv(chain_left)
        kdl_leftarm['ik_v'] = ik_v_left
        ik_p_left = kdl.ChainIkSolverPos_NR(chain_left,fk_p_left,ik_v_left)
        kdl_leftarm['ik_p'] = ik_p_left

	#    chain_left2: 7-DOF chain with no end-effector offset
        kdl_leftarm['chain2'] = chain_left2
        kdl_leftarm['nJnts2'] = chain_left2.getNrOfJoints()
        fk_p_left2 = kdl.ChainFkSolverPos_recursive(chain_left2)
        kdl_leftarm['fk_p2'] = fk_p_left2
        ik_v_left2 = kdl.ChainIkSolverVel_pinv(chain_left2)
        kdl_leftarm['ik_v2'] = ik_v_left2
        ik_p_left2 = kdl.ChainIkSolverPos_NR(chain_left2,fk_p_left2,ik_v_left2)
        kdl_leftarm['ik_p2'] = ik_p_left2


        kdl_leftarm['jacobian_solver'] = kdl.ChainJntToJacSolver(chain_left)

        #Add both chains to dictionary
        self.mekabot_kdl = {'right_arm':kdl_rightarm,'left_arm':kdl_leftarm}

    def FK_kdl(self, arm, q, link_number):
        fk_solver = self.mekabot_kdl[arm]['fk_p']
        endeffec_frame = kdl.Frame()
        kinematics_status = fk_solver.JntToCart(q, endeffec_frame,
                                                link_number)
        if kinematics_status >= 0:
#            print 'End effector transformation matrix:', endeffec_frame
            return endeffec_frame
        else:
            print 'Could not compute forward kinematics.'
            return None

    def FK_rot(self, arm, q, link_number = 7):
        pos, rot = self.FK_all(arm, q, link_number)
        return rot

    # @param arm - 'left_arm' or 'right_arm'
    # @param q - list of 7 joint angles (RADIANs)
    # @param link_number - perform FK up to this link. (1-7)
    # @return 3x1 numpy matrix
    def FK(self, arm, q, link_number = 7):
        pos, rot = self.FK_all(arm, q, link_number)
        return pos

    def FK_all(self, arm, q, link_number = 7):
        q = self.meka_angles_to_kdl(arm, q)
        frame = self.FK_kdl(arm, q, link_number)
        pos = frame.p
        pos = kdl_vec_to_np(pos)
        m = frame.M
        rot = kdl_rot_to_np(m)
        return pos, rot

    def Jac_kdl(self,arm,q):
        ''' returns the Jacobian, given the joint angles
        '''
        J_kdl = kdl.Jacobian(7)
        self.mekabot_kdl[arm]['jacobian_solver'].JntToJac(q,J_kdl)

        kdl_jac =  np.matrix([
            [J_kdl[0,0],J_kdl[0,1],J_kdl[0,2],J_kdl[0,3],J_kdl[0,4],J_kdl[0,5],J_kdl[0,6]],
            [J_kdl[1,0],J_kdl[1,1],J_kdl[1,2],J_kdl[1,3],J_kdl[1,4],J_kdl[1,5],J_kdl[1,6]],
            [J_kdl[2,0],J_kdl[2,1],J_kdl[2,2],J_kdl[2,3],J_kdl[2,4],J_kdl[2,5],J_kdl[2,6]],
            [J_kdl[3,0],J_kdl[3,1],J_kdl[3,2],J_kdl[3,3],J_kdl[3,4],J_kdl[3,5],J_kdl[3,6]],
            [J_kdl[4,0],J_kdl[4,1],J_kdl[4,2],J_kdl[4,3],J_kdl[4,4],J_kdl[4,5],J_kdl[4,6]],
            [J_kdl[5,0],J_kdl[5,1],J_kdl[5,2],J_kdl[5,3],J_kdl[5,4],J_kdl[5,5],J_kdl[5,6]],
            ])
        return kdl_jac
        
    def Jac(self,arm,q):
        ''' q - list of 7 joint angles (meka axes) in RADIANS.
            arm - 'right_arm' or 'left_arm'
            returns 6x7 numpy matrix.
        '''
        jntarr = self.meka_angles_to_kdl(arm,q)
        kdl_jac = self.Jac_kdl(arm,jntarr)
        meka_jac = -kdl_jac  # the kdl jacobian is the negative of meka jacobian (see kdl_angles_to_meka)
        return meka_jac

    def IK_kdl(self,arm,frame, q_init):
        ''' IK, returns jointArray (None if impossible)
            frame - desired frame of the end effector
            q_init - initial guess for the joint angles. (JntArray)
        '''
        nJnts = self.mekabot_kdl[arm]['nJnts']
        ik_solver = self.mekabot_kdl[arm]['ik_p']
        q = kdl.JntArray(nJnts)
        if ik_solver.CartToJnt(q_init,frame,q)>=0:
            for i in range(nJnts):
                q[i] = tr.angle_within_mod180(q[i])
            return q
        else:
            if arm == 'right_arm':
                ik_solver = self.mekabot_kdl[arm]['ik_p_nolim']
                if ik_solver.CartToJnt(q_init,frame,q)>=0:
                    for i in range(nJnts):
                        q[i] = tr.angle_within_mod180(q[i])
                    return q
            print 'Error: could not calculate inverse kinematics'
            return None

    ##
    # Inverse Kinematics using KDL.
    # @param p - 3X1 numpy matrix.
    # @param rot - 3X3 numpy matrix. It transforms a  vector in the
    # torso frame to the end effector frame.
    # @return list of 7 joint angles, or None if IK soln not found.
    def IK(self,arm,p,rot,q_guess=None):
        p_kdl = np_vec_to_kdl(p)
        rot_kdl = np_rot_to_kdl(rot)
        fr = kdl.Frame(rot_kdl,p_kdl)

        if q_guess == None:
            if arm == 'left_arm':
                q_guess = cgd.find_good_config(p,self.q_guess_dict_left)
            elif arm == 'right_arm':    
                q_guess = cgd.find_good_config(p,self.q_guess_dict_right)

        q_guess = self.meka_angles_to_kdl(arm,q_guess)

        q_res = self.IK_kdl(arm,fr,q_guess)
        q_res = self.kdl_angles_to_meka(arm,q_res)

        if self.within_joint_limits(arm,q_res):
            if arm == 'right_arm':  
                if q_res[1]<0.:
                    q_res[1] = math.radians(10.)
                    qg = self.meka_angles_to_kdl(arm,q_res)
                    q_res = self.IK_kdl(arm,fr,qg)
                    q_res = self.kdl_angles_to_meka(arm,q_res)
                if self.within_joint_limits(arm,q_res):
                    return q_res
                else:
                    return None
            else:
                return q_res
        else:
            return None








