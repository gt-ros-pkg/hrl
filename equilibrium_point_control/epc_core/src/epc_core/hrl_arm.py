
#
# Any robot that wants to use EPC should implement the functions
# sketched out in the HRLArm and HRLArmKinematics
#

import numpy as np, math


class HRLArm():
    def __init__(self, kinematics):
        # object of class derived from HRLArmKinematics
        self.kinematics = kinematics
        self.ep = None # equilibrium point

    #------- abstract functions ---------
    def get_joint_angles():
        raise RuntimeError('Unimplemented Function')

    def set_ep(self, *args):
        raise RuntimeError('Unimplemented Function')

    def get_ep(self):
        raise RuntimeError('Unimplemented Function')

    def viz_ep(self, ep):
        raise RuntimeError('Unimplemented Function')
    
    def freeze(self):
        self.set_ep(self.get_ep())

    def get_end_effector_pose(self):
        return self.kinematics.FK(self.get_joint_angles())


class HRLArmKinematics():
    def __init__(self):
        self.tooltip_pos = np.matrix([0.,0.,0.]).T
        self.tooltip_rot = np.matrix(np.eye(3))

    # @param q - array-like (RADIANs)
    # @param link_number - perform FK up to this link. (1-7)
    # @return pos (3X1) np matrix, rot (3X3) np matrix
    def FK(self, q, link_number=None):
        raise RuntimeError('Unimplemented Function')

    def IK(self, p, rot, q_guess=None):
        raise RuntimeError('Unimplemented Function')

    ## compute Jacobian at point pos.
    def Jacobian(self, q, pos=None):
        raise RuntimeError('Unimplemented Function')

    ## define tooltip as a 3x1 np matrix in the wrist coord frame.
    def set_tooltip(self, arm, p, rot=np.matrix(np.eye(3))):
        self.tooltip_pos = p
        self.tooltip_rot = rot



