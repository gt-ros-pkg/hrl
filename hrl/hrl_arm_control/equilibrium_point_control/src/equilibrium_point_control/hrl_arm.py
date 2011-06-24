
#
# Any robot that wants to use EPC should implement the functions
# sketched out in the HRLArm and HRLArmKinematics
#

import numpy as np, math
import copy
from threading import RLock

class HRLArm():
    def __init__(self, kinematics):
        # object of class derived from HRLArmKinematics
        self.kinematics = kinematics
        self.ep = None # equilibrium point
        self.q = None # angles
        self.qdot = None # angular velocity
        self.lock = RLock()

    #------- abstract functions ---------
    def get_joint_angles(self):
        with self.lock:
            return copy.copy(self.q)

    def set_ep(self, *args):
        raise RuntimeError('Unimplemented Function')

    # publish different viz markers.
    def publish_rviz_markers(self):
        raise RuntimeError('Unimplemented Function')

    def get_ep(self):
        with self.lock:
            return copy.copy(self.ep)
    
    # do we really need this function?
    def freeze(self):
        self.set_ep(self.ep)

    def get_end_effector_pose(self):
        return self.kinematics.FK(self.get_joint_angles())


class HRLArmKinematics():
    def __init__(self, n_jts):
        self.tooltip_pos = np.matrix([0.,0.,0.]).T
        self.tooltip_rot = np.matrix(np.eye(3))
        self.n_jts = n_jts

    # FK without the tooltip
    def FK_vanilla(self, q, link_number=None):
        raise RuntimeError('Unimplemented Function')

    # @param q - array-like (RADIANs)
    # @param link_number - perform FK up to this link. (0-n_jts)
    # @return pos (3X1) np matrix, rot (3X3) np matrix
    def FK(self, q, link_number=None):
        if link_number == None:
            link_number = self.n_jts
        link_number = min(link_number, self.n_jts)
        pos, rot = self.FK_vanilla(q, link_number)

        if link_number == self.n_jts:
            tooltip_baseframe = rot * self.tooltip_pos
            pos += tooltip_baseframe
            rot = rot * self.tooltip_rot
        return pos, rot

    # IK without the  tooltip.
    def IK_vanilla(self, p, rot, q_guess=None):
        raise RuntimeError('Unimplemented Function')

    # @param p - 3x1 np matrix
    # @param rot - orientation of end effector frame wrt base of the arm.
    def IK(self, p, rot, q_guess=None):
        # this code should be common to everyone.
        pass

    ## compute Jacobian at point pos.
    def Jacobian(self, q, pos=None):
        raise RuntimeError('Unimplemented Function')

    ## define tooltip as a 3x1 np matrix in the wrist coord frame.
    def set_tooltip(self, p, rot=np.matrix(np.eye(3))):
        self.tooltip_pos = p
        self.tooltip_rot = rot



