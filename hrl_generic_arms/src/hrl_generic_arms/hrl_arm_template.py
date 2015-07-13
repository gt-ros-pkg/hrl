#
# Any robot that wants to use EPC should implement the functions
# sketched out in the HRLArm and HRLArmKinematics
#
# Standards:
#   pos is a 3x1 numpy matrix representing a position
#   rot is a 3x3 numpy matrix represetning a rotation
#   All values are computed in the arm's reference frame (frame 0)
#

import numpy as np, math
import copy
from threading import RLock

##
# Abstract class which represents a basic structure for controlling
# an arm.  An equilibrium point (ep) represents a commanded goal for the
# arm which is passed to the underlying controller.  A basic assumption of
# equilibrium point control is that the underlying controller doesn't necessarily
# reach the arm commanded goal due to contact, gravity, low gains in the controller, etc.
# Thus we employ another layer of control which relocates the equilibrium point
# based on the task at hand.
class HRLArm(object):
    def __init__(self, kinematics):
        # object of class derived from HRLArmKinematics
        self.kinematics = kinematics
        self.ep = None # equilibrium point
        self.q = None # angles
        self.qdot = None # angular velocity
        self.lock = RLock()

    #----------------- abstract functions ---------------------
    ##
    # Commands the arm to move to desired equilbrium point
    def set_ep(self, *args):
        raise RuntimeError('Unimplemented Function')

    # publish different viz markers.
    def publish_rviz_markers(self):
        raise RuntimeError('Unimplemented Function')

    ##
    # Resets the equilibrium point based on some estimate
    def reset_ep(self):
        raise RuntimeError('Unimplemented Function')
    #----------------------------------------------------------

    ##
    # Records the current joint angles.  This must be updated regularly
    # by the child class.
    # @param q array-like of joint angles
    def set_joint_angles(self, q):
        with self.lock:
            self.q = copy.copy(q)

    ##
    # Returns the current joint angles of the arm
    # @return array-like of joint angles
    def get_joint_angles(self):
        with self.lock:
            return copy.copy(self.q)

    ##
    # Returns the current equilibrium point
    # @return equilibrium point
    def get_ep(self):
        with self.lock:
            return copy.copy(self.ep)

    ##
    # Returns the current pose of the tooltip
    # @return (pos, rot)
    def get_end_effector_pose(self):
        q = self.get_joint_angles()
        if q is None:
            return None
        else:
            return self.kinematics.FK(q)


##
# Abstract class containing kinematics interfaces for an arm
class HRLArmKinematics(object):
    ##
    # @param n_jts Number of joints of the arm
    # @param tool_pos Postion offset of the tool from the last link in the kinematic chain
    # @param tool_rot Rotation offset of the tool from the last link in the kinematic chain
    def __init__(self, n_jts, arm_base_pos=np.matrix([0.]*3).T, arm_base_rot=np.matrix(np.eye(3)), 
                              tool_pos=np.matrix([0.]*3).T, tool_rot=np.matrix(np.eye(3))):
        self.n_jts = n_jts
        self.set_arm_base(arm_base_pos, arm_base_rot)
        self.set_tooltip(tool_pos, tool_rot)

    #----------------- abstract functions ---------------------
    ##
    # Returns FK for the last link in the kinematic chain.
    # Must be implemented by child class.
    # @param q Joint angles in radians
    # @param link_number Link in the kinematic chain to return FK for
    #                    0 represents the ground frame, 1 returns the first link's frame, ...
    #                    n_jts + 1 returns the tooltip's frame
    # @return pos (3x1) np matrix, rot (3x3) np matrix
    def FK_vanilla(self, q, link_number=None):
        raise RuntimeError('Unimplemented Function')

    ##
    # Computes IK for the last link in the kinematic chain.
    # Must be implemented by child class.
    # @param pos Desired link position (3x1 np matrix)
    # @param rot Desired link rotation (3x3 np matrix)
    # @param q_guess Estimate of the desired joint angles which seeds the IK solver
    def IK_vanilla(self, pos, rot, q_guess=None):
        raise RuntimeError('Unimplemented Function')

    ##
    # Computes Jacobian in the arm's frame given joint angles at the specified position.
    # Must be implemented by child class.
    # @param q Joint angles for Jacobian
    # @param pos Position at which the Jacobian is centered. If None, the tooltip
    #            location from FK is used.
    # @return Jacobian 6xN np matrix
    def jacobian_vanilla(self, q, pos=None):
        raise RuntimeError('Unimplemented Function')
    #----------------------------------------------------------

    ##
    # Returns FK for the last link in the kinematic chain.
    # Must be implemented by child class
    # @param q Joint angles in radians
    # @param link_number Link in the kinematic chain to return FK for
    #                    0 represents the ground frame, 1 returns the first link's frame, ...
    #                    n_jts + 1 (and None) returns the tooltip's frame
    # @return pos (3x1) np matrix, rot (3x3) np matrix
    def FK(self, q, link_number=None):
        if link_number == None or link_number > self.n_jts:
            use_end_affector = True
            link_number = None
        else:
            use_end_affector = False
            
        pos, rot = self.FK_vanilla(q, link_number)

        # offset arm base to FK end
        pos = self.arm_base_pos + self.arm_base_rot * pos
        rot = self.arm_base_rot * rot

        if use_end_affector:
            # offset end location to tooltip
            tooltip_baseframe = rot * self.tooltip_pos
            pos += tooltip_baseframe
            rot = rot * self.tooltip_rot
        return pos, rot

    ##
    # Computes IK for the tooltip.  The desired location is first transformed
    # back into the last link's frame and IK is performed on that location.
    # @param pos Desired link position (3x1 np matrix)
    # @param rot Desired link rotation (3x3 np matrix)
    # @param q_guess Estimate of the desired joint angles which seeds the IK solver
    def IK(self, pos, rot, q_guess=None):
        base_pos = self.arm_base_rot.T * pos - self.arm_base_rot.T * self.arm_base_pos
        base_rot = self.arm_base_rot.T * rot
        last_link_pos = base_pos - base_rot * self.tooltip_rot.T * self.tooltip_pos
        last_link_rot = base_rot * self.tooltip_rot.T
        return self.IK_vanilla(last_link_pos, last_link_rot, q_guess)

    ##
    # Computes Jacobian in the arm's frame given joint angles at the specified position.
    # @param q Joint angles for Jacobian
    # @param pos Position at which the Jacobian is centered. If None, the tooltip
    #            location from FK is used.
    # @return Jacobian 6xN np matrix
    def jacobian(self, q, pos=None):
        if pos is None:
            pos = self.FK(q)
        return self.jacobian_vanilla(q, pos)

    ##
    # Sets the offset of the first link in the kinematic chain
    # @param arm_base_pos Postion offset of the first link in the kinematic chain
    # @param arm_base_rot Rotation offset of the first link in the kinematic chain
    def set_arm_base(self, arm_base_pos, arm_base_rot=np.matrix(np.eye(3))):
        self.arm_base_pos = arm_base_pos
        self.arm_base_rot = arm_base_rot

    ##
    # Sets the offset of the tool from the last link in the kinematic chain
    # @param tool_pos Postion offset of the tool from the last link in the kinematic chain
    # @param tool_rot Rotation offset of the tool from the last link in the kinematic chain
    def set_tooltip(self, tool_pos, tool_rot=np.matrix(np.eye(3))):
        self.tooltip_pos = tool_pos
        self.tooltip_rot = tool_rot

