#! /usr/bin/python

import rospy
import actionlib

from msg import EllipsoidMoveAction, EllipsoidMoveGoal

class EllipsoidCtrlBackend(object):
    def __init__(self):
        self.ell_ctrl = actionlib.SimpleActionClient("/ellipsoid_move", EllipsoidMoveAction)

    def local_move(self):
        pass

    def global_move(self):
        pass

    def 
