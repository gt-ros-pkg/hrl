import util as ut
import math as mt
import numpy as np
import transforms2d as t2d

class ObjectMotion:
    def __init__(self, robot_motion):
        self.motion = robot_motion

    def predict(self, odometry, object_state):
        """ state of object in last local frame"""
        new_pose = self.motion.predict(odometry, Pose2D(0.0, 0.0, 0.0))
        b_g_a    = t2d.tranform2d(new_pose.pos, angle)
        return ut.homo_to_point(b_g_a * ut.point_to_homo(object_state))

    def predict_partial(self, odometry):
        rm = self.motion.predict_partial(odometry)
        def predict_partial_f(object_state):
            new_pose = rm(t2d.Pose2D(0.0, 0.0, 0.0))
            b_g_a    = t2d.transform2D(new_pose.pos, new_pose.angle)
            return ut.homo_to_point(b_g_a * ut.point_to_homo(object_state))
        return predict_partial_f

def mean_pt(particles):

