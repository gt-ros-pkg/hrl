#Particle filter imports
import pfilter as pf
import robot_motion as rm
import object_motion as om
import detection_appearance as da

#Others
import nodes as nd
import transforms2d as t2d
import numpy as np
import math as mt
import opencv.cv as cv
import types
import functools as ft

velocity     = np.matrix([1.0, 0.0]).T
pose         = t2d.Pose2D(2.0, 0.0, 0.0)

#Setup
mvar         = rm.motion_var()
def to_pos(p):
    return p.pos
particles    = map(to_pos, rm.make_set(mvar, pose, 100))

#Run
robot_motion = rm.RobotMotion(mvar)
motion_model = om.ObjectMotion(robot_motion)
app_model    = da.DetectionAppearance(cov=np.matrix([[(.03*.03), 0], [0, (.03*.03)]]))

filter       = pf.PFilter(motion_model, app_model)
display      = nd.RobotDisp("particle filter", size = 2, draw_center=True, meters_radius=10)

max_weight   = app_model.weight(np.matrix([1.0, 0.0]).T, np.matrix([1.0, 0.0]).T)
draw_func    = ft.partial(da.draw_weighted_2D, display, max_weight)
cur_pos      = pose.pos.copy()
cur_set      = particles

while True:
    display.clear()
    filter.step(t2d.Pose2D(velocity[0,0], velocity[1,0], 0), 
                np.matrix([1.0, 0.0]).T, cur_set, draw_func)
    display.draw(wait=10)
























    #cur_pos = cur_pos + velocity
