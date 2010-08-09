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

velocity     = np.matrix([0.01, 0.0]).T
pose         = t2d.Pose2D(0.0, 0.0, 0.0)

#Setup
mvar         = rm.motion_var()
particles    = rm.make_set(mvar, pose, 100)

#Run
motion_model = rm.RobotMotion(mvar)
app_model    = da.DetectionAppearance(cov=np.matrix([[(.03*.03), 0], [0, (.03*.03)]]))
filter       = pf.PFilter(motion_model, app_model)
display      = nd.RobotDisp("particle filter", size = 2, draw_center=True, meters_radius=10)

max_weight   = app_model.weight(np.matrix([1.0, 0.0]).T, t2d.Pose2D(1.0, 0.0, 0.0))
draw_func    = ft.partial(rm.draw_weighted_Pose2D, display, max_weight)
cur_pos      = pose.pos.copy()
cur_set      = particles

for i in xrange(100):
    display.clear()
    cur_set = filter.step(t2d.Pose2D(velocity[0,0], velocity[1,0], 0), 
                          cur_pos, cur_set, draw_func)
    scur_pos = display.to_screen(cur_pos)
    display.draw(wait=10)

    cur_pos = cur_pos + velocity
    #print "cur_pos", cur_pos.T
















































#cur_set = filter.step(control_input=t2d.Pose2D(velocity[0,0], velocity[1,0], 0), 
#            measurement=cur_pos, particle_set=cur_set)
#cur_pos = cur_pos + velocity
#cur_set = filter.step(control_input=t2d.Pose2D(velocity[0,0], velocity[1,0], 0), 
#            measurement=cur_pos, particle_set=cur_set)
#cur_pos = cur_pos + velocity


#cur_set      = pf.predict(motion_model, t2d.Pose2D(1.0, 0.0, 0.0), cur_set)
#weighted_set = pf.likelihood(app_model, np.matrix([1.0, 0.0]).T, cur_set)
#normalized   = pf.normalize_likelihood(weighted_set)
#for i in xrange(100):
#    cur_set = filter.step(control_input=t2d.Pose2D(velocity[0,0], velocity[1,0], 0), 
#                measurement=cur_pos, particle_set=cur_set)
#    cur_pos = cur_pos + velocity
#
#    display.clear()
#    draw_particles(cur_set)
#    scur_pos = display.to_screen(cur_pos)
#    cv.cvCircle(display.buffer, cv.cvPoint((int) (scur_pos[0,0]), (int) (scur_pos[1,0])), 
#                4, cv.cvScalar(100,0,0), cv.CV_FILLED)
#    display.draw(wait=10)


#filter.step()
#print "particles"
#for s in particles:
#    print s

