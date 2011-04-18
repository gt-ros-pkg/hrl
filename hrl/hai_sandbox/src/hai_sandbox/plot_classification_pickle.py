import roslib; roslib.load_manifest('hai_sandbox')
import rospy

import hai_sandbox.recognize_3d as r3d
import hrl_lib.util as ut
import cv
import sys

fname = sys.argv[1]
pkl = ut.load_pickle(fname)
image_name = pkl['image']
img = cv.LoadImageM(image_name)

#Draw the center
r3d.draw_points(img, pkl['center'], [255, 0, 0], 6, 2)

if pkl.has_key('pos'):
    pos_exp = pkl['pos']
    neg_exp = pkl['neg']
    #Draw points tried
    r3d.draw_points(img, pos_exp, [50, 255, 0], 9, 1)
    r3d.draw_points(img, neg_exp, [50, 0, 255], 9, 1)

if pkl.has_key('pos_pred'):
    pos_pred = pkl['pos_pred']
    neg_pred = pkl['neg_pred']
    #Draw prediction 
    r3d.draw_points(img, pos_pred, [255, 204, 51], 3, -1)
    r3d.draw_points(img, neg_pred, [51, 204, 255], 3, -1)


#Draw what we're selecting
tried_point, label = pkl['tried']
if label == r3d.POSITIVE:
    color = [0,255,0]
else:
    color = [0,0,255]
r3d.draw_points(img, tried_point, color, 8, -1)

cv.NamedWindow('task relevant learner display', cv.CV_WINDOW_AUTOSIZE)
cv.ShowImage('task relevant learner display', img)
while True:
    cv.WaitKey(33)
