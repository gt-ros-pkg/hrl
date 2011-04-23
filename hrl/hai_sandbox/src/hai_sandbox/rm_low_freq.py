import roslib; roslib.load_manifest('hai_sandbox')
import cv
import sys
import os.path as pt

img_path = sys.argv[1]
print 'loading', img_path
img = cv.LoadImageM(img_path)
dst = cv.CloneMat(img)
dif = cv.CloneMat(img)
cv.Smooth(img, dst, cv.CV_GAUSSIAN, 91)
cv.Sub(img, dst, dif)
cv.SaveImage(img_path, dif)

#orig_path, fname = pt.split(img_path)
#name = pt.splitext(fname)[0]
#pt.join(orig_path, name)



