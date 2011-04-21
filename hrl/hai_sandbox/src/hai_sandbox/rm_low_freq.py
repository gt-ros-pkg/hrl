import roslib; roslib.load_manifest('hai_sandbox')
import cv
import sys

img = cv.LoadImageM(sys.argv[1])
dst = cv.CloneMat(img)
cv.Smooth(img, dst, cv.CV_GAUSSIAN)
cv.SaveImage('filtered.png', dst)
