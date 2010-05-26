import roslib
roslib.load_manifest('laser_interface')
import cv
import sys

img = cv.LoadImage(sys.argv[1])
cv.NamedWindow('image', 1)
a = cv.CloneImage(img)
while True:
    cv.ShowImage('image', img)
    cv.WaitKey(33)





