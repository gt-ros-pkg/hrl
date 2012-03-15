from opencv.cv import *
from opencv.highgui import *


cap = cvCreateFileCapture('b.avi')
cvim = cvQueryFrame(cap)
cvFlip(cvim, cvim)
cvSaveImage('c.png', cvim)


