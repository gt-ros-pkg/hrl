
from opencv.cv import *
from opencv.highgui import *

import time


def write_separate_frames(n_frames, cvim):
    for i in range(n_frames):
        cvSaveImage('b.png', cvim)
        print 'done %d'%i

def write_video(n_frames, cvim):
    vwr = cvCreateVideoWriter('b.avi', CV_FOURCC('I','4','2','0'),
    #vwr = cvCreateVideoWriter('b.avi', CV_FOURCC_DEFAULT,
                              30, cvGetSize(cvim), True)

    for i in range(n_frames):
        cvWriteFrame(vwr, cvim)
        print 'done %d'%i


if __name__ == '__main__':
    
    print 'Hello World'

    cvim = cvLoadImage('a.png')

    n_frames = 100

    t0 = time.time()
    #write_separate_frames(n_frames, cvim)
    write_video(n_frames, cvim)
    t1 = time.time()

    print 'Writing rate: ', n_frames/(t1-t0)


