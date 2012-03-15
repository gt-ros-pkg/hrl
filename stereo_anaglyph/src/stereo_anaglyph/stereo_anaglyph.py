#!/usr/bin/python
import roslib
roslib.load_manifest('stereo_anaglyph')
import rospy
import hrl_camera.ros_camera as rc
import cv


def anaglyph(left_color, right_color):
    left_mono = cv.CreateImage(cv.GetSize(left_color), cv.IPL_DEPTH_8U, 1)
    right_mono = cv.CreateImage(cv.GetSize(right_color), cv.IPL_DEPTH_8U, 1)
    green = cv.CreateImage(cv.GetSize(right_color), cv.IPL_DEPTH_8U, 1)
    result = cv.CreateImage(cv.GetSize(right_color), cv.IPL_DEPTH_8U, 3)

    cv.CvtColor(left_color, left_mono, cv.CV_RGB2GRAY)
    cv.CvtColor(right_color, right_mono, cv.CV_RGB2GRAY)
    cv.Merge(left_mono, green, right_mono, None, result)
    return result

cameras = ['/wide_stereo/left/image_rect_color', 
           '/wide_stereo/right/image_rect_color']
stereo_listener = rc.ROSStereoListener(cameras)
cv.NamedWindow('stereo-anaglyph', cv.CV_WINDOW_AUTOSIZE)
cv.WaitKey(10)

while not rospy.is_shutdown():
    l, r = stereo_listener.next()
    red_blue = anaglyph(l, r)
    cv.ShowImage('stereo-anaglyph', red_blue)
    cv.WaitKey(10)

































#from opencv import cv
#from opencv import highgui
#from time import sleep
#
#def makeMagic(left, right, out):
#    chans=[]
#    for i in range(6):
#        chans.append(cv.cvCreateImage(cv.cvGetSize(left),8,1))
#    cv.cvSplit(left, chans[0], chans[1], chans[2], None);
#    cv.cvSplit(right, chans[3], chans[4], chans[5], None);
#    cv.cvMerge(chans[3],chans[4],chans[2], None, out);
#    
#    #cv.cvMerge(None,chans[1],None, None, out);
#
#cam=[]
#def main():
#    cam.append(highgui.cvCreateCameraCapture(0))
#    cam.append(highgui.cvCreateCameraCapture(1))
#    highgui.cvNamedWindow ("carrots", highgui.CV_WINDOW_AUTOSIZE)
#
#    uno=highgui.cvQueryFrame(cam[0]);
#    dos=highgui.cvQueryFrame(cam[1]);
#
#    highgui.cvShowImage("carrots",uno);
#    highgui.cvWaitKey(0);
#    highgui.cvShowImage("carrots",dos);
#    highgui.cvWaitKey(0);
#
#    merge=cv.cvCreateImage(cv.cvGetSize(uno),8,3)
#    makeMagic(uno, dos, merge)
#
#    highgui.cvShowImage("carrots",merge);
#    highgui.cvWaitKey(0);
#
#    while True :
#        uno=highgui.cvQueryFrame(cam[0]);
#        dos=highgui.cvQueryFrame(cam[1]);
#        makeMagic(uno, dos, merge);
#        highgui.cvShowImage("carrots",merge);
#        if highgui.cvWaitKey(1)=="s":
#          cam.append(cam.pop(0))
#        print "tick"
#
#if __name__=="__main__":
#  main()
