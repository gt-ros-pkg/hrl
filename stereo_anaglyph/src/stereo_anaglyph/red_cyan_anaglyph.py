#!/usr/bin/python
import roslib
roslib.load_manifest('stereo_anaglyph')
import rospy
import hrl_camera.ros_camera as rc
import cv

def add_alpha_channel(bgr, alpha_val):
    w, h = cv.GetSize(bgr)
    bgra  = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 4)
    alpha = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 1)
    chan1 = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 1)
    chan2 = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 1)
    chan3 = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 1)
    [cv.Set(c, 0) for c in [chan1, chan2, chan3, bgra, alpha]]

    cv.Split(bgr, chan1, chan2, chan3, None)
    cv.Set(alpha, (alpha_val))
    cv.Merge(chan1, chan2, chan3, alpha, bgra)
    return bgra


def remove_channels(in_bgra, channel_indices):
    w, h = cv.GetSize(in_bgra)
    chan1 = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
    chan2 = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
    chan3 = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
    chan4 = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 1)
    bgra  = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 4)
    [cv.Set(c, 0) for c in [chan1, chan2, chan3, chan4, bgra]]
    cv.Split(in_bgra, chan1, chan2, chan3, chan4)
    chan_list = [chan1, chan2, chan3, chan4]
    for i in channel_indices:
        chan_list[i] = None
    chan_list.append(bgra)
    cv.Merge(*tuple(chan_list))
    return bgra


def anaglyph(left_color, right_color, correction):
    #create oversized image
    #result = cv.CreateImage(cv.GetSize(right_color), cv.IPL_DEPTH_8U, 4)
    w, h = cv.GetSize(left_color)
    bgra = cv.CreateImage((w*2, h), cv.IPL_DEPTH_8U, 4)
    cv.Set(bgra, 0)
    right_bgra = add_alpha_channel(right_color, round(255/2.)) #cyan (remove red?)
    left_bgra  = add_alpha_channel(left_color, round(255/2.)) #red (remove blue?, green?)

    #remove blue & green from left => red
    left_red = remove_channels(left_bgra, [0, 1])
    #remove red from right_bgra => cyan
    right_cyan = remove_channels(right_bgra, [2])


    if correction < 0:
        left_area = cv.GetSubRect(bgra, (-correction,0,w,h))
        right_area = cv.GetSubRect(bgra, (0, 0, w, h))
        valid_area = cv.GetSubRect(bgra, (-correction, 0, w + correction, h))
    else:
        #copy left & right onto bgra
        left_area = cv.GetSubRect(bgra, (0,0,w,h))
        right_area = cv.GetSubRect(bgra, (correction, 0, w, h))
        valid_area = cv.GetSubRect(bgra, (correction, 0, w - correction, h))

    cv.Add(left_red, left_area, left_area)
    cv.Add(right_cyan, right_area, right_area)

    #return right_cyan
    #return left_red
    #return left_bgra
    #return bgra
    return valid_area

if __name__ == '__main__':
    import optparse
    import time
    from sensor_msgs.msg import Image
    from cv_bridge.cv_bridge import CvBridge, CvBridgeError

    p = optparse.OptionParser()
    p.add_option('-c', action='store', default='/wide_stereo', type='string', dest='cam', help='which camera to listen to')
    p.add_option('-d', action='store', default=30, type='int', dest='dist', help='separation distance')
    p.add_option('-s', action='store_true', dest='headless', help='headless mode')
    opt, args = p.parse_args()

    cameras = [opt.cam + '/left/image_rect_color', 
               opt.cam + '/right/image_rect_color']
    stereo_listener = rc.ROSStereoListener(cameras)
    if not opt.headless:
        #cv.NamedWindow('left', 0)
        #cv.NamedWindow('right', 0)
        cv.NamedWindow('stereo-anaglyph', 0)
        cv.ResizeWindow('stereo-anaglyph', 640, 480)
        cv.WaitKey(10)
    else:
        bridge = CvBridge()
        image_pub = rospy.Publisher('stereo_anaglyph', Image)

    anaglyph_cyan_image_distance_correction = rospy.get_param('anaglyph_dist', opt.dist)
   
    left = 1113937# 65361
    right = 1113939#65363 
    escape = 1048603#27
    while not rospy.is_shutdown():
        l, r = stereo_listener.next()
        red_blue = anaglyph(l, r, anaglyph_cyan_image_distance_correction)
        if not opt.headless:
            #cv.ShowImage('left', l)
            #cv.ShowImage('right', r)
            cv.ShowImage('stereo-anaglyph', red_blue)
            k = cv.WaitKey(10)
            print k
            if k == escape:
                break
            if k == left:
                anaglyph_cyan_image_distance_correction = anaglyph_cyan_image_distance_correction - 1
                print anaglyph_cyan_image_distance_correction
            if k == right:
                anaglyph_cyan_image_distance_correction = anaglyph_cyan_image_distance_correction + 1
                print anaglyph_cyan_image_distance_correction
        else:
            rosimage = bridge.cv_to_imgmsg(red_blue, "bgra8")
            image_pub.publish(rosimage)


































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
