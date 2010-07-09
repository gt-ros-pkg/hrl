#!/usr/bin/python

import sys, os
sys.path.append(os.environ['HRLBASEPATH']+'/src/libraries/')
import cameras.dragonfly as dr
import roslib; roslib.load_manifest('modeling_forces')
import rospy
import cv

from collections import deque
import time
import math, numpy as np
import glob

import hrl_lib.util as ut
import hrl_camera.ros_camera as rc

def got_pose_cb(data, got_pose_dict):
    if len(data.objects) != 2:
        got_pose_dict['pose_fail'] = True
    else:
        got_pose_dict['pose_fail'] = False

    got_pose_dict['flag'] = True


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-l', '--log', action='store_true', dest='log', help='log FT data')
    p.add_option('-p', '--pub', action='store_true', dest='pub',
                 help='publish over ROS')
    p.add_option('-c', '--conv', action='store_true', dest='avi_to_pngs',
                 help='convert avi to pngs')
    p.add_option('-b', '--bad', action='store_true', dest='bad',
                 help='find the images on which checker tracking failed.')
    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='directory with images')
    p.add_option('-i', '--images_only', action='store_true',
                 dest='images_only', help='work with images (no pkl)')
    p.add_option('-s', '--single_im', action='store', default='',
                 type='string', dest='single_fname', help='work with one image')
    
    opt, args = p.parse_args()

    camera_name = 'remote_head'

    if opt.pub:
        import cv
        from cv_bridge.cv_bridge import CvBridge, CvBridgeError
        from std_msgs.msg import String
        from std_msgs.msg import Empty
        from sensor_msgs.msg import Image
        from sensor_msgs.msg import CameraInfo
        from checkerboard_detector.msg import ObjectDetection

        rospy.init_node('publish_log_images', anonymous=True)

        if opt.single_fname != '':
            im_name_list = [opt.single_fname for i in range(10)]
            time_list = [time.time() for i in range(10)]
        elif opt.images_only:
            im_name_list = glob.glob(opt.dir+'/*.png')
            #im_name_list = glob.glob(opt.dir+'/*.jpg')
            im_name_list.sort()
            time_list = [1 for i in range(len(im_name_list))]
        else:
            l = glob.glob(opt.dir+'/handheld_pull_log*.pkl')
            if l == []:
                raise RuntimeError('%s does not have a handheld_pull_log'%opt.dir)

            pkl_name = l[0]
            d = ut.load_pickle(pkl_name)
            im_name_list = glob.glob(opt.dir+'/0*.png')
            im_name_list.sort()
            time_list = d['time_list']


        import camera_config as cc
        cp = cc.camera_parameters[camera_name]
        m = np.array([ [ cp['focal_length_x_in_pixels'], 0.,
                         cp['optical_center_x_in_pixels'], 0. ],
                       [ 0., cp['focal_length_y_in_pixels'],
                         cp['optical_center_y_in_pixels'], 0. ],
                       [ 0., 0., 1., 0.] ])

        intrinsic_list = [m[0,0], m[0,1], m[0,2], 0.0,
                          m[1,0], m[1,1], m[1,2], 0.0,
                          m[2,0], m[2,1], m[2,2], 0.0]

        topic_name = 'cvcamera_' + camera_name
        image_pub = rospy.Publisher(topic_name, Image)
        config_pub = rospy.Publisher(topic_name+'_info', CameraInfo)
        ch_pub = rospy.Publisher('/checker_to_poses/trigger', Empty)

        time.sleep(0.5)
        bridge = CvBridge()

        got_pose_dict = {'flag': False, 'pose_fail': False}
        topic_name_cb = '/checkerdetector/ObjectDetection'
        rospy.Subscriber(topic_name_cb, ObjectDetection, got_pose_cb,
                         got_pose_dict)

        failed_im_list = [] # list of filenames on which checkboard detection failed.
        n_images = len(im_name_list)
        for i in range(n_images):
            name = im_name_list[i]
            cv_im = cv.LoadImage(name)

            rosimage = bridge.cv_to_imgmsg(cv_im, "bgr8")
            rosimage.header.stamp = rospy.Time.from_sec(time_list[i])

            image_pub.publish(rosimage)
            config_pub.publish(CameraInfo(P=intrinsic_list))

            t_st = time.time()
            while got_pose_dict['flag'] == False:
                time.sleep(0.5)
                if (time.time()-t_st) > 10.:
                    break

            if got_pose_dict['pose_fail'] == True:
                failed_im_list.append(name)

            time.sleep(0.5)
            got_pose_dict['flag'] = False
            got_pose_dict['pose_fail'] = False

            if rospy.is_shutdown():
                break

        print 'Number of images:', n_images
        ch_pub.publish() # send trigger to the ft logger.
        ut.save_pickle(failed_im_list, 'checker_fail_list.pkl')

    if opt.log:
        from opencv.cv import *
        from opencv.highgui import *

        from std_msgs.msg import Empty
        rospy.init_node('image logger', anonymous=True)
        ft_pub = rospy.Publisher('/ftlogger/trigger', Empty)


        cam = dr.dragonfly2(camera_name)
        cam.set_frame_rate(30)
        cam.set_brightness(0, 651, 0, 65)
        for i in range(10):
            im = cam.get_frame_debayered() # undistorting slows down frame rate

        im_list = deque()
        time_list = []

        cvNamedWindow('Image Logging', CV_WINDOW_AUTOSIZE)

        print 'Started the loop.'
        print 'Hit a to start logging, ESC to exit and save pkl'
        log_images = False

        while not rospy.is_shutdown():
            kp = cvWaitKey(1)
            if (type(kp) == str and kp == '\x1b') or (type(kp) != str and kp & 255 == 27): # ESC then exit.
                t1 = time.time()
                ft_pub.publish() # send trigger to the ft logger.
                break
            if (type(kp) == str and kp == 'a') or (type(kp) != str and kp & 255 == 97): # a to start logging.
                log_images = True
                t0 = time.time()
                ft_pub.publish() # send trigger to the ft logger.
                print 'started logging'

            im = cam.get_frame_debayered() # undistorting slows down frame rate
            if log_images:
                time_list.append(time.time())
                im_list.append(cvCloneImage(im))
        
        print 'frame rate:', len(time_list)/(t1-t0)
        
        print 'before saving the pkl'
        d = {}

        t_string = ut.formatted_time()
        video_name = 'mechanism_video_'+t_string+'.avi'
        vwr = cvCreateVideoWriter(video_name, CV_FOURCC('I','4','2','0'),
                                  30, cvGetSize(im_list[0]), True)

        t0 = time.time()
        im_name_list = []
        time_stamp = ut.formatted_time()
        for im in im_list:
            cvWriteFrame(vwr, im)
            time.sleep(.01) #Important to keep force torque server
                            #from restarting
        t1 = time.time()
        print 'disk writing rate:', len(time_list)/(t1-t0)

        d['time_list'] = time_list
        d['video_name'] = video_name

        fname = 'handheld_pull_log_' + t_string + '.pkl'
        ut.save_pickle(d, fname)

        print 'Done saving the pkl'

    if opt.avi_to_pngs:
        from opencv.cv import *
        from opencv.highgui import *
        import util

        import camera_config as cc
        cp = cc.camera_parameters[camera_name]

        size = (int(cp['calibration_image_width']), int(cp['calibration_image_height']))
        color = cp['color']
        intrinsic_cvmat = cvCreateMat(3,3,cv.CV_32FC1)
        distortion_cvmat = cvCreateMat(1,4,cv.CV_32FC1)

        imat_np = np.array([[cp['focal_length_x_in_pixels'],0,
                             cp['optical_center_x_in_pixels']],
                            [0,cp['focal_length_y_in_pixels'],
                             cp['optical_center_y_in_pixels']],
                            [0,0,1]])
        intrinsic_cvmat = util.numpymat2cvmat(imat_np)

        dmat_np = np.array([[cp['lens_distortion_radial_1'],
                             cp['lens_distortion_radial_2'],
                             cp['lens_distortion_tangential_1'],
                             cp['lens_distortion_tangential_2']]])

        distortion_cvmat = util.numpymat2cvmat(dmat_np)
        undistort_mapx = cvCreateImage(size, IPL_DEPTH_32F, 1)
        undistort_mapy = cvCreateImage(size, IPL_DEPTH_32F, 1)
        cvInitUndistortMap(intrinsic_cvmat, distortion_cvmat,
                           undistort_mapx, undistort_mapy)

        if color == True:
            undistort_image = cvCreateImage(size, IPL_DEPTH_8U, 3)
        else:
            undistort_image = cvCreateImage(size, IPL_DEPTH_8U, 1)

        #pkl_name = glob.glob(opt.dir+'/handheld_pull_log*.pkl')[0]
        #d = ut.load_pickle(pkl_name)
        #video_name = opt.dir+'/'+d['video_name']
        #time_list = d['time_list']

        video_name = glob.glob(opt.dir + 'mechanism_video*.avi')[0]
        cap = cvCreateFileCapture(video_name)

        #for i in range(len(time_list)):
        i = 0
        while True:
            cvim = cvQueryFrame(cap)
            if cvim == None:
                break
            cvFlip(cvim, cvim)
            # undistort the image
            cvRemap(cvim, undistort_image, undistort_mapx, undistort_mapy,
                    CV_INTER_LINEAR, cvScalarAll(0)) 
            nm = opt.dir+'/%05d.png'%i
            print 'Saving', nm
            cvSaveImage(nm, undistort_image)
            i += 1

    if opt.bad:
        import cv

        l = ut.load_pickle(opt.dir+'/checker_fail_list.pkl')
        display = False

        if display:
            wnd = 'Checker Fail Images'
            cv.NamedWindow(wnd, cv.CV_WINDOW_AUTOSIZE)
        else:
            save_dir = opt.dir+'/checker_fail/'
            os.system('mkdir %s'%save_dir)
        for nm in l:
            name = opt.dir+'/'+nm
            cv_im = cv.LoadImage(name)
            if display:
                cv.ShowImage(wnd, cv_im)
                cv.WaitKey(0)
            else:
                save_dir = os.path.normpath(save_dir)
#                print 'save_dir:', save_dir
                file_name = '_'.join(save_dir.split('/')) + '_%s'%os.path.normpath(nm)
                print 'file_name:', file_name
                cv.SaveImage(save_dir + '/' + file_name, cv_im)




