#!/usr/bin/env python
##
# This utility is used for publishing 
# named and calibrated OpenCV accessible
# camera images over ROS.
#
# Usage ./ros_camera CAMERA_NAME
# where CAMERA_NAME is a name 
# in camera_config.py
#

import roslib
roslib.load_manifest('hrl_camera')
import rospy
import cv
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import hrl_camera.hrl_camera as hc
import hrl_camera.camera as cam
from camera_config import camera_parameters as ccp

from sensor_msgs.msg import CameraInfo


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--cam_list', action='store', type='string', dest='cam_list', 
                 default=None, help='colon (:) separated list of camera names')

    opt, args = p.parse_args()

    if opt.cam_list == None:
        import camera_uuid as cu
        rospy.logout('This utility is used for publishing ')
        rospy.logout('named and calibrated OpenCV accessible')
        rospy.logout('camera images over ROS.')
        rospy.logout('')
        rospy.logout('You did not specify a camera name.')
        rospy.logout('Available cameras:' + str(cu.camera_names()))
        exit()

    cam_name_list = opt.cam_list.split(':')

    bridge = CvBridge()
    from socket import gethostname
    rospy.init_node('camera_publisher_'+gethostname(), anonymous=False)

    cameras = []
    image_pub_l = []
    caminfo_pub_l = []

    for c in cam_name_list:
        camera = hc.find_camera(c)
        rospy.logout('====================================================')
        if 'frame_rate' in ccp[c]:
            fps = ccp[c]['frame_rate']
            rospy.logout('Setting '+c+' frame rate to %.2f'%(fps))
            camera.set_frame_rate(fps)

        if 'ros_topic' in ccp[c]:
            topic_name = ccp[c]['ros_topic']
        else:
            topic_name = 'cvcamera_' + c

        image_pub = rospy.Publisher(topic_name, Image)
        config_pub = rospy.Publisher(topic_name+'_info', CameraInfo)

        #camera.set_brightness(*map(int, opt.cam_settings.split('_')))

        rospy.logout('Opening OpenCV camera with ID ' + c)
        rospy.logout('Publishing on topic ' + topic_name)
        rospy.logout('Camera operating at rate %.2f'%(camera.get_frame_rate()))

        cameras.append(camera)
        image_pub_l.append(image_pub)
        caminfo_pub_l.append(config_pub)

    n_cameras = len(cameras)

    while not rospy.is_shutdown():
        for i in range(n_cameras):
            camera, image_pub, config_pub = cameras[i], image_pub_l[i], caminfo_pub_l[i]
            try:
                cv_image = cv.CloneImage(camera.get_frame())
                #cv_image = cv.CloneImage(camera.get_frame_debayered()) # for calibration
                rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
                image_pub.publish(rosimage)
                m = camera.intrinsic_cvmat
                intrinsic_list = [m[0,0], m[0,1], m[0,2], 0.0, m[1,0], m[1,1], m[1,2], 0.0, m[2,0], m[2,1], m[2,2], 0.0]
                config_pub.publish(CameraInfo(P=intrinsic_list))
            except rospy.exceptions.ROSSerializationException, e:
                rospy.logerr('serialization exception')
            except CvBridgeError, e: 
                print e
                break
            except KeyboardInterrupt:
                rospy.logout("Shutting down.")
                break
        time.sleep(1/100.0)

    cv.DestroyAllWindows()


