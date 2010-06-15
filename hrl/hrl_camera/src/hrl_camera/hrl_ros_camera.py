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
#import hrl_camera.camera as cam

from socket import gethostname

from sensor_msgs.msg import CameraInfo


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--n', action='store', type='string', dest='camera_name', 
                 default=None, help='a name in \'camera_config.py\'')

    p.add_option('--f', action='store', type='float', dest='frame_rate', 
                 default=None, help='frame rate')

    p.add_option('--topic', action='store', type='string', dest='topic', 
                 default=None, help='topic to publish images to')

    p.add_option('--camera_settings', action='store', type='string',
                 dest='cam_settings', default=None,
                 help='<brightness>_<shutter>_<gain>_<exposure>')

    opt, args = p.parse_args()

    camera_name, fps = opt.camera_name, opt.frame_rate

    if camera_name == None:
        rospy.logout('This utility is used for publishing ')
        rospy.logout('named and calibrated OpenCV accessible')
        rospy.logout('camera images over ROS.')
        rospy.logout('')
        rospy.logout('You did not specify a camera name.')
        rospy.logout('Available cameras:' + str(cu.camera_names()))
        exit()
    
    cameras = []
    if opt.topic == None:
        topic_name = 'cvcamera_' + camera_name
    else:
        topic_name = opt.topic

    image_pub = rospy.Publisher(topic_name, Image)
    config_pub = rospy.Publisher(topic_name+'_info', CameraInfo)

    hst_nm = gethostname()
    if rospy.has_param(hst_nm+'/firewire_camera_ids') == False:
        # the first access to the firewire bus, get all the camera
        # info and save on the parameter server with the host name
        print '>>>>>>>>> AAAAAAAAAHHHHHHHHHH <<<<<<<<<<<'
        import camera_uuid as cu
        d = cu.camera_names()
        for k in d.keys():
            rospy.set_param(k+'/id', d[k])
        rospy.set_param(hst_nm+'/firewire_camera_ids', True)
    camera = hc.find_camera(camera_name, rospy.get_param(camera_name+'/id'))

#    if fps != None:
#        rospy.logout('Setting '+camera_name+' frame rate to %.2f'%(fps))
#        camera.set_frame_rate(fps)
#    
#    if opt.cam_settings != None:
#        camera.set_brightness(*map(int, opt.cam_settings.split('_')))
#
#    bridge = CvBridge()
#    m = camera.intrinsic_cvmat
#    intrinsic_list = [m[0,0], m[0,1], m[0,2], 0.0,
#                      m[1,0], m[1,1], m[1,2], 0.0,
#                      m[2,0], m[2,1], m[2,2], 0.0]
##    cameras.append((camera_name, topic_name, camera, bridge, image_pub, config_pub, intrinsic_list))
#    rospy.init_node('camera_'+camera_name, anonymous=False)
#
#    rospy.logout('====================================================')
#    rospy.logout('Opening OpenCV camera with ID ' + camera_name)
#    rospy.logout('Publishing on topic ' + topic_name)
#    rospy.logout('Camera operating at rate %.2f'%(camera.get_frame_rate()))
#
#    while not rospy.is_shutdown():
#        try:
#            cv_image = cv.CloneImage(camera.get_frame())
#            #cv_image = cv.CloneImage(camera.get_frame_debayered()) # for calibration
#            rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
#            image_pub.publish(rosimage)
#            config_pub.publish(CameraInfo(P=intrinsic_list))
#        except rospy.exceptions.ROSSerializationException, e:
#            rospy.logerr('serialization exception')
#        except CvBridgeError, e: 
#            print e
#            break
#        except KeyboardInterrupt:
#            rospy.logout("Shutting down.")
#            break
#        time.sleep(1/100.0)
#
#    cv.DestroyAllWindows()


