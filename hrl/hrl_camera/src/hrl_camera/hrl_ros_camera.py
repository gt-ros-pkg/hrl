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


from sensor_msgs.msg import CameraInfo
#from sensor_msgs.msg import RegionOfInterest

#import pdb
#pdb.set_trace()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--n', action='store', type='string', dest='camera_name', 
                 default=None, help='a name in \'camera_config.py\'')

    p.add_option('--n2', action='store', type='string', dest='camera_name2', 
                 default=None, help='a name in \'camera_config.py\'')

    p.add_option('--f', action='store', type='float', dest='frame_rate', 
                 default=None, help='frame rate')

    p.add_option('--f2', action='store', type='float', dest='frame_rate2', 
                 default=None, help='frame rate')

    opt, args = p.parse_args()

    camera_configs = []
    for c, fps in zip([opt.camera_name, opt.camera_name2], [opt.frame_rate, opt.frame_rate2]):
        if c != None:
            camera_configs.append((c,fps))

    if len(camera_configs) <= 0:
        import camera_uuid as cu
        print 'This utility is used for publishing '
        print 'named and calibrated OpenCV accessible'
        print 'camera images over ROS.'
        print ''

        print 'You did not specify a camera name.'
        print 'Available cameras:', cu.camera_names()
        exit()
    
    cameras = []
    for camera_name, fps in camera_configs:
        topic_name = 'cvcamera_' + camera_name
        image_pub = rospy.Publisher(topic_name, Image)
        config_pub = rospy.Publisher(topic_name+'_info', CameraInfo)
        camera = hc.find_camera(camera_name)
        if fps != None:
            print 'Setting', camera_name, 'frame rate to', fps
            camera.set_frame_rate(fps)

        bridge = CvBridge()
        m = camera.intrinsic_cvmat
        intrinsic_list = [m[0,0], m[0,1], m[0,2], 0.0,
                          m[1,0], m[1,1], m[1,2], 0.0,
                          m[2,0], m[2,1], m[2,2], 0.0]
        cameras.append((camera_name, topic_name, camera, bridge, image_pub, config_pub, intrinsic_list))
    rospy.init_node('cvcamera', anonymous=True)

    for camera_name, topic_name, camera, _, _, _, _ in cameras:
        print '===================================================='
        print 'Opening OpenCV camera with ID', camera_name
        print 'Publishing on topic', topic_name
        print 'Camera operating at rate', camera.get_frame_rate()

    while not rospy.is_shutdown():
        try:
            for camera_name, topic_name, camera, bridge, image_pub, config_pub, intrinsic_list in cameras:
                cv_image = cv.CloneImage(camera.get_frame())
                rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
                image_pub.publish(rosimage)
                config_pub.publish(CameraInfo(P=intrinsic_list))
        except cam.NoFrameException, e:
            print e
        except rospy.exceptions.ROSSerializationException, e:
            print 'serialization exception'
        except CvBridgeError, e: 
            print e
            break
        except KeyboardInterrupt:
            print "Shutting down."
            break
        time.sleep(1/100.0)

    cv.DestroyAllWindows()
