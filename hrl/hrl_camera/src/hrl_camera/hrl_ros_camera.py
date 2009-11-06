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

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('-n', action='store', type='string', dest='camera_name', 
                 default=None, help='a name in \'camera_config.py\'')
    p.add_option('-f', action='store', type='float', dest='frame_rate', 
                 default=None, help='frame rate')
    opt, args = p.parse_args()

    frame_rate = opt.frame_rate
    camera_name = opt.camera_name
    if camera_name == None:
        import camera_uuid as cu
        print 'This utility is used for publishing '
        print 'named and calibrated OpenCV accessible'
        print 'camera images over ROS.'
        print ''

        print 'You did not specify a camera name.'
        print 'Available cameras:', cu.camera_names()
        exit()

    topic_name = 'cvcamera_' + camera_name

    image_pub = rospy.Publisher(topic_name, Image)
    rospy.init_node('cvcamera', anonymous=True)
    camera = hc.find_camera(camera_name)
    if frame_rate != None:
        print 'Setting frame rate to', frame_rate
        camera.set_frame_rate(frame_rate)
    bridge = CvBridge()

    print 'Opening OpenCV camera with ID', camera_name
    print 'Publishing on topic', topic_name
    while not rospy.is_shutdown():
        try:
            1+1
            #cv_image = cv.CloneImage(camera.get_frame())
            #rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
            #image_pub.publish(rosimage)
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
