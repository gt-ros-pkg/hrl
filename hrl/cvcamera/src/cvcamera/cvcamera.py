#!/usr/bin/env python
import roslib
roslib.load_manifest('cvcamera')
import sys
import rospy
import cv
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    camera_id = int(sys.argv[1])
    topic_name = 'cvcamera' + str(camera_id)

    image_pub = rospy.Publisher(topic_name, Image)
    rospy.init_node('cvcamera', anonymous=True)

    capture = cv.CaptureFromCAM(camera_id)
    bridge = CvBridge()

    print 'Opening OpenCV camera with ID', camera_id
    print 'Publishing on topic', topic_name
    while not rospy.is_shutdown():
        try:
            cv_image = cv.CloneImage(cv.QueryFrame(capture))
            rosimage = bridge.cv_to_imgmsg(cv_image, "bgr8")
            image_pub.publish(rosimage)
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
