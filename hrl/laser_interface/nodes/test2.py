import roslib; roslib.load_manifest('laser_interface')
import rospy
import message_filters

import cv
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image

import sys
import util as ut
import hrl_lib.rutils as ru

bridge = CvBridge()

def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv.DestroyAllWindows()

def callback(left, right):
    try:
        left_img = bridge.imgmsg_to_cv(left, "bgr8") 
        right_img = bridge.imgmsg_to_cv(right, "bgr8") 
    except CvBridgeError, e: 
        print e
    cv.ShowImage("left", left_img) 
    cv.ShowImage("right", right_img) 
    cv.WaitKey(33)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    topics = ["/wide_stereo/left/image_color", "/wide_stereo/right/image_color"]

    left  = message_filters.Subscriber(topics[0], Image)
    right = message_filters.Subscriber(topics[1], Image)
    ts = message_filters.TimeSynchronizer([left, right], 10)

    cv.NamedWindow("left", 1)
    cv.NamedWindow("right", 1)

    ts.registerCallback(callback)

    try:
        rospy.spin()
    except KeyboardInterrupt, e:
        pass

    #topics = ["/wide_stereo/left/local_image_color", "/wide_stereo/right/local_image_color"]
