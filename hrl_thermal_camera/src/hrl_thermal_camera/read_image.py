#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_thermal_camera')
import rospy
from sensor_msgs.msg import Image



class readImage(object):
    def __init__(self):
        self.image_reader = rospy.Subscriber('/thermal_cam/image_raw', Image, self.image_reader)

    def image_reader(self, msg):
        print msg.height
        print msg.width
        print msg.encoding
        print msg.is_bigendian
        print msg.step
        print len(msg.data)
        print int(msg.data[173160], )


if __name__ == '__main__':
    rospy.init_node('thermal_tf_broadcaster')
    readimage = readImage()
    rospy.spin()
