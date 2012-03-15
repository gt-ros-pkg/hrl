#!/usr/bin/env python
import roslib
roslib.load_manifest('hrl_camera')
import ros_camera as rc
import sys
import cv
import rospy

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Views images published through ROS.'
        print 'Usage: ./image_listener.py ROS_TOPIC_NAME'
    else:
        ros_topic_name = sys.argv[1]
        camera = rc.ROSImageClient(ros_topic_name)
        cv.NamedWindow(ros_topic_name, cv.CV_WINDOW_AUTOSIZE)
    
        while not rospy.is_shutdown():
            f = camera.get_frame()
            cv.ShowImage(ros_topic_name, f)
            cv.WaitKey(10)

