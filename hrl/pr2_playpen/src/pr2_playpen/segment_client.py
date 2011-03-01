#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_playpen')
from UI_segment_object.srv import GetObject
import rospy
import sys

import optparse
p = optparse.OptionParser()

p.add_option('--node', action='store', type='string', dest='node')
p.add_option('--serv', action='store', type='string', dest='service')

opt, args = p.parse_args()

rospy.init_node(opt.node)
rospy.wait_for_service(opt.service)
pub_filtered_cloud = rospy.ServiceProxy(opt.service, GetObject)
r = rospy.Rate(5)

while not rospy.is_shutdown():
    pub_filtered_cloud()
    r.sleep()
