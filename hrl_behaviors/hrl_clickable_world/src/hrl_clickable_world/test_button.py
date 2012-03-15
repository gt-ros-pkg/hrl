#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_world')
import rospy
import std_srvs

from hrl_clickable_world.srv import PerceiveButtons, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import PerceiveButtonsResponse, ButtonActionResponse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class TestButton:
    def __init__(self):
        self.perception_srv = rospy.Service("/clickable_world/test_button_perception",
                                            PerceiveButtons,
                                            self.do_perception)
        self.percept_pub = rospy.Publisher("/clickable_world/test_button_vis",
                                           Marker)
        self.action_srv = rospy.Service("/clickable_world/test_button_action",
                                        ButtonAction,
                                        self.do_action)

    def do_perception(self, req):
        button = Marker()
        button.header.frame_id = "/base_link"
        button.header.stamp = rospy.Time.now()
        button.type = Marker.LINE_STRIP
        button.ns = "buttons"
        button.color.r = 1; button.color.a = 1
        button.scale.x = 0.01; button.scale.y = 0.01; button.scale.z = 0.01; 
        button.pose.orientation.w = 1
        polygon = [(1.5, 0.5), (2.5, 0.5), (2.5, -0.5), (1.5, -0.5), (1.5, 0.5)]
        for pt in polygon:
            point = Point()
            point.x = pt[0]; point.y = pt[1]; point.z = 0.5
            button.points.append(point)
        self.percept_pub.publish(button)

        resp = PerceiveButtonsResponse()
        resp.buttons.append(button)
        return resp

    def do_action(self, req):
        rospy.loginfo("TestButton clicked!")
        rospy.sleep(10)
        rospy.loginfo("TestButton done!")
        return ButtonActionResponse()

def main():
    rospy.init_node('test_button')
    tb = TestButton()
    rospy.spin()

if __name__ == "__main__":
    main()
