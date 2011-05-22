#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_behaviors')
import rospy
import std_srvs.srv

from hrl_clickable_world.srv import PerceiveClickables, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import PerceiveClickablesResponse, ButtonActionResponse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from hrl_table_detect.srv import SegmentSurfaces, GetTableApproaches
from hrl_table_detect.srv import GetTableApproachesRequest

class TableClickable:
    def __init__(self):
        self.perception_srv = rospy.Service("/clickable_world/table_perception",
                                            PerceiveClickables,
                                            self.do_perception)
        self.percept_pub = rospy.Publisher("/clickable_world/table_button_vis",
                                           Marker)
        self.action_srv = rospy.Service("/clickable_world/table_approach_action",
                                        ButtonAction,
                                        self.do_action)
        self.pc_capture_srv = rospy.ServiceProxy("/table_detection/surf_seg_capture_pc",
                                                 std_srvs.srv.Empty)
        self.table_seg_srv = rospy.ServiceProxy("/table_detection/segment_surfaces",
                                                SegmentSurfaces)
        self.table_approach_srv = rospy.ServiceProxy(
                                         "/table_detection/detect_table_approaches",
                                         GetTableApproaches)

    def do_perception(self, req):
        # capture a few clouds
        rate = rospy.Rate(5)
        for i in range(5):
            self.pc_capture_srv()
            rate.sleep()
            
        # segment surfaces
        self.surfaces = self.table_seg_srv().surfaces
        self.percept_pub.publish(self.surfaces[0])

        resp = PerceiveClickablesResponse()
        resp.buttons = self.surfaces
        return resp

    def do_action(self, req):
        rospy.loginfo("Table clicked!")
        approach_req = GetTableApproachesRequest()
        approach_req.table = self.surfaces[0]
        approach_req.approach_pt = req.click_loc
        self.table_approach_srv(approach_req)
        return ButtonActionResponse()

def main():
    rospy.init_node('table_clickable')
    tc = TableClickable()
    rospy.spin()

if __name__ == "__main__":
    main()
