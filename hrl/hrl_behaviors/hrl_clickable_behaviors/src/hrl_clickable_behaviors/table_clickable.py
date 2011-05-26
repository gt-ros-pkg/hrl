#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_behaviors')
import rospy
import std_srvs.srv
import actionlib

from hrl_clickable_world.srv import PerceiveButtons, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import PerceiveButtonsResponse, ButtonActionResponse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from hrl_table_detect.srv import SegmentSurfaces, GetTableApproaches
from hrl_table_detect.srv import GetTableApproachesRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pr2_approach_table.srv import ApproachSrv

class TableClickable:
    def __init__(self):
        self.perception_srv = rospy.Service("/clickable_world/table_perception",
                                            PerceiveButtons,
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
        self.table_approach_detect_srv = rospy.ServiceProxy(
                                         "/table_detection/detect_table_approaches",
                                         GetTableApproaches)
        self.table_move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.approach_table_srv = rospy.ServiceProxy("/approach_table/move_forward_srv",
                                                     ApproachSrv)

    def do_perception(self, req):
        # capture a few clouds
        rate = rospy.Rate(5)
        for i in range(5):
            self.pc_capture_srv()
            rate.sleep()
            
        # segment surfaces
        self.surfaces = self.table_seg_srv().surfaces
        for i in range(len(self.surfaces)):
            self.surfaces[i].color.r = 0
            self.surfaces[i].color.g = 256
            self.surfaces[i].color.b = 0
            self.surfaces[i].color.a = 256
        self.percept_pub.publish(self.surfaces[0])

        resp = PerceiveButtonsResponse()
        resp.buttons = self.surfaces
        return resp

    def do_action(self, req):
        rospy.loginfo("Table clicked!")
        approach_req = GetTableApproachesRequest()
        approach_req.table = self.surfaces[0]
        approach_req.approach_pt = req.pixel3d
        approach_poses = self.table_approach_detect_srv(approach_req).approach_poses
        #return ButtonActionResponse()

        table_move_goal = MoveBaseGoal()
        for approach_pose in approach_poses.poses:
            table_move_goal.target_pose.pose = approach_pose
            table_move_goal.target_pose.header.frame_id = approach_poses.header.frame_id
            table_move_goal.target_pose.header.stamp = rospy.Time.now()
            try:
                self.table_move_client.send_goal(table_move_goal)
                self.table_move_client.wait_for_result()
                cur_pose = self.table_move_client.get_result()
                try:
                    self.approach_table_srv()
                    return ButtonActionResponse()
                except rospy.ROSInterruptException:
                    print "Table approach failed"
            except rospy.ROSInterruptException:
                print "Table move failed"
        return ButtonActionResponse()

def main():
    rospy.init_node('table_clickable')
    tc = TableClickable()
    rospy.spin()

if __name__ == "__main__":
    main()
