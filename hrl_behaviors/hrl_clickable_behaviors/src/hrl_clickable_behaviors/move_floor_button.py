#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_behaviors')
import rospy
import std_srvs.srv
import actionlib
import tf.transformations as tf_trans
from tf.listener import TransformListener
from geometry_msgs.msg import Quaternion

from hrl_clickable_world.srv import PerceiveButtons, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import PerceiveButtonsResponse, ButtonActionResponse
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from hrl_move_floor_detect.srv import SegmentFloor

class MoveFloorButton:
    def __init__(self):
        self.tl = TransformListener()
        self.perception_srv = rospy.Service("/clickable_world/detect_empty_floor",
                                            PerceiveButtons,
                                            self.do_perception)
        self.percept_pub = rospy.Publisher("/clickable_world/floor_button_vis",
                                           Marker)
        self.goal_pub = rospy.Publisher("/clickable_world/move_floor_goal", PoseStamped)
        self.action_srv = rospy.Service("/clickable_world/move_empty_floor",
                                        ButtonAction,
                                        self.do_action)
        self.floor_seg_srv = rospy.ServiceProxy("/move_floor_detect",
                                                SegmentFloor)
        self.floor_move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[move_floor_button] MoveFloorButton loaded.")

    def do_perception(self, req):
        # segment surfaces
        rospy.loginfo("[move_floor_button] Segmenting floor")
        self.surfaces = self.floor_seg_srv().surfaces
        for i in range(len(self.surfaces)):
            self.surfaces[i].color.r = 0
            self.surfaces[i].color.g = 0
            self.surfaces[i].color.b = 256
            self.surfaces[i].color.a = 256
        if len(self.surfaces) != 0:
            self.percept_pub.publish(self.surfaces[0])

        resp = PerceiveButtonsResponse()
        resp.buttons = self.surfaces
        return resp

    def do_action(self, req):
        rospy.loginfo("[move_floor_button] MoveFloorButton clicked!")
        # put 3d pt in base frame
        req.pixel3d.header.stamp = rospy.Time(0)
        move_pt = self.tl.transformPoint("/base_link", req.pixel3d)

        floor_move_goal = MoveBaseGoal()
        floor_move_goal.target_pose.header.frame_id = move_pt.header.frame_id
        floor_move_goal.target_pose.pose.position = move_pt.point
        quat = tf_trans.quaternion_from_euler(0, 0, np.arctan2(move_pt.point.y, move_pt.point.x))
        floor_move_goal.target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        floor_move_goal.target_pose.header.frame_id = "/base_link"
        floor_move_goal.target_pose.header.stamp = rospy.Time.now()
        try:
            self.goal_pub.publish(floor_move_goal.target_pose)
            self.floor_move_client.send_goal(floor_move_goal)
            self.floor_move_client.wait_for_result()
            cur_pose = self.floor_move_client.get_result()
        except rospy.ROSInterruptException:
            print "[move_floor_button] Move floor failed"
        return ButtonActionResponse()

def main():
    rospy.init_node('move_floor_button')
    tb = MoveFloorButton()
    rospy.spin()

if __name__ == "__main__":
    main()
