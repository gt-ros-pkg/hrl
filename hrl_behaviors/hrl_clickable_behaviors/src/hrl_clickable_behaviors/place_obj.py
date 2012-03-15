#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_behaviors')
import rospy
import std_srvs.srv
import actionlib
from tf.listener import TransformListener

from hrl_clickable_world.srv import PerceiveButtons, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import PerceiveButtonsResponse, ButtonActionResponse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from pr2_grasp_behaviors.msg import OverheadGraspSetupAction, OverheadGraspSetupGoal
from pr2_grasp_behaviors.msg import OverheadGraspAction, OverheadGraspGoal
from hrl_table_detect.srv import ObjectButtonDetect, SegmentSurfaces

class PlaceObject:
    def __init__(self):
        self.arm = rospy.get_param("arm", "r")
        self.tl = TransformListener()
        self.perception_srv = rospy.Service("/clickable_world/place_table_perceive",
                                            PerceiveButtons,
                                            self.do_perception)
        self.action_srv = rospy.Service("/clickable_world/place_object",
                                        ButtonAction,
                                        self.do_action)

        self.pc_capture_srv = rospy.ServiceProxy("/table_detection/surf_seg_capture_pc",
                                                 std_srvs.srv.Empty)
        self.table_seg_srv = rospy.ServiceProxy("/table_detection/segment_surfaces",
                                                SegmentSurfaces)
        self.grasp_client = actionlib.SimpleActionClient(self.arm + '_overhead_grasp', OverheadGraspAction)
        self.grasp_client.wait_for_server()
        self.grasp_setup_client = actionlib.SimpleActionClient(self.arm + '_overhead_grasp_setup', OverheadGraspSetupAction)
        self.grasp_setup_client.wait_for_server()

    def do_perception(self, req):
        rospy.loginfo("[place_object] Perceiving table...")
        # capture a few clouds
        rate = rospy.Rate(5)
        for i in range(5):
            self.pc_capture_srv()
            rate.sleep()
            
        # segment surfaces
        self.surfaces = self.table_seg_srv().surfaces
        for i in range(len(self.surfaces)):
            self.surfaces[i].color.r = 256
            self.surfaces[i].color.g = 0
            self.surfaces[i].color.b = 256
            self.surfaces[i].color.a = 256

        resp = PerceiveButtonsResponse()
        resp.buttons = self.surfaces
        return resp

    def do_action(self, req):
        rospy.loginfo("[place_object] Table clicked, placing object...")
        # put 3d pt in grasping frame
        req.pixel3d.header.stamp = rospy.Time(0)
        place_pt = self.tl.transformPoint("/torso_lift_link", req.pixel3d)

        # move to place setup position
        setup_goal = OverheadGraspSetupGoal()
        setup_goal.disable_head = False
        setup_goal.open_gripper = False
        self.grasp_setup_client.send_goal(setup_goal)
        self.grasp_setup_client.wait_for_result()

        ############################################################
        # Creating place goal
        grasp_goal = OverheadGraspGoal()
        grasp_goal.is_grasp = False
        grasp_goal.disable_head = False
        grasp_goal.disable_coll = False
        grasp_goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
        grasp_goal.x = place_pt.point.x
        grasp_goal.y = place_pt.point.y
        grasp_goal.behavior_name = "overhead_grasp"
        grasp_goal.sig_level = 0.999
        ############################################################

        # place object
        self.grasp_client.send_goal(grasp_goal)
        self.grasp_client.wait_for_result()
        result = self.grasp_client.get_result()
        rospy.loginfo("[place_object] Place result: %s" % result)
        obj_in_hand = result.grasp_result == "Object placed"
        
        # go back to grasp setup
        setup_goal.open_gripper = True
        self.grasp_setup_client.send_goal(setup_goal)
        self.grasp_setup_client.wait_for_result()

        return ButtonActionResponse()

def main():
    rospy.init_node('place_object')
    po = PlaceObject()
    rospy.spin()

if __name__ == "__main__":
    main()
