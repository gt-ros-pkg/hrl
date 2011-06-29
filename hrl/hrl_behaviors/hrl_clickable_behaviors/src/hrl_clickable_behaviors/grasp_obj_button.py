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
from hrl_table_detect.srv import ObjectButtonDetect

class GraspObjButton:
    def __init__(self):
        self.arm = rospy.get_param("arm", "r")
        self.tl = TransformListener()
        self.perception_srv = rospy.Service("/clickable_world/detect_objects",
                                            PerceiveButtons,
                                            self.do_perception)
        self.action_srv = rospy.Service("/clickable_world/grasp_object",
                                        ButtonAction,
                                        self.do_action)

        self.obj_seg_srv = rospy.ServiceProxy("/object_button_detect",
                                              ObjectButtonDetect)
        self.grasp_client = actionlib.SimpleActionClient(self.arm + '_overhead_grasp', OverheadGraspAction)
        self.grasp_client.wait_for_server()
        self.grasp_setup_client = actionlib.SimpleActionClient(self.arm + '_overhead_grasp_setup', OverheadGraspSetupAction)
        self.grasp_setup_client.wait_for_server()

    def do_perception(self, req):
        rospy.loginfo("[grasp_obj_button] Perceiving object buttons...")
        # perceive object on table
        self.objects = self.obj_seg_srv().objects
        for i in range(len(self.objects)):
            self.objects[i].color.r = 256
            self.objects[i].color.g = 0
            self.objects[i].color.b = 0
            self.objects[i].color.a = 256

        # return object buttons
        resp = PerceiveButtonsResponse()
        resp.buttons = self.objects
        return resp

    def do_action(self, req):
        rospy.loginfo("[grasp_obj_button] Table clicked, grasping object...")
        # put 3d pt in grasping frame
        req.pixel3d.header.stamp = rospy.Time(0)
        grasp_pt = self.tl.transformPoint("/torso_lift_link", req.pixel3d)

        # move to grasp setup position
        setup_goal = OverheadGraspSetupGoal()
        setup_goal.disable_head = False
        setup_goal.open_gripper = True
        self.grasp_setup_client.send_goal(setup_goal)
        self.grasp_setup_client.wait_for_result()

        ############################################################
        # Creating grasp goal
        grasp_goal = OverheadGraspGoal()
        grasp_goal.is_grasp = True
        grasp_goal.disable_head = False
        grasp_goal.disable_coll = False
        grasp_goal.grasp_type=OverheadGraspGoal.VISION_GRASP
        grasp_goal.x = grasp_pt.point.x
        grasp_goal.y = grasp_pt.point.y
        grasp_goal.behavior_name = "overhead_grasp"
        grasp_goal.sig_level = 0.999
        ############################################################

        # grasp object
        self.grasp_client.send_goal(grasp_goal)
        self.grasp_client.wait_for_result()
        result = self.grasp_client.get_result()
        rospy.loginfo("[grasp_obj_button] Grasp result: %s" % result)
        obj_in_hand = result.grasp_result == "Object grasped"

        # go back to grasp setup
        setup_goal.open_gripper = False
        self.grasp_setup_client.send_goal(setup_goal)
        self.grasp_setup_client.wait_for_result()

        return ButtonActionResponse()

def main():
    rospy.init_node('grasp_obj_button')
    gob = GraspObjButton()
    rospy.spin()

if __name__ == "__main__":
    main()
