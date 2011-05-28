#! /usr/bin/python

import numpy as np

import roslib; roslib.load_manifest('hrl_clickable_world')
import rospy
import std_msgs.msg

from hrl_clickable_world.srv import PerceiveButtons, ButtonAction, DisplayButtons
from hrl_clickable_world.srv import ButtonActionRequest
from hrl_clickable_world.srv import PerceiveButtonsResponse, ButtonActionResponse
from pixel_2_3d.srv import Pixel23d

NS_PREFIX = "/clickable_world/"

class BehaviorManager:
    def __init__(self):
        self.behavior_names = rospy.get_param(NS_PREFIX + "behavior_names")
        self.robot_state = rospy.get_param(NS_PREFIX + "init_conditions")
        self.srv_prefix = rospy.get_param(NS_PREFIX + "behavior_namespace")
        self.percieve_buttons_sub = rospy.Subscriber(NS_PREFIX + "perceive_buttons",
                                                  std_msgs.msg.Bool,
                                                  self.perceive_buttons)
        self.display_buttons_srv = rospy.ServiceProxy(NS_PREFIX + "display_buttons",
                                                      DisplayButtons)
        self.button_action_srv = rospy.Service(NS_PREFIX + "button_action",
                                               ButtonAction,
                                               self.on_button_press)
        self.pixel_2_3d_srv = rospy.ServiceProxy("/pixel_2_3d/pixel_2_3d",
                                                      Pixel23d)
        self.behaviors = {}
        self.perception_srvs = {}
        self.action_srvs = {}
        self.button_types = []
        for name in self.behavior_names:
            behavior = rospy.get_param(NS_PREFIX + name)
            if behavior["perception_srv"] not in self.perception_srvs:
                self.perception_srvs[behavior["perception_srv"]] = rospy.ServiceProxy(
                                    self.srv_prefix + behavior["perception_srv"], 
                                    PerceiveButtons)
            if behavior["action_srv"] not in self.action_srvs:
                self.action_srvs[behavior["action_srv"]] = rospy.ServiceProxy(
                                    self.srv_prefix + behavior["action_srv"], 
                                    ButtonAction)
            self.behaviors[name] = behavior

    def perceive_buttons(self, msg):
        rospy.loginfo("Perceiving buttons")
        self.button_types = []
        cur_button_id = 1
        displayed_buttons = []
        for name in self.behavior_names:
            # check to see if preconditions satisfied
            preconds_satisfied = True
            if "preconditions" in self.behaviors[name]:
                for condition in self.behaviors[name]["preconditions"]:
                    if (self.behaviors[name]["preconditions"][condition] != 
                                                         self.robot_state[condition]):
                        preconds_satisfied = False

            if preconds_satisfied:
                # get the buttons for this perception
                rospy.loginfo("Perceiving buttons for %s" % self.behaviors[name]["perception_srv"])
                try:
                    resp = self.perception_srvs[self.behaviors[name]["perception_srv"]]()
                except:
                    continue
                cur_buttons = resp.buttons
                for i, button in enumerate(cur_buttons):
                    self.button_types.append(name)
                    cur_buttons[i].id = cur_button_id
                    cur_buttons[i].ns = name
                    cur_button_id += 1
                displayed_buttons.extend(cur_buttons)
        # tell display to show buttons
        self.display_buttons_srv(displayed_buttons)

    def on_button_press(self, req):
        if req.button_id == 0:
            return
        do_behav = self.behaviors[self.button_types[req.button_id-1]]
        self.action_srvs[do_behav["action_srv"]](req)
        action_success = True
        # TODO use result from action srv
        if action_success:
            # update robot state
            if "postconditions" in do_behav:
                for condition in do_behav["postconditions"]:
                    self.robot_state[condition] = do_behav["postconditions"][condition]
        rospy.loginfo("Current State: " + str(self.robot_state))

def main():
    rospy.init_node('behavior_manager')
    bm = BehaviorManager()
    rospy.spin()

if __name__ == "__main__":
    main()
