#! /usr/bin/python
import random
import sys

import roslib; roslib.load_manifest("pr2_overhead_grasping")
import rospy
import actionlib

from pr2_overhead_grasping.msg import *
#from pr2_playpen.srv import *
from hrl_lib.keyboard_input import KeyboardInput
from pr2_overhead_grasping.helpers import log
            

if __name__ == '__main__':
    args = sys.argv
    rospy.init_node('do_dishes_client')
    ki = KeyboardInput()
    grasp_client = actionlib.SimpleActionClient('overhead_grasp', OverheadGraspAction)
    grasp_client.wait_for_server()
    grasp_setup_client = actionlib.SimpleActionClient('overhead_grasp_setup', OverheadGraspSetupAction)
    grasp_setup_client.wait_for_server()
    print "grasp_client ready"
    if args[1] == 'reg_test':
        results_dict = {}
        ki.kbhit()
        obj_in_hand = False
        while not rospy.is_shutdown():
            goal = OverheadGraspSetupGoal()
            #goal.disable_head = True
            grasp_setup_client.send_goal(goal)
            #grasp_setup_client.wait_for_result()
            rospy.sleep(1.0)

            if not obj_in_hand:
                print "Grasp started"
                goal = OverheadGraspGoal()
                goal.disable_head = False
                goal.is_grasp = True
                goal.grasp_type=OverheadGraspGoal.VISION_GRASP
                goal.x = 0.5 + random.uniform(-0.1, 0.1)
                goal.y = 0.0 + random.uniform(-0.1, 0.1)
                grasp_client.send_goal(goal)
                grasp_client.wait_for_result()
                result = grasp_client.get_result()
                if result.grasp_result not in results_dict:
                    results_dict[result.grasp_result] = 0
                results_dict[result.grasp_result] += 1
                print "Last result:", result.grasp_result, "Totals:", results_dict
                obj_in_hand = result.grasp_result == "Object grasped"
            if obj_in_hand:
                goal = OverheadGraspGoal()
                goal.disable_head = False
                goal.is_grasp = False
                goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
                goal.x = 0.55 + random.uniform(-0.1, 0.1)
                goal.y = 0.0 + random.uniform(-0.2, 0.2)
                grasp_client.send_goal(goal)
                grasp_client.wait_for_result()
                result = grasp_client.get_result()
                if result.grasp_result not in results_dict:
                    results_dict[result.grasp_result] = 0
                results_dict[result.grasp_result] += 1
                obj_in_hand = result.grasp_result != "Object placed"
                print "Last result:", result.grasp_result, "Totals:", results_dict
    
    elif args[1] == 'playpen_demo':
        rospy.wait_for_service('conveyor')
        conveyor = rospy.ServiceProxy('conveyor', Conveyor)
        print "conveyor ready"
        rospy.wait_for_service('playpen')
        playpen = rospy.ServiceProxy('playpen', Playpen)
        print "playpen ready"
        print "waiting for setup"
        rospy.sleep(20.0)
        while not rospy.is_shutdown():
            conveyor(0.10 * 2.)

            print "Grasp started"
            goal = OverheadGraspGoal()
            goal.is_grasp = True
            goal.grasp_type=OverheadGraspGoal.VISION_GRASP
            goal.x = 0.5
            goal.y = 0.0
            grasp_client.send_goal(goal)
            grasp_client.wait_for_result()
            result = grasp_client.get_result()
            goal = OverheadGraspGoal()
            goal.is_grasp = False
            goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
            goal.x = 0.5 + random.uniform(-0.1, 0.1)
            goal.y = 0.0 + random.uniform(-0.1, 0.1)
            grasp_client.send_goal(goal)
            grasp_client.wait_for_result()
            result = grasp_client.get_result()

            # do playpen stuff

            log("Opening trap door")
            playpen(1)
            rospy.sleep(0.2)
            log("Closing trap door")
            playpen(0)
            rospy.sleep(0.2)



