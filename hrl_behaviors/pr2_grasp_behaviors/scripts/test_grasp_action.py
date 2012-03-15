#! /usr/bin/python
import random
import sys

import roslib; roslib.load_manifest("pr2_grasp_behaviors")
import rospy
import actionlib

from pr2_grasp_behaviors.msg import *
#from pr2_playpen.srv import *
            

def main():
    args = sys.argv
    rospy.init_node('test_grasping')
    if args[1] not in ['r', 'l']:
        print "First arg should be 'r' or 'l'"
        return
    grasp_client = actionlib.SimpleActionClient(args[1] + '_overhead_grasp', OverheadGraspAction)
    grasp_client.wait_for_server()
    grasp_setup_client = actionlib.SimpleActionClient(args[1] + '_overhead_grasp_setup', OverheadGraspSetupAction)
    grasp_setup_client.wait_for_server()
    print "grasp_client ready"
    if args[2] == 'visual' or args[2] == 'manual':
        results_dict = {}
        obj_in_hand = False
        while not rospy.is_shutdown():
            goal = OverheadGraspSetupGoal()
            #goal.disable_head = True
            goal.open_gripper = True
            grasp_setup_client.send_goal(goal)
            #grasp_setup_client.wait_for_result()
            rospy.sleep(1.0)

            if not obj_in_hand:
                print "Grasp started"

                ############################################################
                # Creating grasp goal
                goal = OverheadGraspGoal()
                goal.is_grasp = True
                goal.disable_head = False
                goal.disable_coll = False
                if args[2] == 'visual':
                    goal.grasp_type = OverheadGraspGoal.VISION_GRASP
                else:
                    goal.grasp_type = OverheadGraspGoal.MANUAL_GRASP
                goal.x = 0.5 + random.uniform(-0.1, 0.1)
                goal.y = 0.0 + random.uniform(-0.1, 0.1)
                goal.behavior_name = "overhead_grasp"
                goal.sig_level = 0.9975
                ############################################################

                rospy.sleep(1.0)
                grasp_client.send_goal(goal)
                rospy.sleep(1.0)
                grasp_client.wait_for_result()
                result = grasp_client.get_result()
                if result.grasp_result not in results_dict:
                    results_dict[result.grasp_result] = 0
                results_dict[result.grasp_result] += 1
                print "Last result:", result.grasp_result, "Totals:", results_dict
                obj_in_hand = result.grasp_result == "Object grasped"
            if obj_in_hand:

                ############################################################
                # Creating place goal
                goal = OverheadGraspGoal()
                goal.is_grasp = False
                goal.disable_head = False
                goal.disable_coll = False
                goal.grasp_type = OverheadGraspGoal.MANUAL_GRASP
                goal.x = 0.55 + random.uniform(-0.1, 0.1)
                goal.y = 0.0 + random.uniform(-0.2, 0.2)
                goal.behavior_name = "overhead_grasp"
                goal.sig_level = 0.9975
                ############################################################

                rospy.sleep(1.0)
                grasp_client.send_goal(goal)
                rospy.sleep(1.0)
                grasp_client.wait_for_result()
                result = grasp_client.get_result()
                if result.grasp_result not in results_dict:
                    results_dict[result.grasp_result] = 0
                results_dict[result.grasp_result] += 1
                obj_in_hand = result.grasp_result != "Object placed"
                print "Last result:", result.grasp_result, "Totals:", results_dict
        return
    print "Format: python test_grasp_action.py [r|l] [manual|visual]"

if __name__ == '__main__':
    main()
