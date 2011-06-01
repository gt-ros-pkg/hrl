#! /usr/bin/python
import random
import sys

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy
import actionlib

from hrl_arm_move_behaviors.msg import LinearMoveRelativeAction, LinearMoveRelativeGoal

def main():
    rospy.init_node('test_linear_move')
    args = sys.argv
    move_client = actionlib.SimpleActionClient(args[1] + '_linear_arm_move/move_relative', 
                                               LinearMoveRelativeAction)
    move_client.wait_for_server()
    goal = LinearMoveRelativeGoal()
    goal.distance = 0.15
    move_client.send_goal(goal)
    move_client.wait_for_result()
    result = move_client.get_result()

if __name__ == '__main__':
    main()
