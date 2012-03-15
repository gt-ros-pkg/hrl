#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

def main():
    rospy.init_node("load_tool")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-w', '--wait', dest="wait", default=6,
                 help="Set wait time.")
    p.add_option('-r', '--relax', dest="relax", default=False,
                 action="store_true", help="Set the gripper torque to 0.")
    p.add_option('-t', '--tighten', dest="tighten", default=False,
                 action="store_true", help="Set the gripper torque to 30.")
    (opts, args) = p.parse_args()

    g_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
    g_client.wait_for_server()

    g_goal = Pr2GripperCommandGoal()

    if opts.relax:
        g_goal.command.position = 0
        g_goal.command.max_effort = 0
        g_client.send_goal(g_goal)
        #g_client.wait_for_result()
        return

    if opts.tighten:
        g_goal.command.position = 0
        g_goal.command.max_effort = 30
        g_client.send_goal(g_goal)
        g_client.wait_for_result()
        return
        

    g_goal.command.position = 1
    g_goal.command.max_effort = -1
    g_client.send_goal(g_goal)
    g_client.wait_for_result()

    rospy.sleep(float(opts.wait))

    g_goal.command.position = 0
    g_goal.command.max_effort = 30
    g_client.send_goal(g_goal)
    g_client.wait_for_result()

if __name__ == "__main__":
    main()
