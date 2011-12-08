#!/usr/bin/python

import roslib
roslib.load_manifest('room_explore')
import rospy

import tf

from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped

import util as ut
import time

#srv_name = '/move_base_node/NavfnROS/make_plan'
srv_name = '/move_base_node/make_plan'

rospy.init_node('trial_explore')
listener = tf.TransformListener()
listener.waitForTransform('/base_link', '/map',
                          rospy.Time(0), timeout = rospy.Duration(100) )

get_plan = rospy.ServiceProxy( srv_name, GetPlan )
rospy.get_param('/move_base_node/global_costmap/raytrace_range')
rospy.get_param('/move_base_node/TrajectoryPlannerROS/xy_goal_tolerance')

req = GetPlanRequest()
req.tolerance = 0.5

req.start = ut.robot_in_map( listener )

req.goal.header.stamp = rospy.Time(0)
req.goal.header.frame_id = '/map'
req.goal.pose.position.x = 0.85
#req.goal.pose.position.y = 0.85
req.goal.pose.orientation.w = 1.0

res = get_plan( req )
found = bool( res.plan.poses != [] )
print res
print 'Found Path: ', found


for i in xrange( 100 ):
    t0 = time.time()
    res = get_plan( req )
    t1 = time.time()

    print 'Rate: ', 1.0 / (t1 - t0)
