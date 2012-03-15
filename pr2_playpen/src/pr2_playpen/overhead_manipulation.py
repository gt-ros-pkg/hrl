#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#Based on some code by Kaijen, modified heavily by Marc Killpack


import roslib
roslib.load_manifest('pr2_playpen')
import rospy
from object_manipulator.convert_functions import *
from pr2_playpen.srv import Playpen
from pr2_playpen.srv import Conveyor
from pr2_overhead_grasping.msg import *
import math
import numpy as np

class SimplePickAndPlaceExample():

    def __init__(self):

        rospy.loginfo("initializing overhead grasping")
	grasp_client = actionlib.SimpleActionClient('overhead_grasp', OverheadGraspAction)
	grasp_client.wait_for_server()
	grasp_setup_client = actionlib.SimpleActionClient('overhead_grasp_setup', OverheadGraspSetupAction)
	grasp_setup_client.wait_for_server()
        rospy.loginfo("finished initializing overhead grasping")
        rospy.wait_for_service('playpen')
        rospy.wait_for_service('conveyor')
        self.playpen = rospy.ServiceProxy('playpen', Playpen)
        self.conveyor = rospy.ServiceProxy('conveyor', Conveyor)
        self.objects_dist = [.135, .26-.135, .405-.26, .545-.405, 
                             .70-.545, .865-.70, .995-.865, 1.24-.995]
        self.tries = 0
        self.successes = 0
    #pick up the nearest object to PointStamped target_point with whicharm 
    #(0=right, 1=left)
    def pick_up_object_near_point(self, target_point, whicharm):

        rospy.loginfo("moving the arms to the side")
	setup_goal = OverheadGraspSetupGoal()
	setup_goal.disable_head = True
	grasp_setup_client.send_goal(setup_goal)
	rospy.sleep(1.0)

	goal = OverheadGraspGoal()
	goal.grasp_type = OverheadGraspGoal.VISION_GRASP
	goal.x = target_point[0]
	goal.y = target_point[1]
	grasp_client.send_goal(goal)
	grasp_client.wait_for_result()

	# TODO add success checking
	success = True

        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            self.papm.move_arm_to_side(whicharm)
        else:
            rospy.loginfo("pick-up failed.")

        return success


    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, target_point):

        self.papm.set_place_area(place_rect_center, place_rect_dims)

        rospy.loginfo("putting down the object in the %d gripper" % whicharm)

	goal = OverheadGraspGoal()
	goal.grasp_type = OverheadGraspGoal.MANUAL_GRASP
	goal.x = target_point[0]
	goal.y = target_point[1]
	grasp_client.send_goal(goal)
	grasp_client.wait_for_result()

        if success:
            rospy.loginfo("place returned success")
        else:
            rospy.loginfo("place returned failure")

        return success


if __name__ == "__main__":
    rospy.init_node('simple_pick_and_place_example')
    sppe = SimplePickAndPlaceExample()


    #adjust for your table 
    table_height = 0.529

############here is where we put the loop and service calls to know if successful or not...., when done write success rate/statistics to pkl file
############and call service to advance to next object, etc.

    #.5 m in front of robot, centered
    target_point_xyz = [.625, 0, table_height]   #this is currently an approximation/hack should us ar tag
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    arm = 0

    for i in xrange(len(sppe.objects_dist)):
        sppe.playpen(0)
        sppe.conveyor(sppe.objects_dist[i])

        while sppe.tries<2:
            print "arm is ", arm
#            print "target _point is ", target_point.x
            success = sppe.pick_up_object_near_point(target_point_xyz, arm)   #right arm

            if success:
                sppe.successes=sppe.successes + 1
                #square of size 30 cm by 30 cm
                place_rect_dims = [.1, .1]

                
                #.5 m in front of robot, to the right
                radius = np.random.uniform(0,0.20, 1)[0]
                angle = np.random.uniform(0, 2*math.pi, 1)[0]
                center_xyz = [.625+math.cos(angle)*radius, math.sin(angle)*radius, table_height+.2]

                sppe.place_object(arm, center_xyz)

            arm = arm.__xor__(1)
            sppe.tries = sppe.tries+1

        sppe.playpen(1)
        sppe.successes = 0
        sppe.tries = 0

#    sppe.playpen(0)
