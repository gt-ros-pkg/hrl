#! /usr/bin/python
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
roslib.load_manifest('pr2_grasp_behaviors')
import rospy
import actionlib
from object_manipulator.convert_functions import *
from pr2_playpen.srv import Playpen
from pr2_playpen.srv import Conveyor
from pr2_playpen.srv import Check
from pr2_playpen.srv import Train
from UI_segment_object.srv import Save
#from pr2_overhead_grasping.msg import *
from pr2_grasp_behaviors.msg import *
import os
import datetime
import cPickle

import math
import numpy as np

SAVE = True

class SimplePickAndPlaceExample():

    def __init__(self):

        print "waiting for conveyor"
        rospy.wait_for_service('playpen')
        rospy.wait_for_service('conveyor')
        print "started conveyor and playpen"
        self.playpen = rospy.ServiceProxy('playpen', Playpen)
        self.conveyor = rospy.ServiceProxy('conveyor', Conveyor)
        # self.objects_dist = [.135, .26-.135, .405-.26, .545-.405, 
        #                      .70-.545, .865-.70, .995-.865, 1.24-.995,
        #                      .135, .26-.135, .405-.26, .545-.405, 
        #                      .70-.545, .865-.70, .995-.865, 1.24-.995]
        self.objects_dist = [0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12,
                             0.12, 0.12, 0.12]

        self.tries = 0
        self.successes = 0

        self.grasp_client = [None, None]
        self.grasp_setup_client = [None, None]

        self.grasp_client[0] = actionlib.SimpleActionClient('r_overhead_grasp', OverheadGraspAction)
        print "died before r_overhead_grasp"
        self.grasp_client[0].wait_for_server()
        
        self.grasp_setup_client[0] = actionlib.SimpleActionClient('r_overhead_grasp_setup', OverheadGraspSetupAction)
        self.grasp_setup_client[0].wait_for_server()
        self.grasp_client[1] = actionlib.SimpleActionClient('l_overhead_grasp', OverheadGraspAction)
        self.grasp_client[1].wait_for_server()
        self.grasp_setup_client[1] = actionlib.SimpleActionClient('l_overhead_grasp_setup', OverheadGraspSetupAction)
        self.grasp_setup_client[1].wait_for_server()

    def move_to_side(self, whicharm, open_gripper=False):
        rospy.loginfo("moving the arms to the side")
        setup_goal = OverheadGraspSetupGoal()
        setup_goal.disable_head = True
        setup_goal.open_gripper = open_gripper
        self.grasp_setup_client[whicharm].send_goal(setup_goal)
        self.grasp_setup_client[whicharm].wait_for_result()

    #pick up the nearest object to PointStamped target_point with whicharm 
    #(0=right, 1=left)
    def pick_up_object_near_point(self, target_point, whicharm):
        self.move_to_side(whicharm, False)
#############once is it positioned, we don't want to move the head at all !!!#############
#        rospy.loginfo("pointing the head at the target point")
#        self.papm.point_head(get_xyz(target_point.point),
#                             target_point.header.frame_id)
#########################################################################################        

        rospy.loginfo("picking up the nearest object to the target point")
        ############################################################
        # Creating grasp grasp_goal
        grasp_goal = OverheadGraspGoal()
        grasp_goal.is_grasp = True
        grasp_goal.disable_head = True
        grasp_goal.disable_coll = False
        grasp_goal.grasp_type=OverheadGraspGoal.VISION_GRASP
        grasp_goal.grasp_params = [0] * 3
        grasp_goal.grasp_params[0] = target_point.point.x
        grasp_goal.grasp_params[1] = target_point.point.y
        grasp_goal.behavior_name = "overhead_grasp"
        grasp_goal.sig_level = 0.999
        ############################################################

        self.grasp_client[whicharm].send_goal(grasp_goal)
        self.grasp_client[whicharm].wait_for_result()
        result = self.grasp_client[whicharm].get_result()
        success = (result.grasp_result == "Object grasped")

        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            resetup_goal = OverheadGraspSetupGoal()
            resetup_goal.disable_head = True
            self.grasp_setup_client[whicharm].send_goal(resetup_goal)
            self.grasp_setup_client[whicharm].wait_for_result()
        else:
            rospy.loginfo("pick-up failed.")

        return success


    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, place_rect_dims, place_rect_center):

        rospy.loginfo("putting down the object in the r gripper")

        ############################################################
        # Creating place goal
        grasp_goal = OverheadGraspGoal()
        grasp_goal.is_grasp = False
        grasp_goal.disable_head = True
        grasp_goal.disable_coll = False
        grasp_goal.grasp_type=OverheadGraspGoal.MANUAL_GRASP
        grasp_goal.grasp_params = [0] * 3
        grasp_goal.grasp_params[0] = place_rect_center.pose.position.x
        grasp_goal.grasp_params[1] = place_rect_center.pose.position.y
        grasp_goal.behavior_name = "overhead_grasp"
        grasp_goal.sig_level = 0.999
        ############################################################

        self.grasp_client[whicharm].send_goal(grasp_goal)
        self.grasp_client[whicharm].wait_for_result()
        result = self.grasp_client[whicharm].get_result()
        success = (result.grasp_result == "Object placed")

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
    date = datetime.datetime.now()

    f_name = date.strftime("%Y-%m-%d_%H-%M-%S")

    #save_dir = os.getcwd()+'/../../data/'+f_name
    save_dir = '/removable/mkillpack/'+f_name
    #playpen_dir = '/home/mkillpack/svn/gt-ros-pkg/hrl/pr2_playpen/data/' #should add way to sync
    os.mkdir(save_dir)

#print "CHECING FOR DIRECTORY :  ", os.getcwd()+'/../../data/'+f_name
    #.5 m in front of robot, centered
    target_point_xyz = [.625, 0, table_height]   #this is currently an approximation/hack should use ar tag
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    arm = 0
    rospy.wait_for_service('playpen_train')
    rospy.wait_for_service('playpen_check_success')
    if SAVE == True:
        # rospy.wait_for_service('playpen_save_pt_cloud')
        # rospy.wait_for_service('playpen_save_image')
        rospy.wait_for_service('pr2_save_pt_cloud')
        rospy.wait_for_service('pr2_save_image')

    try:
        train = rospy.ServiceProxy('playpen_train', Train)
        check_success = rospy.ServiceProxy('playpen_check_success', Check)
        if SAVE == True:
            save_pr2_cloud = rospy.ServiceProxy('pr2_save_pt_cloud', Save)
            save_pr2_image = rospy.ServiceProxy('pr2_save_image', Save)
            # save_playpen_cloud = rospy.ServiceProxy('playpen_save_pt_cloud', Save)
            # save_playpen_image = rospy.ServiceProxy('playpen_save_image', Save)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    sppe.move_to_side(0, False)
    sppe.move_to_side(1, False)

    for i in xrange(len(sppe.objects_dist)):
        try:
            file_handle = open(save_dir+'/object'+str(i).zfill(3)+'.pkl', 'w')
            data = {}
            sppe.playpen(0)
            rospy.sleep(10)
            num_samples = train('table')
            sppe.conveyor(sppe.objects_dist[i])
            # data['object'+str(i).zfill(3)] = {}
            # data['object'+str(i).zfill(3)]['success'] = []
            # data['object'+str(i).zfill(3)]['frames'] = []

            data['success'] = []
            data['frames'] = []

            start_time = rospy.get_time()
            # while sppe.tries<3:
            while rospy.get_time()-start_time < 2880.0:
                print "arm is ", arm
                sppe.move_to_side(arm, True)
                rospy.sleep(2)
                if SAVE == True:
                    save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_before_pr2.pcd')
                    save_pr2_image(save_dir+'/object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_before_pr2.png')
                # save_playpen_cloud(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_before_playpen.pcd')
                # save_playpen_image(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_before_playpen.png')
                num_samples = train('object')
                success = sppe.pick_up_object_near_point(target_point, arm)
                sppe.move_to_side(arm, False)

                result = []
                rospy.sleep(7)
                for j in xrange(5):
                   result.append(check_success('').result)
                result.sort()
                if result[2] == 'table':
                   success = True
                   data['success'].append(success)
                elif result[2] == 'object':
                   success = False
                   data['success'].append(success)
                #else:
                   #success = False
                   #sppe.tries = sppe.tries-1 # this is to compensate for failures in perception hopefully

                print "SUCCESS IS :", success
                data['success'].append(success)

                if SAVE == True:
                    save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_after_pr2.pcd')
                    save_pr2_image(save_dir+'/object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_after_pr2.png')
                # save_playpen_cloud(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_after_playpen.pcd')
                # save_playpen_image(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(sppe.tries).zfill(3)+'_after_playpen.png')

                if success:
                    sppe.successes=sppe.successes + 1
                    #square of size 30 cm by 30 cm
                    place_rect_dims = [.1, .1]

                    x = np.random.uniform(-0.18, 0.18, 1)[0]
                    y = np.random.uniform(-0.18, 0.18, 1)[0]
                    center_xyz = [.625+x, y, table_height+.10]


                    #aligned with axes of frame_id
                    center_quat = [0,0,0,1]
                    place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                            'base_link')

                    sppe.place_object(arm, place_rect_dims, place_rect_center)

                sppe.move_to_side(arm, True)
                arm = arm.__xor__(1)
                sppe.tries = sppe.tries+1

            sppe.playpen(1)
            sppe.successes = 0
            sppe.tries = 0
            cPickle.dump(data, file_handle)
            file_handle.close()
            sppe.playpen(0)

        except:
            print "failed for object"
            
