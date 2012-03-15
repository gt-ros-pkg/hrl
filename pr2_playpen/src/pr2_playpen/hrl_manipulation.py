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
import hrl_pr2_lib.pressure_listener as pl
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

class HRLControllerPlaypen():

    def __init__(self):
        print "waiting for conveyor"
        rospy.wait_for_service('playpen')
        rospy.wait_for_service('conveyor')
        print "started conveyor and playpen"
        self.playpen = rospy.ServiceProxy('playpen', Playpen)
        self.conveyor = rospy.ServiceProxy('conveyor', Conveyor)
        self.objects_dist = [0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13,
                             0.13, 0.13, 0.13]

        self.tries = 0
        self.successes = 0

        self.r_grip_press = pl.PressureListener(topic='/pressure/r_gripper_motor')
        self.l_grip_press = pl.PressureListener(topic='/pressure/l_gripper_motor')

        self.grasp_client = [None, None]
        self.grasp_setup_client = [None, None]

        self.grasp_client[0] = actionlib.SimpleActionClient('r_overhead_grasp', OverheadGraspAction)
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
    import optparse
    p = optparse.OptionParser()

    p.add_option('--path', action='store', dest='path_save',type='string',
                 default=None, help='this is path to directory for saving files')

    opt, args = p.parse_args()

    rospy.init_node('simple_pick_and_place_example')
    hcp = HRLControllerPlaypen()

    #adjust for your table 
    table_height = 0.529
    date = datetime.datetime.now()

    f_name = date.strftime("%Y-%m-%d_%H-%M-%S")

    if opt.path_save == None:
        print "Not logging or saving data from playpen"
        SAVE = False
    else:
        save_dir = opt.path_save+f_name
        print "Logging and saving playpen data in :", save_dir
        SAVE = True
    

    if SAVE == True:
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

    print "moving arms to side"
    hcp.move_to_side(0, False)
    hcp.move_to_side(1, False)
    print "done moving arms, sleeping ..."
    
    rospy.sleep(15)
    print "done sleeping, now training for table top"

    num_samples = train('table')
    for i in xrange(len(hcp.objects_dist)):
        try:
            if SAVE==True:
                file_handle = open(save_dir+'/object'+str(i).zfill(3)+'.pkl', 'w')
            data = {}
            hcp.playpen(0)
            hcp.conveyor(hcp.objects_dist[i])

            data['success'] = []
            data['frames'] = []
            data['pressure'] = {}
            data['pressure']['which_arm'] = []
            data['pressure']['data'] = []

            start_time = rospy.get_time()
            # while hcp.tries<3:
            while rospy.get_time()-start_time < 100.0:
                print "arm is ", arm
                hcp.move_to_side(arm, True)
                rospy.sleep(7)
                if SAVE == True:
                    save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_before_pr2.pcd')
                    save_pr2_image(save_dir+'/object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_before_pr2.png')
                # save_playpen_cloud(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_before_playpen.pcd')
                # save_playpen_image(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_before_playpen.png')
                num_samples = train('object')
                print "attempting to pick up the object"
                success = hcp.pick_up_object_near_point(target_point, arm)
                print "starting to move arm to side at :", rospy.get_time()
                hcp.move_to_side(arm, False)
                print "moved past move to side arm command at :", rospy.get_time()
                results = []
                
                print "sleeping for 10 seconds, is this necessary ..."
                rospy.sleep(10)
                num_samples = 7
                for j in xrange(num_samples):
                   results.append(check_success('').result)
                results.sort()
                print "results are :", results
                print "index into result is :", int(num_samples/2)
                if results[int(num_samples/2)] == 'table':
                   success = True
                   data['success'].append(success)
                elif results[int(num_samples/2)] == 'object':
                   success = False
                   data['success'].append(success)
                else:
                   success = None
                   #hcp.tries = hcp.tries-1 # this is to compensate for failures in perception hopefully

                print "SUCCESS IS :", success

                if SAVE == True:
                    save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_after_pr2.pcd')
                    save_pr2_image(save_dir+'/object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_after_pr2.png')
                # save_playpen_cloud(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_after_playpen.pcd')
                # save_playpen_image(playpen_dir+'object'+str(i).zfill(3)+'_try'+str(hcp.tries).zfill(3)+'_after_playpen.png')

                if success:
                    hcp.successes=hcp.successes + 1
                    #square of size 30 cm by 30 cm
                    place_rect_dims = [.1, .1]
                    
                    inside = False
                    while inside == False:
                        x = np.random.uniform(-0.18, 0.18, 1)[0]
                        y = np.random.uniform(-0.18, 0.18, 1)[0]
                        if math.sqrt(x*x+y*y) <= 0.18:
                            inside = True
                
                    center_xyz = [.625+x, y, table_height+.10]


                    #aligned with axes of frame_id
                    center_quat = [0,0,0,1]
                    place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                            'base_link')

                    hcp.place_object(arm, place_rect_dims, place_rect_center)

                hcp.move_to_side(arm, True)
                arm = arm.__xor__(1)
                hcp.tries = hcp.tries+1

            hcp.playpen(1)
            hcp.successes = 0
            hcp.tries = 0
            cPickle.dump(data, file_handle)
            file_handle.close()
            hcp.playpen(0)

        except:
            print "failed for object"
            
