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
from pr2_playpen.pick_and_place_manager import *
from object_manipulator.convert_functions import *
from pr2_playpen.srv import Playpen
from pr2_playpen.srv import Conveyor
from pr2_playpen.srv import Check
from pr2_playpen.srv import Train
from pr2_playpen.srv import Save
import os
import datetime

import math
import numpy as np

class SimplePickAndPlaceExample():

    def __init__(self):

        rospy.loginfo("initializing pick and place manager")
        self.papm = PickAndPlaceManager()
        rospy.loginfo("finished initializing pick and place manager")
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
        self.papm.move_arm_out_of_way(0)
        self.papm.move_arm_out_of_way(1)

#############once is it positioned, we don't want to move the head at all !!!#############
#        rospy.loginfo("pointing the head at the target point")
#        self.papm.point_head(get_xyz(target_point.point),
#                             target_point.header.frame_id)
#########################################################################################        


        rospy.loginfo("detecting the table and objects")
        self.papm.call_tabletop_detection(take_static_collision_map = 1,
                             update_table = 1, clear_attached_objects = 1)

        rospy.loginfo("picking up the nearest object to the target point")
        success = self.papm.pick_up_object_near_point(target_point,
                                                      whicharm)

        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            #self.papm.move_arm_to_side(whicharm)
            self.papm.move_arm_out_of_way(whicharm)
        else:
            rospy.loginfo("pick-up failed.")

        return success


    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, place_rect_dims, place_rect_center):

        self.papm.set_place_area(place_rect_center, place_rect_dims)

        rospy.loginfo("putting down the object in the %s gripper"\
                      %self.papm.arm_dict[whicharm])
        success = self.papm.put_down_object(whicharm,
                      max_place_tries = 5,
                      use_place_override = 1)

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

#    f_name = str(date.year)+str(date.month).zfill(2)+str(date.day).zfill(2)+'_'+str(date.hour).zfill(2)+str(date.minute).zfill(2)+str(date.second).zfill(2)
    f_name = date.strftime("%Y-%m-%d_%H-%M-%S")

    save_dir = os.getcwd()+'../data/'+f_name
    os.mkdir(save_dir)

    print "CHECING FOR DIRECTORY :  ", os.getcwd()+'../data/'+f_name
    #.5 m in front of robot, centered
    target_point_xyz = [.625, 0, table_height]   #this is currently an approximation/hack should use ar tag
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    arm = 0
    rospy.wait_for_service('playpen_train_success')
    rospy.wait_for_service('playpen_check_success')
    rospy.wait_for_service('playpen_save_pt_cloud')
    rospy.wait_for_service('pr2_save_pt_cloud')

    try:
        train_success = rospy.ServiceProxy('playpen_train_success', Train)
        check_success = rospy.ServiceProxy('playpen_check_success', Check)
        save_pr2_cloud = rospy.ServiceProxy('pr2_save_pt_cloud', Save)
        save_playpen_cloud = rospy.ServiceProxy('playpen_save_pt_cloud', Save)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    num_samples = train_success()

    data = {}

    for i in xrange(len(sppe.objects_dist)):
        sppe.playpen(0)
        sppe.conveyor(sppe.objects_dist[i])
        data['object'+str(i).zfill(3)] = {}
        data['object'+str(i).zfill(3)]['success'] = []
        data['object'+str(i).zfill(3)]['frames'] = []
        while sppe.tries<6:
            print "arm is ", arm
#            print "target _point is ", target_point.x
            save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_before_pr2')
            save_playpen_cloud(save_dir+'/object'+str(i).zfill(3)+'_before_playpen')
            success = sppe.pick_up_object_near_point(target_point, arm)   #right arm

            result = []
            for i in xrange(5):
                result.append(check_success(''))
                rospy.sleep(4)
            
            if result[2] = 'table':
                success = True
            elif result[2] = 'object':
                success = False
            else:
                success = False
                sppe.tries = sppe.tries-1 # this is to compensate for failures in perception hopefully
            data['object'+str(i).zfill(3)]['success'].append(success)
                
            if success:
                sppe.successes=sppe.successes + 1
                #square of size 30 cm by 30 cm
                place_rect_dims = [.1, .1]

                x = np.random.uniform(-0.2, 0.20, 1)[0]
                y = np.random.uniform(-0.2, 0.20, 1)[0]
                center_xyz = [.625+x, y, table_height+.10]


                #aligned with axes of frame_id
                center_quat = [0,0,0,1]
                place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                        'base_link')

                sppe.place_object(arm, place_rect_dims, place_rect_center)

            save_pr2_cloud(save_dir+'/object'+str(i).zfill(3)+'_after_pr2')
            save_playpen_cloud(save_dir+'/object'+str(i).zfill(3)+'_after_playpen')

            arm = arm.__xor__(1)
            sppe.tries = sppe.tries+1

        sppe.playpen(1)
        sppe.successes = 0
        sppe.tries = 0

    
#    sppe.playpen(0)
