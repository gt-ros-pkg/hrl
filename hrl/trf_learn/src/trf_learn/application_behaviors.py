#
# Copyright (c) 2010, Georgia Tech Research Corporation
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

import roslib; roslib.load_manifest('trf_learn')
import rospy
import functools as ft
import numpy as np
import math
import time
import cv
import tf
import pdb

import tf.transformations as tr
import hrl_camera.ros_camera as rc
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_opencv.image3d as i3d
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.linear_move as lm
import hrl_pr2_lib.linear_move as lm
import dynamic_reconfigure.client as dr
import hrl_pr2_lib.devices as hd

def image_diff_val2(before_frame, after_frame):
    br = np.asarray(before_frame)
    ar = np.asarray(after_frame)
    max_sum = br.shape[0] * br.shape[1] * br.shape[2] * 255.
    sdiff = np.abs((np.sum(br) / max_sum) - (np.sum(ar) / max_sum))
    #sdiff = np.sum(np.abs(ar - br)) / max_sum
    return sdiff

class ManipulationBehaviors:

    def __init__(self, arm, pr2_obj, tf_listener=None):
        try:
            rospy.init_node('linear_move', anonymous=True)
        except Exception, e:
            rospy.loginfo('call to init_node failed %s' % str(e))
        self.movement = lm.LinearReactiveMovement(arm, pr2_obj, tf_listener)

    def reach(self, point, pressure_thres,\
            reach_direction=np.matrix([0,0,0]).T, orientation=None):
        MOVEMENT_TOLERANCE = .1
        #REACH_TOLERANCE = .1

        #self.movement.set_movement_mode_cart()
        #pdb.set_trace()
        self.movement.set_pressure_threshold(pressure_thres)
        loc_bl = self.movement.arm_obj.pose_cartesian_tf()[0]
        front_loc = point.copy()
        front_loc[0,0] = max(loc_bl[0,0], .4)
        #front_loc[0,0] = loc_bl[0,0]
        #pdb.set_trace()

        if orientation == None:
            start_loc = self.movement.arm_obj.pose_cartesian_tf()
            orientation = start_loc[1]
        self.movement.pressure_listener.rezero()
        #pdb.set_trace()
        #for i in range(2):
        r1, residual_error = self.movement.move_absolute((front_loc, orientation), stop='pressure', pressure=pressure_thres)

        #if residual_error > MOVEMENT_TOLERANCE or r1 != None: #if this step fails, we move back then return
        #    #self.move_absolute(start_loc, stop='accel')
        #    pdb.set_trace()
        #    return False, r1, None

        #We expect impact here
        pos_error = None
        try:
            #pdb.set_trace()
            #loc_bl = self.movement.current_location()[0]
            #reach_direction = loc_bl - point 
            #reach_direction = reach_direction / np.linalg.norm(reach_direction)
            point_reach = point + reach_direction
            r2, pos_error = self.movement.move_absolute((point_reach, \
                    self.movement.arm_obj.pose_cartesian_tf()[1]), stop='pressure_accel', pressure=pressure_thres)

        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
            r2 = None

        touch_loc_bl = self.movement.arm_obj.pose_cartesian_tf()
        #if r2 == None or r2 == 'pressure' or r2 == 'accel' or (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction))):
        if r2 == 'pressure' or r2 == 'accel' or (pos_error != None and (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction)))):
            self.movement.pressure_listener.rezero()

            # b/c of stiction & low gains, we can't move precisely for small
            # distances, so move back then move forward again
            #reach_dir = reach_direction / np.linalg.norm(reach_direction)
            cur_pose, cur_ang = self.movement.arm_obj.pose_cartesian_tf()

            #p1 = cur_pose - (reach_dir * .05)
            #p2 = cur_pose + move_back_distance
            #rospy.loginfo('moving back')
            #_, pos_error = self.movement.move_absolute((p1, cur_ang), stop='None', pressure=pressure_thres)
            #self.movement.pressure_listener.rezero()

            #rospy.loginfo('moving forward again')
            #_, pos_error = self.movement.move_absolute((p2, cur_ang), stop='None', pressure=pressure_thres)

            #self.movement.move_relative_gripper(4.*move_back_distance, stop='none', pressure=pressure_thres)
            #self.movement.move_relative_gripper(-3.*move_back_distance, stop='none', pressure=pressure_thres)
            #self.movement.pressure_listener.rezero()
            return True, r2, touch_loc_bl
        else:
            #pdb.set_trace()
            #shouldn't get here
            return False, r2, None

    def press(self, direction, press_pressure, contact_pressure):
        #make contact first
        self.movement.set_movement_mode_cart()
        #pdb.set_trace()
        r1, diff_1 = self.movement.move_relative_gripper(direction, stop='pressure', pressure=contact_pressure)
        #now perform press
        if r1 == 'pressure' or r1 == 'accel':
            self.movement.set_movement_mode_cart()
            r2, diff_2 = self.movement.move_relative_gripper(direction, stop='pressure_accel', pressure=press_pressure)
            if r2 == 'pressure' or r2 == 'accel' or r2 == None:
                return True, r2
            else:
                return False, r2
        else:
            return False, r1

class ApplicationBehaviorsDB:

    def __init__(self, optical_frame, tf_listener=None):
        if tf_listener == None:
            tf_listener = tf.TransformListener()
        self.tf_listener = tf_listener
        self.optical_frame = optical_frame

        self.robot = pr2.PR2(self.tf_listener, base=True)
        self.behaviors = ManipulationBehaviors('l', self.robot, tf_listener=self.tf_listener)
        self.laser_scan = hd.LaserScanner('point_cloud_srv')
        self.prosilica = rc.Prosilica('prosilica', 'polled')

        self.wide_angle_camera_left = rc.ROSCamera('/wide_stereo/left/image_rect_color')
        self.wide_angle_configure = dr.Client('wide_stereo_both')


        #TODO: define start location in frame attached to torso instead of base_link
        self.start_location_light_switch = (np.matrix([0.35, 0.30, 1.1]).T, np.matrix([0., 0., 0., 0.1]))
        self.start_location_drawer       = (np.matrix([0.20, 0.40, .8]).T,  
                                            np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0)))
        self.folded_pose = np.matrix([ 0.10134791, -0.29295995,  0.41193769]).T
        self.driving_param = {'light_switch_up':   {'coarse': .9, 'fine': .6, 'voi': .4},
                              'light_switch_down': {'coarse': .9, 'fine': .6, 'voi': .4},

                              'light_rocker_down': {'coarse': .9, 'fine': .6, 'voi': .4},
                              'light_rocker_up':   {'coarse': .9, 'fine': .6, 'voi': .4},

                              'pull_drawer':       {'coarse': .9, 'fine': .5, 'voi': .4},
                              'push_drawer':       {'coarse': .9, 'fine': .5, 'voi': .4}}

        self.create_arm_poses()
        self.driving_posture('light_switch_down')
        self.robot.projector.set(False)

    #######################################################################################
    #Behavior Indexing Functions
    #######################################################################################
    def get_behavior_by_task(self, task_type):
        if task_type == 'light_switch_down':
            return ft.partial(self.light_switch, 
                        #point_offset=np.matrix([0,0,.03]).T,
                        point_offset=np.matrix([0,0, -.08]).T,
                        press_contact_pressure=300,
                        #move_back_distance=np.matrix([-.0075,0,0]).T,
                        press_pressure=6000,
                        press_distance=np.matrix([0.01,0,-.15]).T,
                        visual_change_thres=.023)

        elif task_type == 'light_switch_up':
            return ft.partial(self.light_switch, 
                        #point_offset=np.matrix([0,0,-.08]).T,
                        point_offset=np.matrix([0,0,.08]).T,
                        press_contact_pressure=300,
                        #move_back_distance=np.matrix([-.0075,0,0]).T,
                        press_pressure=6000,
                        press_distance=np.matrix([0.01,0,.15]).T,
                        visual_change_thres=.023)

        elif task_type == 'light_rocker_up':
            return ft.partial(self.light_rocker_push,
                        pressure=500,
                        visual_change_thres=.025, offset=np.matrix([0,0,-.05]).T)

        elif task_type == 'light_rocker_down':
            return ft.partial(self.light_rocker_push,
                        pressure=500,
                        visual_change_thres=.025, offset=np.matrix([0,0,.05]).T)

        elif task_type == 'pull_drawer':
            return self.drawer

        elif task_type == 'push_drawer':
            return self.drawer_push

        else:
            pdb.set_trace()

    def manipulation_posture(self, task_type):
        self.robot.projector.set(False)
        for i in range(3):
            self.prosilica.get_frame()
        self.robot.projector.set(True)
        #rospy.sleep(1)

        self.robot.left_gripper.open(False, .005)
        #self.robot.right_gripper.open(True, .005)
        self.behaviors.movement.pressure_listener.rezero()

        if task_type == 'light_switch_down' or task_type == 'light_switch_up':
            if np.linalg.norm(self.start_location_light_switch[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(.2, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()

        elif task_type == 'light_rocker_up' or task_type == 'light_rocker_down':
            if np.linalg.norm(self.start_location_light_switch[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(.2, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()

        elif task_type == 'pull_drawer' or task_type == 'push_drawer':
            if np.linalg.norm(self.start_location_drawer[0] - self.robot.left.pose_cartesian_tf()[0]) < .3:
                return
            self.robot.torso.set_pose(0.01, True)
            self.untuck()
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='pressure')
            self.behaviors.movement.pressure_listener.rezero()
        else:
            pdb.set_trace()

    def driving_posture(self, task_type):
        self.robot.projector.set(False)
        self.close_gripper()

        if np.linalg.norm(self.folded_pose - self.robot.left.pose_cartesian_tf()[0]) < .1:
            return
        #TODO: specialize this
        self.robot.torso.set_pose(0.03, True)
        self.robot.left_gripper.open(False, .005)
        self.robot.right_gripper.open(True, .005)
        self.behaviors.movement.pressure_listener.rezero()

        if task_type == 'light_switch_down' or task_type == 'light_switch_up':
            self.tuck()

        elif task_type == 'light_rocker_up' or task_type == 'light_rocker_down':
            self.tuck()

        elif task_type == 'pull_drawer' or task_type == 'push_drawer':
            self.tuck()
        else:
            pdb.set_trace()

    #######################################################################################
    #Scripty Behaviors
    #######################################################################################
    def create_arm_poses(self):
        self.right_tucked = np.matrix([[-0.02362532,  1.10477102, -1.55669475, \
                -2.12282706, -1.41751231, -1.84175899,  0.21436806]]).T

        self.left_tucked = np.matrix([[ 0.05971848,  1.24980184,  1.79045674, \
                -1.68333801, -1.73430635, -0.09838841, -0.08641928]]).T

        #lift the right arm up a little bit
        self.r0 = np.matrix([[-0.22774141,  0.7735819 , -1.45102092, \
                -2.12152412, -1.14684579, -1.84850287,  0.21397648]]).T

        #left arm rotates
        self.l0 = np.matrix([[ 0.06021592,  1.24844832,  1.78901355, -1.68333801, 1.2, -0.10152105, -0.08641928]]).T

        #left arm moves out
        self.l1 = np.matrix([[0.94524406,  1.24726399,  1.78548574, -1.79148173,  1.20027637, -1.0, -0.08633226]]).T

        #left arm rotates outward a little more
        self.l2 = np.matrix([[ 1.53180837,  1.24362641,  1.78452361, -1.78829678,  1.1996979,-1.00446167, -0.08741998]]).T

    def untuck(self):
        if np.linalg.norm(self.robot.left.pose() - self.left_tucked) < .3:
            rospy.loginfo('untuck: not in tucked position.  Ignoring request')
            return
        #assume we are tucked
        self.behaviors.movement.set_movement_mode_ik()
        self.robot.right.set_pose(self.r0, 1.)
        self.robot.left.set_poses(np.column_stack([self.l0, self.l1, self.l2]), \
                                  np.array([1., 2., 3.]))
        self.robot.right.set_pose(self.right_tucked, 1.)
        self.behaviors.movement.set_movement_mode_cart()

    def tuck(self):
        if np.linalg.norm(self.robot.left.pose() - self.left_tucked) < .5:
            rospy.loginfo('tuck: Already tucked. Ignoring request.')
            return
        #lift the right arm up a little bit
        self.behaviors.movement.set_movement_mode_ik()
        self.robot.right.set_pose(self.r0, 1.)
        self.robot.left.set_poses(np.column_stack([self.l2, self.l1, self.l0, self.left_tucked]), \
                                  np.array([4., 5., 6., 7.]))
        self.robot.right.set_pose(self.right_tucked, 1.)
        self.behaviors.movement.set_movement_mode_cart()

    def close_gripper(self):
        GRIPPER_CLOSE = .003
        #self.robot.left_gripper.open(True, position=GRIPPER_CLOSE)
        self.behaviors.movement.gripper_close()

    def open_gripper(self):
        GRIPPER_OPEN = .08
        #self.robot.left_gripper.open(True, position=GRIPPER_OPEN)
        self.behaviors.movement.gripper_open()

    def look_at(self, point_bl, block=True):
        #self.robot.head.look_at(point_bl-np.matrix([0,0,.15]).T, pointing_frame=self.optical_frame, 
        #                        pointing_axis=np.matrix([1,0,0.]).T, wait=block)
        print 'LOOKING AT POINT', point_bl.T, self.optical_frame
        #self.robot.head.look_at(point_bl, pointing_frame=self.optical_frame, pointing_axis=np.matrix([1, 0, 0.]).T, wait=block)
        self.robot.head.look_at(point_bl, wait=block)

    #######################################################################################
    #Mobility Behaviors
    #######################################################################################
    ##
    # Drive using within a dist_far distance of point_bl
    def drive_approach_behavior(self, point_bl, dist_far):
    # navigate close to point
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        t_current_map, r_current_map = self.robot.base.get_pose()
        rospy.loginfo('drive_approach_behavior: point is %.3f m away"' % np.linalg.norm(t_current_map[0:2].T - point_map[0:2,0].T))

        point_dist = np.linalg.norm(point_bl)
        bounded_dist = np.max(point_dist - dist_far, 0)
        point_close_bl = (point_bl / point_dist) * bounded_dist
        point_close_map = tfu.transform_points(map_T_base_link, point_close_bl)
        rvalue = self.robot.base.set_pose(point_close_map.T.A1.tolist(), \
                                          r_current_map, '/map', block=True)
        t_end, r_end = self.robot.base.get_pose()
        rospy.loginfo('drive_approach_behavior: ended up %.3f m away from laser point' % np.linalg.norm(t_end[0:2] - point_map[0:2,0].T))
        rospy.loginfo('drive_approach_behavior: ended up %.3f m away from goal' % np.linalg.norm(t_end[0:2] - point_close_map[0:2,0].T))
        rospy.loginfo('drive_approach_behavior: returned %d' % rvalue)
        return rvalue

    ##
    # Drive so that we are perpendicular to a wall at point_bl (radii voi_radius) 
    # stop at dist_approach
    def approach_perpendicular_to_surface(self, point_bl, voi_radius, dist_approach):
        map_T_base_link0 = tfu.transform('map', 'base_link', self.tf_listener)
        point_map0 = tfu.transform_points(map_T_base_link0, point_bl)
        #pdb.set_trace()
        self.turn_to_point(point_bl, block=False)

        point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), \
                                        point_map0)
        point_cloud_bl = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 2.5)
        point_cloud_np_bl = ru.pointcloud_to_np(point_cloud_bl)
        rospy.loginfo('approach_perpendicular_to_surface: pointcloud size %d' \
                % point_cloud_np_bl.shape[1])
        voi_points_bl, limits_bl = i3d.select_rect(point_bl, voi_radius, voi_radius, voi_radius, point_cloud_np_bl)
        #TODO: use closest plane instead of closest points determined with KDTree
        normal_bl = i3d.calc_normal(voi_points_bl)
        point_in_front_mechanism_bl = point_bl + normal_bl * dist_approach
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_in_front_mechanism_map = tfu.transform_points(map_T_base_link, point_in_front_mechanism_bl)

        #Navigate to point (TODO: check for collisions)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        t_current_map, r_current_map = self.robot.base.get_pose()
        rospy.loginfo('approach_perpendicular_to_surface: driving for %.3f m to front of surface' \
                % np.linalg.norm(t_current_map[0:2] - point_in_front_mechanism_map[0:2,0].T))
        #pdb.set_trace()
        rvalue = self.robot.base.set_pose(point_in_front_mechanism_map.T.A1.tolist(), r_current_map, 'map')
        if rvalue != 3:
            return rvalue

        t1_current_map, r1_current_map = self.robot.base.get_pose()
        rospy.loginfo('approach_perpendicular_to_surface: %.3f m away from from of surface' % np.linalg.norm(t1_current_map[0:2] - point_in_front_mechanism_map[0:2,0].T))

        #Rotate to face point (TODO: check for collisions)
        base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        point_bl = tfu.transform_points(base_link_T_map, point_map)
        #pdb.set_trace()
        self.turn_to_point(point_bl, block=False)
        time.sleep(2.)

        return rvalue

    def approach_location(self, point_bl, coarse_stop, fine_stop, voi_radius=.2):
        point_dist = np.linalg.norm(point_bl[0:2,0])
        rospy.loginfo('approach_location: Point is %.3f away.' % point_dist)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)

        dist_theshold = coarse_stop + .1
        if point_dist > dist_theshold:
            rospy.loginfo('approach_location: Point is greater than %.1f m away (%.3f).  Driving closer.' % (dist_theshold, point_dist))
            rospy.loginfo('approach_location: point_bl ' + str(point_bl.T))

            ret = self.drive_approach_behavior(point_bl, dist_far=coarse_stop)
            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
            if ret != 3:
                dist_end = np.linalg.norm(point_bl_t1[0:2,0])
                if dist_end > dist_theshold:
                    rospy.logerr('approach_location: drive_approach_behavior failed! %.3f' % dist_end)
                    self.robot.sound.say("I am unable to navigate to that location")
                    return False, 'failed'

            ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=voi_radius, dist_approach=fine_stop)
            if ret != 3:
                rospy.logerr('approach_location: approach_perpendicular_to_surface failed!')
                return False, 'failed'

            self.robot.sound.say('done')
            rospy.loginfo('approach_location: DONE DRIVING!')
            return True, 'done'
        else:
            return False, 'ignored'

    def turn_to_point(self, point_bl, block=True):
        ang = math.atan2(point_bl[1,0], point_bl[0,0])
        rospy.loginfo('turn_to_point: turning by %.2f deg' % math.degrees(ang))
        #pdb.set_trace()
        self.robot.base.turn_by(-ang, block=block, overturn=True)

    def location_approach_driving(self, task, point_bl):
        #Get closer if point is far away
        ap_result = self.approach_location(point_bl, 
                        coarse_stop=self.driving_param[task]['coarse'], 
                        fine_stop=self.driving_param[task]['fine'], 
                        voi_radius=self.driving_param[task]['voi'])

        if ap_result[1] == 'failed':
            return False, 'approach_location failed'

        if ap_result[1] == 'ignore':
            #reorient with planner
            ret = self.approach_perpendicular_to_surface(point_bl, 
                    voi_radius=self.driving_param[task]['voi'], 
                    dist_approach=self.driving_param[task]['fine'])
            if ret != 3:
                rospy.logerr('location_approach_driving: approach_perpendicular_to_surface failed!')
                return False, 'approach_perpendicular_to_surface failed'
            else:
                return True, None

        return True, None

    def move_base_planner(self, trans, rot):
        #pdb.set_trace()
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        #Do this to clear out any hallucinated obstacles
        self.turn_to_point(p_bl)
        rvalue = self.robot.base.set_pose(trans, rot, '/map', block=True)
        p_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), np.matrix(trans).T)
        #pdb.set_trace()
        self.robot.base.move_to(p_bl[0:2,0], True)
        t_end, r_end = self.robot.base.get_pose()
        return rvalue==3, np.linalg.norm(t_end[0:2] - np.array(trans)[0:2])

    #######################################################################################
    #Application Specific Behaviors
    #######################################################################################

    def camera_change_detect(self, threshold, f, args):
        config = self.wide_angle_configure.get_configuration()
        config['auto_gain'] = False
        config['auto_exposure'] = False
        self.wide_angle_configure.update_configuration(config)

        #take before sensor snapshot
        start_pose = self.robot.head.pose()
        #pdb.set_trace()
        #self.robot.head.set_pose(np.radians(np.matrix([1.04, -20]).T), 1)
        self.robot.head.set_pose(np.radians(np.matrix([30., -20]).T), 1)
        time.sleep(4)
        for i in range(7):
            before_frame = self.wide_angle_camera_left.get_frame()
        cv.SaveImage('before.png', before_frame)
        f_return = f(*args)
        time.sleep(5)
        for i in range(7):
            after_frame = self.wide_angle_camera_left.get_frame()

        cv.SaveImage('after.png', after_frame)
        sdiff = image_diff_val2(before_frame, after_frame)
        self.robot.head.set_pose(start_pose, 1)
        self.robot.head.set_pose(start_pose, 1)
        time.sleep(3)        
        #take after snapshot
        #threshold = .03
        config['auto_gain'] = True
        config['auto_exposure'] = True
        self.wide_angle_configure.update_configuration(config)

        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, threshold))
        if sdiff > threshold:
            rospy.loginfo('difference detected!')
            return True, f_return
        else:
            rospy.loginfo('NO differences detected!')
            return False, f_return

    def light_switch(self, point, 
            point_offset, press_contact_pressure, 
            press_pressure, press_distance, visual_change_thres):

        try:
            #pdb.set_trace()
            #print '===================================================================='
            #point = point + point_offset 
            rospy.loginfo('reaching to ' + str(point.T))
            #pdb.set_trace()
            #self.behaviors.movement.gripper_close()
            self.close_gripper()
            #self.robot.left_gripper.open(True, position=.005)
            time.sleep(1)
            self.behaviors.movement.pressure_listener.rezero()
            #TODO: have go_home check whether it is actually at that location
            #self.behaviors.move_absolute(self.start_location, stop='pressure_accel')

            #start_loc = self.current_location()
            #pdb.set_trace()
            success, reason, touchloc_bl = self.behaviors.reach(point, \
                    press_contact_pressure, \
                    reach_direction=np.matrix([0.1,0,0]).T)
            #r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.01, 0., 0.]).T, \
            #        stop='none', pressure=press_contact_pressure)

            if touchloc_bl != None:
                dist = np.linalg.norm(point - touchloc_bl[0])
                #print '===================================================================='
                #print '===================================================================='
                #TODO assure that reaching motion did touch the point that we intended to touch.
                rospy.loginfo('Touched point is %.3f m away from observed point' % dist)
                #print '===================================================================='
                #print '===================================================================='

            if not success:
                error_msg = 'Reach failed due to "%s"' % reason
                rospy.loginfo(error_msg)
                rospy.loginfo('Failure recovery: moving back')
                self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='accel', \
                        pressure=press_contact_pressure)
                #raise TaskError(error_msg)
                return False, None, point+point_offset

            rospy.loginfo('pressing')

            #Should not be making contact
            self.behaviors.movement.pressure_listener.rezero()
            change, press_ret = self.camera_change_detect(visual_change_thres, \
                    self.behaviors.press, \
                    (press_distance, press_pressure, press_contact_pressure))
            success, reason = press_ret
            if not success:
                rospy.loginfo('Press failed due to "%s"' % reason)

            #code reward function
            #monitor self collision => collisions with the environment are not self collisions
            rospy.loginfo('moving back')
            #self.behaviors.movement.set_movement_mode_cart()
            r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.03, 0., 0.]).T, \
                    stop='none', pressure=press_contact_pressure)
            if r1 != None:
                rospy.loginfo('moving back failed due to "%s"' % r1)
                return change, None, point+point_offset

            rospy.loginfo('reseting')
            self.behaviors.movement.pressure_listener.rezero()
            r2, pos_error2 = self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            if r2 != None and r2 != 'no solution':
                rospy.loginfo('moving back to start location failed due to "%s"' % r2)
                return change, None, point+point_offset
            self.behaviors.movement.pressure_listener.rezero()

            rospy.loginfo('DONE.')
            return change, touchloc_bl, point+point_offset

        except lm.RobotSafetyError, e:
            rospy.loginfo('>>>> ROBOT SAFETY ERROR! RESETTING. %s' % str(e))
            self.behaviors.movement.pressure_listener.rezero()
            r2, pos_error2 = self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure')
            return change, None, point+point_offset

    def light_rocker_push(self, point, pressure, visual_change_thres, offset):
        rospy.loginfo('Reaching')
        linear_movement = self.behaviors.movement
        #linear_movement.gripper_close()
        self.close_gripper()
        self.behaviors.movement.pressure_listener.rezero()
        #pdb.set_trace()
        #try:
        self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        #except lm.RobotSafetyError, e:
        #    rospy.loginfo('robot safety error %s' % str(e))

        def reach_with_back_up(point, thres, reach_direction):
            self.behaviors.reach(point, thres, reach_direction)
            try:
                r1, pos_error1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.05, 0., 0.]).T, stop='none')
            except lm.RobotSafetyError, e:
                rospy.loginfo('robot safety error %s' % str(e))
        change, press_ret = self.camera_change_detect(visual_change_thres, \
                                    #self.behaviors.reach, \
                                    reach_with_back_up, \
                                    #(point, pressure, np.matrix([0,0,0.]).T, np.matrix([.1,0,0]).T))
                                    (point, pressure, np.matrix([.1,0,0]).T))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel')
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        try:
            self.behaviors.movement.move_absolute(self.start_location_light_switch, stop='pressure_accel', pressure=3000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        rospy.loginfo('Reseting')
        return change, '', point+offset

    def drawer_push(self, point_bl):
        PUSH_TOLERANCE = .1
        #pdb.set_trace()
        linear_movement = self.behaviors.movement
        #linear_movement.gripper_open()
        #pdb.set_trace()
        self.open_gripper()
        self.behaviors.movement.pressure_listener.rezero()
        #self.robot.left_gripper.open(True, position=.08)
        rospy.loginfo("Moving to start location")
        #linear_movement.move_absolute((self.start_location_drawer[0], 
        #    np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))))
        linear_movement.move_absolute((self.start_location_drawer[0], 
            #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))), 
            stop='pressure_accel', pressure=1000)

        #calc front loc
        self.behaviors.movement.set_pressure_threshold(1000)
        loc_bl = self.behaviors.movement.arm_obj.pose_cartesian_tf()[0]
        front_loc = point_bl.copy()
        front_loc[0,0] = max(loc_bl[0,0], .4)

        #pdb.set_trace()
        #move to front
        rospy.loginfo("Moving to front location")
        #orientation = np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))
        orientation = np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))
        self.behaviors.movement.pressure_listener.rezero()
        r1, residual_error = self.behaviors.movement.move_absolute((front_loc, orientation), 
                                stop='pressure', pressure=1500)
        linear_movement.pressure_listener.rezero()

        #move until contact
        rospy.loginfo("Touching surface")
        try:
            linear_movement.move_relative_gripper(np.matrix([.5,0,0]).T, stop='pressure_accel', pressure=100)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        contact_loc_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]

        #Push
        rospy.loginfo("PUSH!!!")
        current_position = self.robot.left.pose_cartesian_tf()
        target_position = current_position[0] + np.matrix([.4,0,0.]).T
        try:
            #linear_movement.move_relative_gripper(np.matrix([.2,0,0]).T, stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
            linear_movement.move_absolute((target_position, current_position[1]), stop='pressure_accel', pressure=6000)
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        pushed_loc_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]

        rospy.loginfo("Moving away")
        try:
            linear_movement.move_relative_gripper(np.matrix([-.05,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.10,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        try:
            linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))

        linear_movement.pressure_listener.rezero()
        #linear_movement.move_relative_base(np.matrix([-.2, .3, 0.1]).T, stop='pressure_accel', pressure=300)
        linear_movement.move_absolute((self.start_location_drawer[0], 
                    #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
                    np.matrix(tr.quaternion_from_euler(np.radians(0.), 0, 0))), 
                    stop='pressure_accel', pressure=1000)

        move_dist = np.linalg.norm(contact_loc_bl - pushed_loc_bl)
        rospy.loginfo('pushed for distance %.3f' % move_dist)
        success = move_dist > PUSH_TOLERANCE
        return success, 'pushed', pushed_loc_bl

    def drawer(self, point):
        #Prepare
        GRIPPER_OPEN = .08
        GRIPPER_CLOSE = .003
        MAX_HANDLE_SIZE = .03
        linear_movement = self.behaviors.movement
        gripper = self.robot.left_gripper

        #gripper.open(True, position=GRIPPER_OPEN)
        #linear_movement.gripper_open()
        self.open_gripper()
        linear_movement.move_absolute((self.start_location_drawer[0], 
            #np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))), 
            stop='pressure_accel', pressure=1000)

        #Reach
        success, reason, touchloc_bl = self.behaviors.reach(point, 300, #np.matrix([0.0, 0, 0]).T, 
                             reach_direction=np.matrix([0.1, 0, 0]).T, 
                             orientation=np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0)))

        #Error recovery
        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            try:
                linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='pressure_accel', pressure=300)
            except lm.RobotSafetyError, e:
                rospy.loginfo('robot safety error %s' % str(e))
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='accel', pressure=300)
            return False, 'reach failed', point

        #Grasp
        GRASP_THRES = 100
        try:
            linear_movement.move_relative_gripper(np.matrix([-.01,0,0]).T, stop='none')
        except lm.RobotSafetyError, e:
            rospy.loginfo('robot safety error %s' % str(e))
        #lbf, rbf = linear_movement.pressure_listener.get_pressure_readings()
        #pdb.set_trace()
        self.close_gripper()
        #linear_movement.gripper_close()
        #gripper.open(True, position=GRIPPER_CLOSE)
        #linear_movement.pressure_listener.rezero()
        #laf, raf = linear_movement.pressure_listener.get_pressure_readings()

        #linear_movement.move_relative_gripper(np.matrix([-.05,0,0]).T, stop='none')
        #gripper.open(True, position=.03)
        #linear_movement.pressure_listener.rezero()
        #gripper.open(True, position=GRIPPER_CLOSE)
        #linear_movement.pressure_listener.rezero()
        #bf = np.row_stack((lbf, rbf))
        #af = np.row_stack((laf, raf))
        #pdb.set_trace()
        #grasped_handle = np.any(np.abs(af-bf) > GRASP_THRES) or (gripper.pose()[0,0] > GRIPPER_CLOSE)
        grasped_handle = (gripper.pose()[0,0] > GRIPPER_CLOSE) and (gripper.pose()[0,0] < MAX_HANDLE_SIZE)

        if not grasped_handle:
            rospy.loginfo('Failed to grasp handle :(')
            #linear_movement.gripper_open()
            self.open_gripper()
            #gripper.open(True, position=GRIPPER_OPEN)
            linear_movement.pressure_listener.rezero()
            linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='pressure_accel', pressure=300)
            self.behaviors.movement.move_absolute(self.start_location_drawer, stop='accel', pressure=300)
            return False, 'failed to grasp handle', point

        #Pull
        linear_movement.pressure_listener.rezero()
        linear_movement.move_relative_gripper(np.matrix([-.1,0,0]).T, stop='accel', pressure=2500)
        linear_movement.move_absolute(linear_movement.arm_obj.pose_cartesian_tf(), 
                                    stop='pressure_accel', pressure=300)
        #linear_movement.gripper_close()
        #linear_movement.gripper_close()
        self.close_gripper()
        rospy.sleep(1)
        linear_movement.pressure_listener.rezero()
        #lap, rap = linear_movement.pressure_listener.get_pressure_readings()
        #ap = np.row_stack((lap, rap))
        #still_has_handle = np.any(np.abs(ap-af) < GRASP_THRES) or (gripper.pose()[0,0] > GRIPPER_CLOSE)
        still_has_handle = gripper.pose()[0,0] > GRIPPER_CLOSE
        #pdb.set_trace()
        try:
            linear_movement.move_relative_base(np.matrix([-.15,0,0]).T, stop='accel', pressure=2500)
        except lm.RobotSafetyError, e:
            #linear_movement.gripper_open()
            self.open_gripper()
            linear_movement.pressure_listener.rezero()
            rospy.loginfo('robot safety error %s' % str(e))

        #Release & move back 
        #linear_movement.gripper_open()
        location_handle_bl = linear_movement.arm_obj.pose_cartesian_tf()[0]
        #gripper.open(True, position=.08)
        #linear_movement.gripper_open()
        self.open_gripper()
        rospy.sleep(2)
        linear_movement.pressure_listener.rezero()

        #linear_movement.move_relative_gripper(np.matrix([-.15, 0, 0]).T, stop='pressure_accel', pressure=300)
        linear_movement.move_relative_base(np.matrix([-.2, 0, 0.]).T, stop='pressure_accel', pressure=1000)
        linear_movement.move_relative_base(np.matrix([-.1, .2, 0.1]).T, stop='pressure_accel', pressure=1000)
        self.behaviors.movement.move_absolute(self.start_location_drawer, stop='pressure_accel', pressure=1000)

        return still_has_handle, 'pulled', location_handle_bl

