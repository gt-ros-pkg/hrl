import roslib; roslib.load_manifest('hai_sandbox')
import rospy
#import hrl_pr2_lib.pr2_kinematics as pk

#import pr2_msgs.msg as pm
#import pr2_gripper_reactive_approach.reactive_grasp as rgr
#import pr2_gripper_reactive_approach.controller_manager as con
#from pr2_gripper_sensor_msgs.msg import PR2GripperEventDetectorGoal
#import object_manipulator.convert_functions as cf
#from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Point 
import tf.transformations as tr
from std_msgs.msg import String
import tf
import cv

#import copy
import numpy as np
import math
import time
import subprocess as sb
import scipy.spatial as sp
import pdb
import os

import hrl_camera.ros_camera as rc
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.devices as hd
import hrl_lib.rutils as ru
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import hrl_lib.prob as pr
import hrl_pr2_lib.linear_move as lm
#import hrl_pr2_lib.devices as de

import hrl_pr2_lib.collision_monitor as cmon
import hai_sandbox.recognize_3d as r3d

#from sound_play.msg import SoundRequest

class TaskError(Exception):
    def __init__(self, value):
        self.parameter = value

    def __str__(self):
        return repr(self.parameter)

class ActionType:
    def __init__(self, inputs, outputs):
        self.inputs = inputs
        self.outputs = outputs

class ParamType:
    def __init__(self, name, ptype, options=None):
        self.name = name
        self.ptype = ptype
        self.options = None

class Action:

    def __init__(self, name, params):
        self.name = name
        self.params = params

class BehaviorDescriptor:

    def __init__(self):
        self.descriptors = {
                            'twist':       ActionType([ParamType('angle', 'radian')], [ParamType('success', 'bool')]),
                            'linear_move': ActionType([ParamType('start_loc', 'se3'), 
                                                       ParamType('movement', 'r3'), 
                                                       ParamType('stop', 'discrete', ['pressure', 'pressure_accel'])], 
                                                      [ParamType('success', 'bool')]),
                            }

        start_location = (np.matrix([0.3, 0.15, 0.9]).T, np.matrix([0., 0., 0., 0.1]))
        movement       = np.matrix([.4, 0, 0.]).T
        self.seed = [Action('linear_move', [start_location, movement, 'pressure']),
                     Action('linear_move', [Action('current_location', [])])]
        self.run(self.seed)


class LaserPointerClient:
    def __init__(self, target_frame='/base_link', tf_listener=None, robot=None):
        self.dclick_cbs = []
        #self.dclick_cbs_raw = []
        self.point_cbs = []
        self.target_frame = target_frame
        self.laser_point_base = None
        self.robot = robot
        self.base_sound_path = (sb.Popen(["rospack", "find", "hai_sandbox"], stdout=sb.PIPE).communicate()[0]).strip()

        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()

    def transform_point(self, point_stamped):
        point_head = point_stamped.point
        #Tranform into base link
        base_T_head = tfu.transform(self.target_frame, point_stamped.header.frame_id, self.tf_listener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, _ = tfu.matrix_as_tf(point_mat_base)
        return np.matrix(t_base).T
        
    def laser_point_handler(self, point_stamped):
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/blow.wav').play()
        self.laser_point_base = self.transform_point(point_stamped)
        for f in self.point_cbs:
            f(self.laser_point_base)

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        self.robot.sound.waveSound(self.base_sound_path + '/sounds/beep.wav').play()
        #if self.laser_point_base != None:
        for f in self.dclick_cbs:
            f(self.laser_point_base)
        self.laser_point_base = None
        #for f in self.dclick_cb_raw(

    def add_double_click_cb(self, func):
        self.dclick_cbs.append(func)

    #def add_double_click_cb_raw(self, func):
    #    self.dclick_cbs_raw.append(func)

    def add_point_cb(self, func):
        self.point_cbs.append(func)



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
            rospy.loginfo('call to init_node failed')
        self.movement = lm.LinearReactiveMovement(arm, pr2_obj, tf_listener)


    def reach(self, point, pressure_thres, move_back_distance):
        self.movement.set_pressure_threshold(pressure_thres)
        loc_bl = self.movement.current_location()[0]
        front_loc = point.copy()
        front_loc[0,0] = loc_bl[0,0]

        start_loc = self.movement.current_location()
        self.movement.pressure_listener.rezero()
        r1, _ = self.movement.move_absolute((front_loc, start_loc[1]), stop='pressure', pressure=pressure_thres)
        r1, _ = self.movement.move_absolute((front_loc, start_loc[1]), stop='pressure', pressure=pressure_thres)
        if r1 != None and r1 != 'no solution': #if this step fails, we move back then return
            #self.move_absolute(start_loc, stop='accel')
            return False, r1, None

        #We expect impact here
        try:
            r2, pos_error = self.movement.move_absolute((point, self.movement.current_location()[1]), stop='pressure', pressure=pressure_thres)
        except lm.RobotSafetyError, e:
            pass
        touch_loc_bl = self.movement.current_location()
        if r2 == None or r2 == 'pressure' or r2 == 'accel' or pos_error < .05:
            self.movement.set_movement_mode_cart()
            self.movement.pressure_listener.rezero()
            #b/c of stiction, we can't move precisely for small distances
            self.movement.move_relative_gripper(4.*move_back_distance, stop='none', pressure=pressure_thres)
            self.movement.move_relative_gripper(-3.*move_back_distance, stop='none', pressure=pressure_thres)
            self.movement.pressure_listener.rezero()
            return True, r2, touch_loc_bl
        else:
            #shouldn't get here
            return False, r2, None


    def press(self, direction, press_pressure, contact_pressure):
        #make contact first
        self.movement.set_movement_mode_cart()
        #pdb.set_trace()
        r1 = self.movement.move_relative_gripper(direction, stop='pressure', pressure=contact_pressure)
        #now perform press
        if r1 == 'pressure' or r1 == 'accel':
            self.movement.set_movement_mode_cart()
            r2 = self.movement.move_relative_gripper(direction, stop='pressure_accel', pressure=press_pressure)
            if r2 == 'pressure' or r2 == 'accel' or r2 == None:
                return True, r2
            else:
                return False, r2
        else:
            return False, r1


    def twist(self, angle):
        pos, rot = self.movement.cman.return_cartesian_pose() # in base_link
        ax, ay, az = tr.euler_from_quaternion(rot) 
        nrot = tr.quaternion_from_euler(ax + angle, ay, az)
        stop_funcs = self.movement._process_stop_option(stop)
        #return self._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, 
        #                            stop_funcs, timeout=self.timeout, settling_time=5.0)
        return self.movement._move_cartesian(np.matrix(pos).T, np.matrix(nrot).T, \
                stop_funcs, timeout=self.timeout, settling_time=5.0)


class ApplicationBehaviors:

    def __init__(self):
        self.LOCATION_ADD_RADIUS = .5
        rospy.init_node('linear_move', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.robot = pr2.PR2(self.tf_listener, base=True)
        self.behaviors = ManipulationBehaviors('l', self.robot, tf_listener=self.tf_listener)
        self.laser_scan = hd.LaserScanner('point_cloud_srv')
        #self.prosilica = rc.Prosilica('prosilica', 'streaming')
        self.prosilica = rc.Prosilica('prosilica', 'polled')
        self.prosilica_cal = rc.ROSCameraCalibration('/prosilica/camera_info')
        self.left_cal = rc.ROSCameraCalibration('/wide_stereo/left/camera_info')
        self.right_cal = rc.ROSCameraCalibration('/wide_stereo/right/camera_info')

        self.wide_angle_camera_left = rc.ROSCamera('/wide_stereo/left/image_rect_color')
        self.wide_angle_camera_right = rc.ROSCamera('/wide_stereo/right/image_rect_color')

        #pdb.set_trace()
        self.laser_listener = LaserPointerClient(tf_listener=self.tf_listener, robot=self.robot)
        self.laser_listener.add_double_click_cb(self.click_cb)

        self.critical_error = False

        #self.behaviors.set_pressure_threshold(300)
        #TODO: define start location in frame attached to torso instead of base_link
        self.start_location = (np.matrix([0.35, 0.30, 1.1]).T, np.matrix([0., 0., 0., 0.1]))
        #self.start_location = (np.matrix([0.25, 0.30, 1.3]).T, np.matrix([0., 0., 0., 0.1]))
        #pdb.set_trace()

        # loading stored locations
        self.saved_locations_fname = 'saved_locations.pkl'
        self.locations = []
        self.location_labels = []
        self.location_data = []

        if os.path.isfile(self.saved_locations_fname):
            location_data = ut.load_pickle(self.saved_locations_fname) #each col is a 3d point, 3xn mat
            for idx, rloc in enumerate(location_data):
                self.locations.append(rloc['center'])
                self.location_labels.append(idx)
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.locations).T))
            self.location_data = location_data



        # joint angles used for tuck
        self.behaviors.movement.set_movement_mode_cart()
        self.r1 = np.matrix([[-0.31006769,  1.2701541 , -2.07800829, -1.45963243, -4.35290489,
                         -1.86052221,  5.07369192]]).T
        self.l0 = np.matrix([[  1.05020383,  -0.34464327,   0.05654   ,  -2.11967694,
                         -10.69100221,  -1.95457839,  -3.99544713]]).T
        self.l1 = np.matrix([[  1.06181076,   0.42026402,   0.78775801,  -2.32394841,
                         -11.36144995,  -1.93439025,  -3.14650108]]).T
        self.l2 = np.matrix([[  0.86275197,   0.93417818,   0.81181124,  -2.33654346,
                         -11.36121856,  -2.14040499,  -3.15655164]]).T
        self.l3 = np.matrix([[ 0.54339568,  1.2537778 ,  1.85395725, -2.27255481, -9.92394984,
                         -0.86489749, -3.00261708]]).T
        #pdb.set_trace()
        self.tuck()

    def go_to_home_pose(self):
        self.behaviors.movement.set_movement_mode_cart()
        self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')
        self.behaviors.movement.set_movement_mode_ik()
        return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')

    def camera_change_detect(self, threshold, f, args):
        #take before sensor snapshot
        start_pose = self.robot.head.pose()
        self.robot.head.set_pose(np.radians(np.matrix([1.04, -20]).T), 1)
        time.sleep(4)
        for i in range(4):
            before_frame = self.wide_angle_camera_left.get_frame()
        cv.SaveImage('before.png', before_frame)
        f_return = f(*args)
        time.sleep(2)
        for i in range(3):
            after_frame = self.wide_angle_camera_left.get_frame()

        cv.SaveImage('after.png', after_frame)
        sdiff = image_diff_val2(before_frame, after_frame)
        #pdb.set_trace()
        self.robot.head.set_pose(start_pose, 1)
        time.sleep(3)        
        #take after snapshot
        #threshold = .03
        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, threshold))
        if sdiff > threshold:
            rospy.loginfo('difference detected!')
            return True, f_return
        else:
            rospy.loginfo('NO differences detected!')
            return False, f_return

    def light_switch1(self, point, 
            point_offset, press_contact_pressure, move_back_distance,
            press_pressure, press_distance, visual_change_thres):
        print '===================================================================='
        point = point + point_offset 
        rospy.loginfo('>>>> REACHING to ' + str(point.T))
        #self.behaviors.gripper_close()
        #TODO: have go_home check whether it is actually at that location
        #self.behaviors.move_absolute(self.start_location, stop='pressure_accel')

        #start_loc = self.current_location()
        #pdb.set_trace()
        success, reason, touchloc_bl = self.behaviors.reach(point, press_contact_pressure, move_back_distance)
        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            self.behaviors.movement.move_absolute(self.start_location, stop='accel', \
                    pressure=press_contact_pressure)
            #raise TaskError(error_msg)
            return False, None

        rospy.loginfo('>>>> PRESSING')
        #should not be making contact
        self.behaviors.movement.pressure_listener.rezero()
        change, press_ret = self.camera_change_detect(visual_change_thres, self.behaviors.press, (press_distance, press_pressure, press_contact_pressure))
        success, reason = press_ret
        if not success:
            rospy.loginfo('Press failed due to "%s"' % reason)

        #code reward function
        #monitor self collision => collisions with the environment are not self collisions
        rospy.loginfo('>>>> MOVING BACK')
        self.behaviors.movement.set_movement_mode_cart()
        r1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.03, 0., 0.]).T, \
                stop='none', pressure=press_contact_pressure)
        if r1 != None:
            rospy.loginfo('moving back failed due to "%s"' % r1)
            return False, None

        rospy.loginfo('>>>> RESETING')
        r2, pos_error = self.go_to_home_pose()
        if r2 != None and r2 != 'no solution':
            rospy.loginfo('moving back to start location failed due to "%s"' % r2)
            return False, None
        self.behaviors.movement.pressure_listener.rezero()

        rospy.loginfo('DONE.')
        return change, touchloc_bl

    def light_switch2(self, point):
        success, reason, touchloc = self.behaviors.movement.reach(point)
        if not success:
            rospy.loginfo('Reach failed due to "%s"' % reason)

        rospy.loginfo('RESETING')
        r2, pos_error = self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')
        if r2 != None:
            rospy.loginfo('moving back to start location failed due to "%s"' % r2)
            return 

    def record_perceptual_data(self, point_touched_bl):
        #what position should the robot be in?
        #set arms to non-occluding pose

        #record region around the finger where you touched
        rospy.loginfo('Getting laser scan.')
        points = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 20.)
        rospy.loginfo('Getting Prosilica image.')
        prosilica_image = self.prosilica.get_frame()
        rospy.loginfo('Getting image from left wide angle camera.')
        left_image  = self.wide_angle_camera_left.get_frame()
        rospy.loginfo('Getting image from right wide angle camera.')
        right_image = self.wide_angle_camera_left.get_frame()
        rospy.loginfo('Waiting for calibration.')
        while self.prosilica_cal.has_msg == False:
            time.sleep(.1)

        #which frames?
        rospy.loginfo('Getting transforms.')
        pro_T_bl = tfu.transform('/high_def_optical_frame', '/base_link', self.tf_listener)
        laser_T_bl = tfu.transform('/laser_tilt_link', '/base_link', self.tf_listener)
        tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
        prosilica_name = '%s_highres.png' % tstring
        left_name = '%s_left.png' % tstring
        right_name = '%s_right.png' % tstring
        rospy.loginfo('Saving images (basename %s)' % tstring)
        cv.SaveImage(prosilica_name, prosilica_image)
        cv.SaveImage(left_name, left_image)
        cv.SaveImage(right_name, right_image)

        rospy.loginfo('Saving pickles')
        pickle_fname = '%s_interest_point_dataset.pkl' % tstring   
        ut.save_pickle({'touch_point': point_touched_bl,
                        'points_laser': points,

                        'high_res': prosilica_name,
                        'left_image': left_name,
                        'right_image': right_name,

                        'laser_T_bl': laser_T_bl, 
                        'pro_T_bl': pro_T_bl,
                        'point_touched': point_touched_bl,
                        
                        'prosilica_cal': self.prosilica_cal, 
                        'left_cal': self.left_cal,
                        'right_cal': self.right_cal},
                        pickle_fname)
        print 'Recorded to', pickle_fname

    def gather_interest_point_dataset(self, point):
        gaussian = pr.Gaussian(np.matrix([0, 0, 0.]).T, \
                np.matrix([[1., 0, 0], \
                           [0, .02**2, 0], \
                           [0, 0, .02**2]]))

        for i in range(100):
            # perturb_point
            gaussian_noise = gaussian.sample()
            gaussian_noise[0,0] = 0
            #npoint = point + gaussian_noise
            #success_off, touchloc_bl = self.light_switch1(npoint, 
            #pdb.set_trace()
            success_off, touchloc_bl = self.light_switch1(point, 
                            point_offset=np.matrix([-.15, 0, 0]).T, press_contact_pressure=300, 
                            move_back_distance=np.matrix([-.005,0,0]).T, press_pressure=2500, 
                            press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
            rospy.loginfo('Lights turned off? %s' % str(success_off))

            pdb.set_trace()
            self.behaviors.movement.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
            self.record_perceptual_data(touchloc_bl)
            self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')
            if success_off:
                self.behaviors.movement.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
                self.record_perceptual_data(touchloc_bl)
                self.behaviors.movement.move_absolute(self.start_location, stop='pressure_accel')

                success_on, touchloc_bl2 = self.light_switch1(point, 
                                point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, 
                                move_back_distance=np.matrix([-0.005, 0, 0]).T, press_pressure=2500, 
                                press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)
                ##1
                #if success_on:
                #    self.movement.behaviors.move_absolute((np.matrix([.15, .45, 1.3]).T, self.start_location[1]), stop='pressure_accel')
                #    self.record_perceptual_data(touchloc_bl)
                #    self.movement.behaviors.move_absolute(self.start_location, stop='pressure_accel')
                #Turn on lights
                #success_on, touchloc_bl = self.light_switch1(npoint, 
            else:
                return

    def drive_approach_behavior(self, point_bl, dist_far):
        # navigate close to point
        #pdb.set_trace()
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

    def approach_perpendicular_to_surface(self, point_bl, voi_radius, dist_approach):
        #TODO: Turn to face point
        #TODO: make this scan around point instead of total scan of env
        #determine normal
        #pdb.set_trace()
        map_T_base_link0 = tfu.transform('map', 'base_link', self.tf_listener)
        point_map0 = tfu.transform_points(map_T_base_link0, point_bl)
        self.turn_to_point(point_bl)

        point_bl = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), \
                                        point_map0)
        point_cloud_bl = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 2.5)
        point_cloud_np_bl = ru.pointcloud_to_np(point_cloud_bl)
        voi_points_bl, limits_bl = r3d.select_rect(point_bl, voi_radius, voi_radius, voi_radius, point_cloud_np_bl)
        #TODO: use closest plane instead of closest points determined with KDTree
        normal_bl = r3d.calc_normal(voi_points_bl)
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
        self.turn_to_point(point_bl)
        return rvalue
        #ang = math.atan2(point_bl[1,0], point_bl[0,0])
        #self.robot.base.turn_by(ang, block=True)
        #pdb.set_trace()

    def turn_to_point(self, point_bl):
        ang = math.atan2(point_bl[1,0], point_bl[0,0])
        print 'turn_to_point: turning by %.2f deg' % math.degrees(ang)
        self.robot.base.turn_by(-ang, block=True)

    def tuck(self):
        #pdb.set_trace()
        ldiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.left.pose(), self.l3))
                # np.linalg.norm(self.robot.left.pose() - self.l3)
        rdiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.right.pose(), self.r1))
        #rdiff = np.linalg.norm(self.robot.right.pose() - self.r1)
        if ldiff < .3 and rdiff < .3:
            rospy.loginfo('tuck: Already tucked. Ignoring request.')
            return
        self.robot.right.set_pose(self.r1, block=False)
        self.robot.left.set_pose(self.l0, block=True)
        poses = np.column_stack([self.l0, self.l1, self.l2, self.l3])
        self.robot.left.set_poses(poses, np.array([0., 3., 6., 9]))

    def untuck(self):
        if np.linalg.norm(self.robot.left.pose() - self.l0) < .3:
            rospy.loginfo('untuck: Already untucked. Ignoring request.')
            return
        self.robot.right.set_pose(self.r1, 2., block=False)
        self.robot.left.set_pose(self.l3, 2., block=True)
        poses = np.column_stack([self.l3, self.l2, self.l1, self.l0])
        self.robot.left.set_poses(poses, np.array([0., 3., 6., 9.]))

    def click_cb(self, point_bl):
        if point_bl!= None:
            self.run_behaviors(point_bl)
        else:
            if len(self.locations) < 1:
                return
            rospy.loginfo('click_cb: double clicked but no 3d point given')
            rospy.loginfo('click_cb: will use the last successful location given')
            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl = tfu.transform_points(base_link_T_map, self.locations[-1])
            rospy.loginfo('click_cb: using ' + str(self.locations[-1].T))
            self.run_behaviors(point_bl, stored_point=True)

    def run_behaviors(self, point_bl_t0, stored_point=False):
        point_dist = np.linalg.norm(point_bl_t0[0:2,0])
        rospy.loginfo('run_behaviors: Point is %.3f away.' % point_dist)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl_t0)
        #self.location_add(point_map)
        #self.location_add(np.matrix([[-1.12325851], [-0.61091271], [ 1.26014893]])).T

        DIST_STOP = .7
        DIST_THRESHOLD = .8
        if point_dist > DIST_THRESHOLD:
            rospy.loginfo('run_behaviors: Point is greater than %.1f m away (%.3f).  Driving closer.' % (DIST_THRESHOLD, point_dist))
            ##self.turn_to_point(point_bl_t0)
            rospy.loginfo( 'run_behaviors: CLICKED on point_bl ' + str(point_bl_t0.T))

            ret = self.drive_approach_behavior(point_bl_t0, dist_far=DIST_STOP)
            if ret != 3:
                base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
                dist_end = np.linalg.norm(point_bl_t1[0:2,0])
                if dist_end > DIST_THRESHOLD:
                    rospy.logerr('run_behaviors: drive_approach_behavior failed! %.3f' % dist_end)
                    self.robot.sound.say("I am unable to navigate to that location")
                    return

            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)

            ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=.2, dist_approach=.50)
            if ret != 3:
                rospy.logerr('run_behaviors: approach_perpendicular_to_surface failed!')
                return

            #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
            #point_bl_t2 = tfu.transform_points(base_link_T_map, point_map)
            self.robot.sound.say('done')
            rospy.loginfo('run_behaviors: DONE DRIVING!')
        else:
            rospy.loginfo('run_behaviors: Point is less than %.1f m away (%.3f).  Attempting manipulation' % (DIST_THRESHOLD, point_dist))
            try:
                ret = self.approach_perpendicular_to_surface(point_bl_t0, voi_radius=.2, dist_approach=.50)
                if ret != 3:
                    rospy.logerr('run_behaviors: approach_perpendicular_to_surface failed!')
                    return

                base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
                point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
                rospy.loginfo('run_behaviors: go_home_pose')
                #pdb.set_trace()
                self.untuck()
                self.go_to_home_pose()
                #self.go_to_home_pose()
                success_off, _ = self.light_switch1(point_bl_t1, 
                                point_offset=np.matrix([0,0,.03]).T, press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
                                press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
                if success_off and not stored_point:
                    self.location_add(point_map)
                self.tuck()
            except lm.RobotSafetyError, e:
                rospy.loginfo('run_behaviors: Caught a robot safety exception "%s"' % str(e.parameter))
                self.behaviors.movement.move_absolute(self.start_location, stop='accel')

            except TaskError, e:
                rospy.loginfo('run_behaviors: TaskError: %s' % str(e.parameter))
            rospy.loginfo('run_behaviors: DONE MANIPULATION!')
            self.robot.sound.say('done')

    def location_add(self, point_map):
        #pdb.set_trace()
        close_by_locs = self.locations_tree.query_ball_point(np.array(point_map.T), self.LOCATION_ADD_RADIUS)[0]
        if len(close_by_locs) == 0:
            rospy.loginfo('location_add: point not close to any existing location. creating new record.')
            self.location_data.append({'center': point_map, 'points':[point_map]})
            self.locations.append(point_map)
            self.location_labels.append(len(self.location_data) - 1)
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.locations).T))
        else:
            #If close by locations found then add to points list and update center
            location_idx = self.location_labels[close_by_locs[0]]
            ldata = self.location_data[location_idx]

            rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
            ldata['points'].append(point_map)
            ldata['center'] = np.column_stack(ldata['points']).mean(1)
            self.locations[location_idx] = ldata['center']
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.locations).T))

        ut.save_pickle(self.location_data, self.saved_locations_fname)
        rospy.loginfo('location_add: saved point in map.')

    def run(self):
        #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
        #print 'RECORDING'
        #self.record_perceptual_data(point)
        #print 'DONE RECORDING'

        r = rospy.Rate(10)
        rospy.loginfo('Ready.')
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    l = ApplicationBehaviors()
    l.run()



























        #if tf_listener == None:
        #    self.tf_listener = tf.TransformListener()
        #else:
        #    self.tf_listener = tf_listener

        #self.pr2 = pr2_obj
        #self.cman = con.ControllerManager(arm, self.tf_listener, using_slip_controller=1)
        #self.reactive_gr = rgr.ReactiveGrasper(self.cman)
        #if arm == 'l':
        #    ptopic = '/pressure/l_gripper_motor'
        #    self.arm_obj = self.pr2.left
        #    self.ik_frame = 'l_wrist_roll_link'
        #    self.tool_frame = 'l_gripper_tool_frame'
        #else:
        #    ptopic = '/pressure/r_gripper_motor'
        #    self.arm_obj = self.pr2.right
        #    self.ik_frame = 'r_wrist_roll_link'
        #    self.tool_frame = 'r_gripper_tool_frame'
        #self.movement_mode = 'ik' #or cart

        #rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        #self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)

    #def set_movement_mode_ik(self):
    #    self.movement_mode = 'ik'
    #    self.reactive_gr.cm.switch_to_joint_mode()
    #    self.reactive_gr.cm.freeze_arm()

    #def set_movement_mode_cart(self):
    #    self.movement_mode = 'cart'







                #pdb.set_trace()
                #self.gather_interest_point_dataset(point)
                #point = np.matrix([ 0.60956734, -0.00714498,  1.22718197]).T
                #pressure_parameters = range(1900, 2050, 30)

                #self.record_perceptual_data(point)
                #successes = []
                #parameters = [np.matrix([-.15, 0, 0]).T, 300, np.matrix([-.005, 0, 0]).T, 3500, np.matrix([0,0,-.15]).T, .03]

                #for p in pressure_parameters:
                #    experiment = []
                #    for i in range(4):
                #        #Turn off lights
                #        rospy.loginfo('Experimenting with press_pressure = %d' % p)
                #        success_off = self.light_switch1(point, 
                #                        point_offset=np.matrix([-.15,0,0]).T, press_contact_pressure=300, move_back_distance=np.matrix([-.005,0,0]).T,\
                #                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, visual_change_thres=.03)
                #        experiment.append(success_off)
                #        rospy.loginfo('Lights turned off? %s' % str(success_off))
                #        return

                #        #Turn on lights
                #        success_on = self.light_switch1(point, 
                #                        point_offset=np.matrix([-.15,0,-.10]).T, press_contact_pressure=300, move_back_distance=np.matrix([-0.005, 0, 0]).T,
                #                        press_pressure=3500, press_distance=np.matrix([0,0,.1]).T, visual_change_thres=.03)
                #        #def light_switch1(self, point, 
                #        #        point_offset, press_contact_pressure, move_back_distance,
                #        #        press_pressure, press_distance, visual_change_thres):

                #        print 'Lights turned on?', success_on
                #    successes.append(experiment)

                #ut.save_pickle({'pressure': pressure_parameters, 
                #                'successes': successes}, 'pressure_variation_results.pkl')










        #return self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        ##stop if you hit a tip, side, back, or palm
        #(left_touching, right_touching, palm_touching) = self.reactive_gr.check_guarded_move_contacts()
        ##saw a contact, freeze the arm
        #if left_touching or right_touching or palm_touching:
        #    rospy.loginfo("CONTACT made!")
        #    return True
        #else:
        #    return False

        #print 'move returning'
        #return whether the left and right fingers were touching
        #return (left_touching, right_touching, palm_touching)




    #def execute_action_list(self):

    #def run(self, seed):
    #    # search for pairs of perception operators and manipulation operators that would work
    #    population = 10
    #    seeds = []
    #    for i in range(population):
    #        aseed = copy.deepcopy(seed)
    #        # 'bool', 'radian', 'se3', 'r3', 'discrete', 
    #        new_seed_actions = []
    #        for action in aseed:

    #            if replace_action:
    #                pass

    #            if delete_action:
    #                pass
    #            
    #            if insert_action:
    #                #pick random action from descriptors list
    #                new_action = 
    #                new_seed_actions += new_action
    #                pass
    #            
    #            if perturb_parameter:
    #                num_params = len(action.params)
    #                rand_param_idx = ...
    #                self.descriptors[action.name].params[rand_param_idx]
    #                rand_param_types[rand_param_types]


    #            #can replace/delete/insert action
    #            #can pick a parameter and perturb it

    #    pdb.set_trace()
    #    print seed

        #point = np.matrix([0.63125642, -0.02918334, 1.2303758 ]).T
        #print 'move direction', movement.T
        #print 'CORRECTING', point.T
        #print 'NEW', point.T
        #start_location = (np.matrix([0.25, 0.15, 0.7]).T, np.matrix([0., 0., 0., 0.1]))
        #movement = np.matrix([.4, 0., 0.]).T
        #what other behavior would I want?
        # touch then move away..
        # move back but more slowly..
        # want a safe physical
        #   a safe exploration strategy
        #self.behaviors.linear_move(self.behaviors.current_location(), back_alittle, stop='none')
        #loc_before = self.behaviors.current_location()[0]
        #loc_after = self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), down, stop='pressure_accel')
        #self.behaviors.linear_move(self.behaviors.current_location(), back, stop='none')
        #pdb.set_trace()
        #b.twist(math.radians(30.))
        #bd = BehaviorDescriptor()
        #movement = point - self.behaviors.current_location()[0]
        #pdb.set_trace()
        #self.behaviors.linear_move(self.behaviors.current_location(), movement, stop='pressure_accel')

        #loc = self.behaviors.current_location()[0]
        #front_loc = point.copy()
        #front_loc[0,0] = loc[0,0]
        #self.behaviors.set_pressure_threshold(150)
        #self.behaviors.move_absolute((front_loc, self.behaviors.current_location()[1]), stop='pressure_accel')
        #self.behaviors.move_absolute((point, self.behaviors.current_location()[1]), stop='pressure_accel')




    #def detect_event(self):
    #    self.behaviors.cman._start_gripper_event_detector(timeout=40.)
    #    stop_func = self.behaviors._tactile_stop_func
    #    while stop_func():

        #pass
        #self.robot = pr2.PR2()
        #self.kin = pk.PR2Kinematics(self.robot.tf_listener)

    #def linear_move(self, start_location, direction, distance, arm):
    #    if arm == 'left':
    #        arm_kin = self.kin.left
    #    else:
    #        arm_kin = self.kin.right

    #    start_pose = arm_kin.ik(start_location)
    #    loc = start_location[0:3, 4]
    #    end_location = loc + distance*direction
    #    end_pose = arm_kin.ik(end_location)

    #    self.robot.left_arm.set_pose(start_pose, 5.)             #!!!
    #    self.robot.left_arm.set_pose(end_pose, 5.)               #!!!

            ##stop if you hit a tip, side, back, or palm
            #(left_touching, right_touching, palm_touching) = rg.check_guarded_move_contacts()
            ##saw a contact, freeze the arm
            #if left_touching or right_touching or palm_touching:
            #    rospy.loginfo("saw contact")
            #    rg.cm.switch_to_joint_mode()
            #    rg.cm.freeze_arm()
            #    break

    #import pdb
    #start_location = [0.34, 0.054, 0.87] + [0.015454981255042808, -0.02674860197736427, -0.012255429236635201, 0.999447577565171]
    #direction = np.matrix([1., 0., 0.]).T

    #self.reactive_l.move_cartesian_step(start_location, blocking = 1)
    #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #self.cman_r     = con.ControllerManager('r')
        #self.reactive_r = rgr.ReactiveGrasper(self.cman_r)

        #self.cman_r.start_joint_controllers()
        #self.reactive_r.start_gripper_controller()
    
        #(pos, rot) = self.cman.return_cartesian_pose()
        #pdb.set_trace()
        #currentgoal = pos + rot
        #currentgoal[2] -= .05
        #self.reactive_l.move_cartesian_step(currentgoal, blocking = 1)
        #(left_touching, right_touching, palm_touching) = self.reactive_l.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        #exit()
        #end_loc = start_location + direction * distance
        #self.reactive_l.move_cartesian_step(start_loc, blocking = 1)
        #self.reactive_l.move_cartesian_step(end_loc, blocking = 1)
    #left_pose = b.robot.left.pose()
    #left_cart = ut.load_pickle('start_pose.pkl')
    #pdb.set_trace()
    #kin_sol = b.kin.left.ik(left_cart)
    #b.robot.left.set_pose(kin_sol, 5.)
    ##b.linear_move(left_cart)
    ##left_cart = b.kin.left.fk(left_pose)
    ##pdb.set_trace()
    #print left_cart

    #(pos, rot) = cm.return_cartesian_pose()
    #currentgoal = pos+rot
    #currentgoal[2] -= .05
    #rg.move_cartesian_step(currentgoal, blocking = 1)
    #exit()


#b.linear_move()
#cart_pose = kin.left.fk('torso_lift_link', 'l_wrist_roll_link', joints)
#kin.left.ik(cart_pose, 'torso_lift_link')

    #def light_switch1_on(self, point, press_pressure=3500, press_contact_pressure=150):
    #    point = point + np.matrix([-.15, 0, -0.20]).T

    #    success, reason = self.behaviors.reach(point)
    #    if not success:
    #        rospy.loginfo('Reach failed due to "%s"' % reason)

    #    rospy.loginfo('PRESSING')
    #    success, reason = self.behaviors.press(np.matrix([0, 0, .20]).T, \
    #            press_pressure, press_contact_pressure)
    #    if not success:
    #        rospy.loginfo('Press failed due to "%s"' % reason)
    #        return 

    #    rospy.loginfo('RESETING')
    #    r2 = self.behaviors.move_absolute(self.start_location, stop='pressure_accel')
    #    if r2 != None:
    #        rospy.loginfo('moving back to start location failed due to "%s"' % r2)
    #        return 

    #    print 'DONE.'


    #def _tactile_stop_func(self):
    #    r1 = self.pressure_listener.check_threshold() 
    #    r2 = self.pressure_listener.check_safety_threshold()
    #    if r1:
    #        rospy.loginfo('Pressure exceeded!')
    #    if r2:
    #        rospy.loginfo('Pressure safety limit EXCEEDED!')
    #    return r1 or r2







        #r1 = self.pressure_listener.check_threshold() 
        #r2 = self.pressure_listener.check_safety_threshold()
        #if r1:
        #    rospy.loginfo('Pressure exceeded!')
        #if r2:
        #    rospy.loginfo('Pressure safety limit EXCEEDED!')
        #pressure_state = r1 or r2
        #pressure_state = self.pressure_listener.check_threshold() or self.pressure_listener.check_safety_threshold()
        #action finished (trigger seen)




    #def optimize_parameters(self, x0, x_range, behavior, objective_func, reset_env_func, reset_param):
    #    reset_retries = 3
    #    num_params = len(x0)
    #    x = copy.deepcopy(x0)

    #    # for each parameter
    #    #for i in range(num_params):
    #    while i < num_params:
    #        #search for a good setting
    #        not_converged = True
    #        xmin = x_range[i, 0]
    #        xmax = x_range[i, 1]

    #        while not_converged:
    #            current_val = x[i]
    #            candidates_i = [(x[i] + xmin) / 2., (x[i] + xmax) / 2.]
    #            successes = []
    #            for cand in candidates_i:
    #                x[i] = cand
    #                success = behavior(x)
    #                if success:
    #                    for reset_i in range(reset_retries):
    #                        reset_success = reset_env_func(*reset_param)
    #                        if reset_success:
    #                            break
    #                successes.append(success)

    #            if successes[0] and successes[1]:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')
    #            elif successes[0] and not successes[1]:
    #                next_val = candidates_i[0]
    #            elif successes[1] and not successes[0]:
    #                next_val = candidates_i[1]
    #            else:
    #                raise RuntimeException('What? this isn\'t suppose to happen.')


    #        #if all the trials are bad
    #        if not test(successes):
    #            #go back by 1 parameter
    #            i = i - 1


    #        #if there are more than one good parameter
    #        for p in params
    #            ... = objective_func(p)

    #        i = i + 1

    #    return x


