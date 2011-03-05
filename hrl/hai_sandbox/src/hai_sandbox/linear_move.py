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
#import sensor_msgs.msg as smsg
#import message_filters
import tf
import cv
import functools as ft

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
from hai_sandbox.recognize_3d import InterestPointDataset
#import psyco
#psyco.full()

#import hai_sandbox.interest_point_actions as ipa
#import hai_sandbox.kinect_listener as kl

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

#TODO move this
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

    ##
    # reach direction
    def reach(self, point, pressure_thres, move_back_distance, \
            reach_direction=np.matrix([0,0,0]).T, orientation=None):
        MOVEMENT_TOLERANCE = .1
        #REACH_TOLERANCE = .1

        #self.movement.set_movement_mode_cart()
        self.movement.set_pressure_threshold(pressure_thres)
        loc_bl = self.movement.arm_obj.pose_cartesian_tf()[0]
        front_loc = point.copy()
        front_loc[0,0] = loc_bl[0,0]
        #pdb.set_trace()

        if orientation == None:
            start_loc = self.movement.arm_obj.pose_cartesian_tf()
            orientation = start_loc[1]
        self.movement.pressure_listener.rezero()
        #pdb.set_trace()
        #for i in range(2):
        r1, residual_error = self.movement.move_absolute((front_loc, orientation), stop='pressure', pressure=pressure_thres)

        if residual_error > MOVEMENT_TOLERANCE or r1 != None: #if this step fails, we move back then return
            #self.move_absolute(start_loc, stop='accel')
            return False, r1, None

        #We expect impact here
        try:
            #pdb.set_trace()
            #loc_bl = self.movement.current_location()[0]
            #reach_direction = loc_bl - point 
            #reach_direction = reach_direction / np.linalg.norm(reach_direction)
            point_reach = point + reach_direction
            r2, pos_error = self.movement.move_absolute((point_reach, \
                    self.movement.arm_obj.pose_cartesian_tf()[1]), stop='pressure_accel', pressure=pressure_thres)

        except lm.RobotSafetyError, e:
            r2 = None

        touch_loc_bl = self.movement.arm_obj.pose_cartesian_tf()
        #if r2 == None or r2 == 'pressure' or r2 == 'accel' or (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction))):
        if r2 == 'pressure' or r2 == 'accel' or (pos_error < (MOVEMENT_TOLERANCE + np.linalg.norm(reach_direction))):
            self.movement.pressure_listener.rezero()

            # b/c of stiction & low gains, we can't move precisely for small
            # distances, so move back then move forward again
            reach_dir = reach_direction / np.linalg.norm(reach_direction)
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

        self.laser_listener = LaserPointerClient(tf_listener=self.tf_listener, robot=self.robot)
        self.laser_listener.add_double_click_cb(self.click_cb)

        #self.kinect_listener = kl.KinectListener()
        #self.kinect_cal = rc.ROSCameraCalibration('camera/rgb/camera_info')
        self.kinect_features = r3d.KinectFeatureExtractor(self.tf_listener)

        #self.kinect_img_sub = message_filters.Subscriber('/camera/rgb/image_color', smsg.Image)
        #self.kinect_depth_sub = message_filters.Subscriber('/camera/depth/points2', smsg.PointCloud2)
        #ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
        #ts.registerCallback(callback)

        self.critical_error = False

        #self.behaviors.set_pressure_threshold(300)
        #TODO: define start location in frame attached to torso instead of base_link
        self.start_location = (np.matrix([0.35, 0.30, 1.1]).T, np.matrix([0., 0., 0., 0.1]))
        #self.start_location = (np.matrix([0.25, 0.30, 1.3]).T, np.matrix([0., 0., 0., 0.1]))

        #loading stored locations
        self.saved_locations_fname = 'saved_locations.pkl'
        self.location_centers = []
        self.location_labels = []
        self.location_data = []
        self.locations_tree = None

        if os.path.isfile(self.saved_locations_fname):
            location_data = ut.load_pickle(self.saved_locations_fname) #each col is a 3d point, 3xn mat
            for idx, rloc in enumerate(location_data):
                self.location_centers.append(rloc['center'])
                self.location_labels.append(idx)
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
            self.location_data = location_data

        # joint angles used for tuck
        self.create_arm_poses()
        #pdb.set_trace()
        #self.untuck()
        #self.behaviors.movement.set_movement_mode_ik()
        #self.movement.set_movement_mode_ik()
        #self.tuck()
        #self.r1 = np.matrix([[-0.31006769,  1.2701541 , -2.07800829, -1.45963243, -4.35290489,
        #                 -1.86052221,  5.07369192]]).T
        #self.l0 = np.matrix([[  1.05020383,  -0.34464327,   0.05654   ,  -2.11967694,
        #                 -10.69100221,  -1.95457839,  -3.99544713]]).T
        #self.l1 = np.matrix([[  1.06181076,   0.42026402,   0.78775801,  -2.32394841,
        #                 -11.36144995,  -1.93439025,  -3.14650108]]).T
        #self.l2 = np.matrix([[  0.86275197,   0.93417818,   0.81181124,  -2.33654346,
        #                 -11.36121856,  -2.14040499,  -3.15655164]]).T
        #self.l3 = np.matrix([[ 0.54339568,  1.2537778 ,  1.85395725, -2.27255481, -9.92394984,
        #                 -0.86489749, -3.00261708]]).T
        self.learners = {}
        #self.load_classifier('light_switch', 'labeled_light_switch_data.pkl')
        self.load_classifier('light_switch', 'friday_730_light_switch2.pkl')
        self.img_pub = r3d.ImagePublisher('active_learn')


    def load_classifier(self, name, fname):
        print 'loading classifier'
        dataset = ut.load_pickle(fname)
        self.train(dataset, name)

    def train(self, dataset, name):
        rec_params = self.kinect_features.rec_params
        nneg = np.sum(dataset.outputs == r3d.NEGATIVE) #TODO: this was copied and pasted from r3d
        npos = np.sum(dataset.outputs == r3d.POSITIVE)
        print '================= Training ================='
        print 'NEG examples', nneg
        print 'POS examples', npos
        print 'TOTAL', dataset.outputs.shape[1]
        neg_to_pos_ratio = float(nneg)/float(npos)
        weight_balance = ' -w0 1 -w1 %.2f' % neg_to_pos_ratio
        print 'training'
        learner = r3d.SVMPCA_ActiveLearner(use_pca=True)
        #TODO: figure out something scaling inputs field!
        learner.train(dataset, dataset.inputs,
                      rec_params.svm_params + weight_balance,
                      rec_params.variance_keep)
        self.learners[name] = {'learner': learner, 'dataset': dataset}
        print 'done loading'


    def draw_dots_nstuff(self, img, points2d, labels):
        pidx = np.where(labels == r3d.POSITIVE)[1].A1.tolist()
        nidx = np.where(labels == r3d.NEGATIVE)[1].A1.tolist()
        uidx = np.where(labels == r3d.UNLABELED)[1].A1.tolist()

        scale = 1

        if len(uidx) > 0:
            upoints = points2d[:, uidx]
            r3d.draw_points(img, upoints * scale, [255,255,255], 2, -1)

        if len(nidx) > 0:
            npoints = points2d[:, nidx]
            r3d.draw_points(img, npoints * scale, [0,0,255], 2, -1)

        if len(pidx) > 0:
            ppoints = points2d[:, pidx]
            r3d.draw_points(img, ppoints * scale, [0,255,0], 2, -1)

    #def tuck(self):
    #    ldiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.left.pose(), self.l3))
    #            # np.linalg.norm(self.robot.left.pose() - self.l3)
    #    rdiff = np.linalg.norm(pr2.diff_arm_pose(self.robot.right.pose(), self.r1))
    #    #rdiff = np.linalg.norm(self.robot.right.pose() - self.r1)
    #    if ldiff < .3 and rdiff < .3:
    #        rospy.loginfo('tuck: Already tucked. Ignoring request.')
    #        return
    #    self.robot.right.set_pose(self.r1, block=False)
    #    self.robot.left.set_pose(self.l0, block=True)
    #    poses = np.column_stack([self.l0, self.l1, self.l2, self.l3])
    #    #pdb.set_trace()
    #    self.robot.left.set_poses(poses, np.array([0., 1.5, 3, 4.5]))


    #def untuck(self):
    #    if np.linalg.norm(self.robot.left.pose() - self.l0) < .3:
    #        rospy.loginfo('untuck: Already untucked. Ignoring request.')
    #        return
    #    self.robot.right.set_pose(self.r1, 2., block=False)
    #    self.robot.left.set_pose(self.l3, 2.,  block=True)
    #    poses = np.column_stack([self.l3, self.l2, self.l1, self.l0])
    #    self.robot.left.set_poses(poses, np.array([0., 3., 6., 9.])/2.)

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
        #pdb.set_trace()
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
        #pdb.set_trace()
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

    def go_to_home_pose(self):
        #self.behaviors.movement.set_movement_mode_cart()
        return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
        #self.behaviors.movement.set_movement_mode_ik()
        #return self.behaviors.movement.move_absolute(self.start_location, stop='pressure')

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
        #pdb.set_trace()
        print '===================================================================='
        point = point + point_offset 
        rospy.loginfo('>>>> REACHING to ' + str(point.T))
        #pdb.set_trace()
        self.behaviors.movement.gripper_close()
        time.sleep(1)
        self.behaviors.movement.pressure_listener.rezero()
        #TODO: have go_home check whether it is actually at that location
        #self.behaviors.move_absolute(self.start_location, stop='pressure_accel')

        #start_loc = self.current_location()
        #pdb.set_trace()
        success, reason, touchloc_bl = self.behaviors.reach(point, \
                press_contact_pressure, move_back_distance, \
                reach_direction=np.matrix([0.1,0,0]).T)

        dist = np.linalg.norm(point - touchloc_bl[0])
        print '===================================================================='
        print '===================================================================='
        #TODO assure that reaching motion did touch the point that we intended to touch.
        rospy.loginfo('!! Touched point is %.3f m away from observed point !!' % dist)
        print '===================================================================='
        print '===================================================================='

        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            self.behaviors.movement.move_absolute(self.start_location, stop='accel', \
                    pressure=press_contact_pressure)
            #raise TaskError(error_msg)
            return False, None

        rospy.loginfo('>>>> PRESSING')

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
        rospy.loginfo('>>>> MOVING BACK')
        #self.behaviors.movement.set_movement_mode_cart()
        r1 = self.behaviors.movement.move_relative_gripper(np.matrix([-.03, 0., 0.]).T, \
                stop='none', pressure=press_contact_pressure)
        if r1 != None:
            rospy.loginfo('moving back failed due to "%s"' % r1)
            return False, None

        rospy.loginfo('>>>> RESETING')
        r2, pos_error = self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
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

    def drawer(self, point):
        linear_movement = self.behaviors.movement
        linear_movement.gripper_open()
        #pdb.set_trace()
        linear_movement.move_absolute((self.start_location[0], np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0))))
        success, reason, touchloc_bl = self.behaviors.reach(point, 300, np.matrix([0.0, 0, 0]).T, 
                             reach_direction=np.matrix([0.1, 0, 0]).T, 
                             orientation=np.matrix(tr.quaternion_from_euler(np.radians(90.), 0, 0)))

        if not success:
            error_msg = 'Reach failed due to "%s"' % reason
            rospy.loginfo(error_msg)
            rospy.loginfo('Failure recovery: moving back')
            #linear_movement.set_movement_mode_cart()
            linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='pressure_accel', pressure=300)
            #self.behaviors.movement.move_absolute(self.start_location, stop='accel', pressure=300)

        #grasp
        linear_movement.gripper_close()
        #linear_movement.set_movement_mode_cart()
        linear_movement.move_relative_gripper(np.matrix([-.25,0,0]).T, stop='none', pressure=1000)

        #linear_movement.set_movement_mode_cart()
        linear_movement.gripper_open()
        time.sleep(1)

        linear_movement.pressure_listener.rezero()
        #pdb.set_trace()
        #linear_movement.move_relative_gripper(np.matrix([-.15, 0, 0]).T, stop='pressure_accel', pressure=300)
        linear_movement.move_relative_base(np.matrix([-.2, .3, 0.3]).T, stop='pressure_accel', pressure=300)
        #linear_movement.set_movement_mode_ik()

    ##
    # Drive using within a dist_far distance of point_bl
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

    ##
    # Drive so that we are perpendicular to a wall at point_bl (radii voi_radius) 
    # stop at dist_approach
    def approach_perpendicular_to_surface(self, point_bl, voi_radius, dist_approach):
        #return 3
        #TODO: Turn to face point
        #TODO: make this scan around point instead of total scan of env
        #determine normal
        #pdb.set_trace()
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
        #pdb.set_trace()
        self.turn_to_point(point_bl, block=False)
        time.sleep(2.)

        return rvalue
        #ang = math.atan2(point_bl[1,0], point_bl[0,0])
        #self.robot.base.turn_by(ang, block=True)
        #pdb.set_trace()

    def approach_location(self, point_bl, coarse_stop, fine_stop, voi_radius=.2):
        #return
        point_dist = np.linalg.norm(point_bl[0:2,0])
        rospy.loginfo('approach_location: Point is %.3f away.' % point_dist)
        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)

        dist_theshold = coarse_stop + .1
        if point_dist > dist_theshold:
            rospy.loginfo('approach_location: Point is greater than %.1f m away (%.3f).  Driving closer.' % (dist_theshold, point_dist))
            rospy.loginfo('approach_location: CLICKED on point_bl ' + str(point_bl.T))

            ret = self.drive_approach_behavior(point_bl, dist_far=coarse_stop)
            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
            if ret != 3:
                dist_end = np.linalg.norm(point_bl_t1[0:2,0])
                if dist_end > dist_theshold:
                    rospy.logerr('approach_location: drive_approach_behavior failed! %.3f' % dist_end)
                    self.robot.sound.say("I am unable to navigate to that location")
                    return False

            ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=voi_radius, dist_approach=fine_stop)
            if ret != 3:
                rospy.logerr('approach_location: approach_perpendicular_to_surface failed!')
                return False

            self.robot.sound.say('done')
            rospy.loginfo('approach_location: DONE DRIVING!')
            return True
        else:
            return False

    def turn_to_point(self, point_bl, block=True):
        ang = math.atan2(point_bl[1,0], point_bl[0,0])
        rospy.loginfo('turn_to_point: turning by %.2f deg' % math.degrees(ang))
        #pdb.set_trace()
        self.robot.base.turn_by(-ang, block=block, overturn=True)

    #def load_classifier(self, classifier_name, data_file_name):
    #    self.learners[classifier_name] = ipa.InterestPointPerception(classifier_name, 
    #            data_file_name, self.tf_listener)

    def stationary_light_switch_behavior(self, point_bl):
        while True:
            print 'Enter a command u(ntuck), s(tart), l(ight), t(uck), e(x)it.'
            a = raw_input()
            
            if a == 'u': 
                self.untuck()

            if a == 's':
                self.behaviors.movement.pressure_listener.rezero()
                self.behaviors.movement.set_movement_mode_cart()
                self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
                #pdb.set_trace()
                #TODO: set start location to be some set joint angles

            if a == 'l':
                point_offset = np.matrix([0, 0, 0.03]).T
                success, _ = self.light_switch1(point_bl, point_offset=point_offset, \
                        press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                        visual_change_thres=.03)

            if a == 't':
                self.tuck()

            if a == 'x':
                break

    def location_activated_behaviors(self, point_bl, stored_point=False):
        driving_param = {'light_switch': {'coarse': .7, 'fine': .5, 'voi': .2},
                         'drawer':       {'coarse': .7, 'fine': .7, 'voi': .2}}

        map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        point_map = tfu.transform_points(map_T_base_link, point_bl)
        matches = self.find_close_by_points(point_map)

        if len(matches) > 0:
            #pdb.set_trace()
            ldata = self.location_data[self.location_labels[matches[0]]]
            task = ldata['task']
            rospy.loginfo('Found closeby location %s' % str(ldata))
        else:
            rospy.loginfo( 'No location matches found. Please enter location type:')
            for i, k in enumerate(driving_param.keys()):
                rospy.loginfo(' %d %s' %(i,k))
            task_number = raw_input()
            task = driving_param.keys()[int(task_number)]

        self.robot.sound.say('task %s' % task.replace('_', ' '))
        rospy.loginfo('Task is %s' % task)
        if self.approach_location(point_bl, 
                coarse_stop=driving_param[task]['coarse'], 
                fine_stop=driving_param[task]['fine'], 
                voi_radius=driving_param[task]['voi']):
            return

        else:
            ret = self.approach_perpendicular_to_surface(point_bl, 
                    voi_radius=driving_param[task]['voi'], 
                    dist_approach=driving_param[task]['fine'])

            if ret != 3:
                rospy.logerr('location_activated_behaviors: approach_perpendicular_to_surface failed!')
                return

            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
            try:
                self.untuck()
                self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
                self.behaviors.movement.pressure_listener.rezero()

                if task == 'light_switch':
                    #self.location_add(perturbed_map, task)
                    # TODO: what happens when we first encounter this location?! experiment n times to create dataset?
                    self.practice(point_bl_t1, 
                            ft.partial(self.light_switch1, 
                                point_offset=np.matrix([0,0,.03]).T,
                                press_contact_pressure=300,
                                move_back_distance=np.matrix([-.0075,0,0]).T,
                                press_pressure=3500,
                                press_distance=np.matrix([0,0,-.15]).T,
                                visual_change_thres=.03), 
                            'light_switch')
                    self.tuck()

                    if False: #Old branch where we retry blindly
                        MAX_RETRIES = 15
                        rospy.loginfo('location_activated_behaviors: go_home_pose')
                        #self.go_to_home_pose()
                        self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
                        gaussian = pr.Gaussian(np.matrix([ 0,      0,      0.]).T, \
                                               np.matrix([[1.,     0,      0], \
                                                          [0, .02**2,      0], \
                                                          [0,      0, .02**2]]))
                        retry_count = 0
                        success = False
                        gaussian_noise = np.matrix([0, 0, 0.0]).T
                        point_offset = np.matrix([0, 0, 0.03]).T
                        while not success:
                            perturbation = gaussian_noise
                            perturbed_point_bl = point_bl_t1 + perturbation
                            success, _ = self.light_switch1(perturbed_point_bl, point_offset=point_offset, \
                                    press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
                                    press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                                    visual_change_thres=.03)
                            gaussian_noise = gaussian.sample()
                            gaussian_noise[0,0] = 0
                            retry_count = retry_count + 1 

                            if retry_count > MAX_RETRIES:
                                self.robot.sound.say('giving up tried %d times already' % MAX_RETRIES)
                                break
                            elif not success:
                                 self.robot.sound.say('retrying')

                        if success:
                            self.robot.sound.say('successful!')

                            if not stored_point or retry_count > 1:
                                map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
                                perturbed_map = tfu.transform_points(map_T_base_link, perturbed_point_bl)
                                self.location_add(perturbed_map, task)
                                self.robot.sound.say('recorded point')

                            #if retry_count > 1:
                            #    if not self.add_perturbation_to_location(point_map, perturbation):
                            #        self.robot.sound.say('unable to add perturbation to database! please fix')
                        self.tuck()

    
                if task == 'drawer':
                    self.drawer(point_bl_t1)
                    self.tuck()
                    self.robot.sound.say('done')
                    self.location_add(point_map, task)
                    
                    #except lm.RobotSafetyError, e:
                    #    rospy.loginfo('location_activated_behaviors: Caught a robot safety exception "%s"' % str(e.parameter))
                    #    #self.behaviors.movement.move_absolute(self.start_location, stop='accel')

            except lm.RobotSafetyError, e:
                rospy.loginfo('location_activated_behaviors: Caught a robot safety exception "%s"' % str(e.parameter))
                self.behaviors.movement.move_absolute(self.start_location, stop='accel')
    
            except TaskError, e:
                rospy.loginfo('location_activated_behaviors: TaskError: %s' % str(e.parameter))
            rospy.loginfo('location_activated_behaviors: DONE MANIPULATION!')
            self.robot.sound.say('done')

    def click_cb(self, point_bl):
        #point_bl = np.matrix([ 0.68509375,  0.06559023,  1.22422832]).T
        #self.stationary_light_switch_behavior(point_bl)
        #mode = 'autonomous'
        #mode = 'light switch'
        mode = 'live_label'
        if point_bl!= None:
            if mode == 'live_label':
                #self.execute_behavior(point_bl, 
                light_switch_beh = ft.partial(self.light_switch1, 
                                        point_offset=np.matrix([0,0,.03]).T,
                                        press_contact_pressure=300,
                                        move_back_distance=np.matrix([-.0075,0,0]).T,
                                        press_pressure=3500,
                                        press_distance=np.matrix([0,0,-.15]).T,
                                        visual_change_thres=.03)
                point_map = tfu.transform_points(tfu.transform('map', 'base_link', self.tf_listener), point_bl)

                while not rospy.is_shutdown():
                    point_bl_cur = tfu.transform_points(tfu.transform('base_link', 'map', self.tf_listener), point_map)
                    self.execute_behavior(point_bl_cur, light_switch_beh, 'light_switch')

            if mode == 'capture data':
                self.robot.head.look_at(point_bl, 'base_link', True)
                self.robot.sound.say("taking a scan")
                #self.record_perceptual_data(point_bl)
                #pdb.set_trace()
                rdict = self.kinect_features.kinect_listener.read()
                self.record_perceptual_data_kinect(point_bl, rdict)
                self.robot.sound.say("saved scan")

            if mode == 'light switch':
                point_offset = np.matrix([0, 0, 0.03]).T
                success, _ = self.light_switch1(point_bl, point_offset=point_offset, \
                        press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
                        press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                        visual_change_thres=.03)

                self.behaviors.movement.pressure_listener.rezero()
                self.behaviors.movement.set_movement_mode_cart()
                self.behaviors.movement.move_absolute(self.start_location, stop='pressure')


            if mode == 'autonomous learn':
                def light_behavior(point):
                    point_offset = np.matrix([0, 0, 0.03]).T
                    success, _ = self.light_switch1(point, point_offset=point_offset, \
                                    press_contact_pressure=300, move_back_distance=np.matrix([-.0075,0,0]).T,\
                                    press_pressure=3500, press_distance=np.matrix([0,0,-.15]).T, \
                                    visual_change_thres=.03)
                    if success:
                        return 1.0
                    else:
                        return 0.0

                self.untuck()
                self.behaviors.movement.move_absolute(self.start_location, stop='pressure')
                self.behaviors.movement.pressure_listener.rezero()
                self.autonomous_learn(point_bl, light_behavior, 'light_switch')

            if mode == 'location activated':
                self.location_activated_behaviors(point_bl)

        elif mode == 'location activated':
            if len(self.location_centers) < 1:
                return
            rospy.loginfo('click_cb: double clicked but no 3d point given')
            rospy.loginfo('click_cb: will use the last successful location given')
            base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
            point_bl = tfu.transform_points(base_link_T_map, self.location_centers[-1])
            rospy.loginfo('click_cb: using ' + str(self.location_centers[-1].T))
            self.location_activated_behaviors(point_bl, stored_point=True)


    def find_close_by_points(self, point_map):
        if self.locations_tree != None:
            close_by_locs = self.locations_tree.query_ball_point(np.array(point_map.T), self.LOCATION_ADD_RADIUS)[0]
            return close_by_locs
        else:
            return []

    def find_close_by_points_match_task(self, point_map, task):
        matches = self.find_close_by_points(point_map)
        task_matches = []
        for m in matches:
            idx = self.location_labels[m]
            ldata = self.location_data[idx]
            if ldata['task'] == task:
                task_matches.append(m)
        return task_matches

    def location_add(self, point_map, task, data):
        close_by_locs = self.find_close_by_points_match_task(point_map, task)
        if len(close_by_locs) == 0:
            rospy.loginfo('location_add: point not close to any existing location. creating new record.')
            self.location_data.append({
                'task': task, 
                'center': point_map, 
                'perceptual_dataset': None,
                'points':[point_map]})
            self.location_centers.append(point_map)
            self.location_labels.append(len(self.location_data) - 1)
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))
        else:
            #If close by locations found then add to points list and update center
            location_idx = self.location_labels[close_by_locs[0]]
            ldata = self.location_data[location_idx]

            rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
            ldata['points'].append(point_map)
            ldata['center'] = np.column_stack(ldata['points']).mean(1)
            self.location_centers[location_idx] = ldata['center']
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

        ut.save_pickle(self.location_data, self.saved_locations_fname)
        rospy.loginfo('location_add: saved point in map.')


    def location_add(self, point_map, task):
        close_by_locs = self.find_close_by_points_match_task(point_map, task)
        if len(close_by_locs) == 0:
            rospy.loginfo('location_add: point not close to any existing location. creating new record.')
            self.location_data.append({
                'task': task, 
                'center': point_map, 
                'points':[point_map]})
            self.location_centers.append(point_map)
            self.location_labels.append(len(self.location_data) - 1)
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

        else:
            #If close by locations found then add to points list and update center
            location_idx = self.location_labels[close_by_locs[0]]
            ldata = self.location_data[location_idx]

            rospy.loginfo('location_add: point close to %d at %s.' % (location_idx, str(ldata['center'].T)))
            ldata['points'].append(point_map)
            ldata['center'] = np.column_stack(ldata['points']).mean(1)
            self.location_centers[location_idx] = ldata['center']
            self.locations_tree = sp.KDTree(np.array(np.column_stack(self.location_centers).T))

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

    def record_perceptual_data_laser_scanner(self, point_touched_bl):
        #what position should the robot be in?
        #set arms to non-occluding pose

        #record region around the finger where you touched
        rospy.loginfo('Getting laser scan.')
        points = []
        for i in range(3):
            rospy.loginfo('scan %d' % i)
            points.append(self.laser_scan.scan(math.radians(180.), math.radians(-180.), 20./3.))

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

        data_pkl = {'touch_point': point_touched_bl,
                    'points_laser': points,
                    'laser_T_bl': laser_T_bl, 
                    'pro_T_bl': pro_T_bl,

                    'high_res': prosilica_name,
                    'prosilica_cal': self.prosilica_cal, 

                    'left_image': left_name,
                    'left_cal': self.left_cal,

                    'right_image': right_name,
                    'right_cal': self.right_cal}
                    #'point_touched': point_touched_bl}
                    

        ut.save_pickle(data_pkl, pickle_fname)
        print 'Recorded to', pickle_fname

    #def record_processed_data_kinect2(self, point3d_bl, kinect_fea):
    #    instances, locs2d_image, locs3d_bl, image = kinect_fea #self.kinect_features.read(point3d_bl)
    #    #rospy.loginfo('Getting a kinect reading')

    #    tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
    #    kimage_name = '%s_highres.png' % tstring
    #    cv.SaveImage(kimage_name, kimage)

    #    preprocessed_dict = {'instances': instances,
    #                         'points2d': locs2d_image,
    #                         'points3d': locs3d_bl,
    #                         'image': kimage_name,
    #                         'labels': labels,
    #                         'sizes': feature_extractor.sizes}


        #self.kinect_features.read(point3d_bl)
        #rdict = self.kinect_listener.read()
        #kimage = rdict['image']
        #rospy.loginfo('Waiting for calibration.')
        #while self.kinect_cal.has_msg == False:
        #    time.sleep(.1)

        #which frames?
        #rospy.loginfo('Getting transforms.')
        #k_T_bl = tfu.transform('openni_rgb_optical_frame', '/base_link', self.tf_listener)
        #tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
        #kimage_name = '%s_highres.png' % tstring
        #rospy.loginfo('Saving images (basename %s)' % tstring)
        #cv.SaveImage(kimage_name, kimage)
        #rospy.loginfo('Saving pickles')
        #pickle_fname = '%s_interest_point_dataset.pkl' % tstring   

        #data_pkl = {'touch_point': point3d_bl,
        #            'points3d': rdict['points3d'],
        #            'image': kimage_name,
        #            'cal': self.prosilica_cal, 
        #            'k_T_bl': k_T_bl}
                    #'point_touched': point3d_bl}

        #ut.save_pickle(data_pkl, pickle_fname)
        #print 'Recorded to', pickle_fname

    def record_perceptual_data_kinect(self, point3d_bl, rdict=None):
        rospy.loginfo('saving dataset..')
        #self.kinect_features.read(point3d_bl)
        if rdict == None:
            rospy.loginfo('Getting a kinect reading')
            rdict = self.kinect_listener.read()
        kimage = rdict['image']
        rospy.loginfo('Waiting for calibration.')
        while self.kinect_features.cal.has_msg == False:
            time.sleep(.1)

        #which frames?
        rospy.loginfo('Getting transforms.')
        k_T_bl = tfu.transform('openni_rgb_optical_frame', '/base_link', self.tf_listener)
        tstring = time.strftime('%A_%m_%d_%Y_%I:%M%p')
        kimage_name = '%s_highres.png' % tstring
        rospy.loginfo('Saving images (basename %s)' % tstring)
        cv.SaveImage(kimage_name, kimage)
        rospy.loginfo('Saving pickles')
        pickle_fname = '%s_interest_point_dataset.pkl' % tstring   

        data_pkl = {'touch_point': point3d_bl,
                    'points3d': rdict['points3d'],
                    'image': kimage_name,
                    'cal': self.kinect_features.cal, 
                    'k_T_bl': k_T_bl}
                    #'point_touched': point3d_bl}

        ut.save_pickle(data_pkl, pickle_fname)
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

    ##
    # The behavior can be making a service call to a GUI that ask users how to label
    def practice(self, point3d_bl, behavior, learner_name):
        instances, locs2d_image, locs3d_bl, image, raw_dict = self.kinect_features.read(point3d_bl)
        self.record_perceptual_data_kinect(point3d_bl, raw_dict)
        converged = False
        indices_added = []
        rec_params = self.kinect_features.rec_params

        while not converged:
            #Find remaining instances
            remaining_pt_indices = inverse_indices(indices_added, instances.shape[1])
            remaining_instances = instances[:, remaining_pt_indices]
            ridx, selected_dist, converged = self.learner.select_next_instances_no_terminate(remaining_instances)
            selected_idx = remaining_pt_indices[ridx]
            indices_added.append(selected_idx)

            #Get label
            label = behavior(locs3d_bl[:, selected_idx])
            #self.location_add(locs3e_bl[:, selected_idx], leraner_name, 

            #Retrain
            #TODO FIX THIS FUNCTION IT NO WORKY WORKY
            #self.learners[learner_name]['dataset'].add(instances[:,selected_idx], label, 
            #        locs2d_image[:, selected_idx], locs3d_bl[:, selected_idx])
            self.train(self.learners[learner_name]['dataset'], learner_name)

            #Classify
            predictions = np.matrix(self.learners[learner_name]['learner'].classify(instances))

            #draw
            img = cv.CloneMat(image)
            self.draw_dots_nstuff(img, locs3d_image, predictions)

            #publish
            self.img_pub.publish(img)

    def execute_behavior(self, point3d_bl, behavior, learner_name):
        instances, locs2d_image, locs3d_bl, image, _ = self.kinect_features.read(point3d_bl)
        rec_params = self.kinect_features.rec_params
        predictions = np.matrix(self.learners[learner_name]['learner'].classify(instances))

        #select the positive predictions
        #locs3d_bl[:, np.where(predictions == r3d.POSITIVE)[1].A1.tolist()]
        #behavior(infered_point)
        #get the median

        #draw
        img = cv.CloneMat(image)
        self.draw_dots_nstuff(img, locs2d_image, predictions)

        #publish
        print 'publishing.'
        self.img_pub.publish(img)

    
    def autonomous_learn(self, point3d_bl, behavior, object_name): 
        # We learn, but must moderate between spatial cues and requirements of
        # the learner. Spatial cue is a heuristic that can guide to positive
        # examples. Learning heuristic reduces the number of experiments to
        # perform given that we know that we are generally *not* successful
        # (assume that this procedure launches only during non mission critial circumstances).
        # So in the case where we're actively learning we're going to ignore the spatial heuristic.
        # Well... can we incorporate distance to the selected 3d point as a feature?
        # ah!
        learn_manager = self.learners[object_name]
        #scan and extract features
        self.robot.head.look_at(point3d_bl, 'base_link', True)
        learn_manager.scan(point3d_bl)
        gaussian = pr.Gaussian(np.matrix([ 0,      0,     0.]).T, \
                               np.matrix([[1.,     0,      0], \
                                          [0, .02**2,      0], \
                                          [0,      0, .02**2]]))

        #pdb.set_trace()
        gaussian_noise = np.matrix([0,0,0.]).T
        while not learn_manager.is_ready():
             pi = point3d_bl + gaussian_noise
             label = behavior(pi)
             #look at point, then try to add again
             if not learn_manager.add_example(pi, np.matrix([label])):
                 rospy.logerr('Unable to extract features from point %s' % str(pi.T))
                 continue
             learn_manager.train()
             learn_manager.draw_and_send()
             gaussian_noise = gaussian.sample()
             gaussian_noise[0,0] = 0

        #Acquire data
        #Given image, cloud, 3d point ask, extract features.
        #while no_interruptions and stopping_criteria_not_reached
        #    maximally_informative_point = get maximally informative point
        #    label = behavior(maximally_informative_point)
        #    retrain!
        converged = False
        while not converged:
            indices, dists = learn_manager.select_next_instances(1)
            if idx != None:
                pt2d = learn_manager.points2d[:, indices[0]]
                pt3d = learn_manager.points3d[:, indices[0]]
                label = behavior(pt3d)
                #learn_manager.add_example(pt3d, np.matrix([label]), pt2d)
                if not learn_manager.add_example(pi, np.matrix([label])):
                    rospy.logerr('Unable to extract features from point %s' % str(pi.T))
                    continue
                learn_manager.train()
                learn_manager.draw_and_send()
            else:
                converged = True



if __name__ == '__main__':
    l = ApplicationBehaviors()
    l.run()





















































        #def __init__(self, object_name, labeled_data_fname, tf_listener):
        #make learner
        #learner = SVMActiveLearnerApp()
        #labeled_light_switch_dataset = ut.load_pickle(data_file_name)
        #learner.train(labeled_light_switch_dataset, 
        #              labeled_light_switch_dataset.sizes['intensity']
        #              self.params.variance_keep)
        #self.learners[classifier_name] = learner


    #def locate_light_switch(self):
    #    #capture data
    #    pointcloud_msg = self.laser_scan.scan(math.radians(180.), math.radians(-180.), 20.)
    #    prosilica_image = self.prosilica.get_frame() #TODO check if this is a cvmat
    #    while self.prosilica_cal.has_msg == False:
    #        time.sleep(.1)

    #    #preprocess 
    #    ic_data = IntensityCloudData(pointcloud_msg, prosilica_image, 
    #                    tfu.transform('/high_def_optical_frame', '/base_link', self.tf_listener), 
    #                    self.prosilica_cal,                                                       
    #                    r3d.Recognize3DParam())
    #    instances = ic_data.extract_vectorized_features()

    #    results = []
    #    for i in range(instances.shape[1]):
    #        nlabel = self.learners['light_switch'].classify(instances[:, i])
    #        results.append(nlabel)

    #    results = np.matrix(results)
    #    positive_indices = np.where(results == r3d.POSITIVE)[1]

    #    #want 3d location of each instance
    #    positive_points_3d = ic_data.sampled_points[:, positive_indices]

    #    #return a random point for now
    #    rindex = np.random.randint(0, len(positive_indices))
    #    return positive_points_3d[:,rindex]


    #def add_perturbation_to_location(self, point_map, perturbation):
    #    locs = self.find_close_by_points(point_map)
    #    if locs != None:
    #        location = self.location_data[self.location_labels(locs[0])]
    #        if not location.has_key('perturbation'):
    #            location['perturbation'] = []
    #        location['perturbation'].append(perturbation)
    #        return True
    #    return False
















                #self.go_to_home_pose()
                #print '>>>> POINT IS', point_bl_t1.T
                #point_bl_t1 = np.matrix([[ 0.73846737,  0.07182931,  0.55951065]]).T
        #DIST_THRESHOLD = .8 for lightswitch
        #DIST_THRESHOLD = .85 #for drawers
        #DIST_APPROACH = .5
        #COARSE_STOP = .7
        #FINE_STOP = .7
        #VOI_RADIUS = .2

        #point_dist = np.linalg.norm(point_bl_t0[0:2,0])
        #rospy.loginfo('run_behaviors: Point is %.3f away.' % point_dist)
        #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        #point_map = tfu.transform_points(map_T_base_link, point_bl_t0)

        #if point_dist > DIST_THRESHOLD:
        #    rospy.loginfo('run_behaviors: Point is greater than %.1f m away (%.3f).  Driving closer.' % (DIST_THRESHOLD, point_dist))
        #    ##self.turn_to_point(point_bl_t0)
        #    rospy.loginfo( 'run_behaviors: CLICKED on point_bl ' + str(point_bl_t0.T))

        #    ret = self.drive_approach_behavior(point_bl_t0, dist_far=COARSE_STOP)
        #    if ret != 3:
        #        base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        #        point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)
        #        dist_end = np.linalg.norm(point_bl_t1[0:2,0])
        #        if dist_end > DIST_THRESHOLD:
        #            rospy.logerr('run_behaviors: drive_approach_behavior failed! %.3f' % dist_end)
        #            self.robot.sound.say("I am unable to navigate to that location")
        #            return

        #    base_link_T_map = tfu.transform('base_link', 'map', self.tf_listener)
        #    point_bl_t1 = tfu.transform_points(base_link_T_map, point_map)

        #    ret = self.approach_perpendicular_to_surface(point_bl_t1, voi_radius=VOI_RADIUS, dist_approach=FINE_STOP)
        #    if ret != 3:
        #        rospy.logerr('run_behaviors: approach_perpendicular_to_surface failed!')
        #        return

        #    #map_T_base_link = tfu.transform('map', 'base_link', self.tf_listener)
        #    #point_bl_t2 = tfu.transform_points(base_link_T_map, point_map)
        #    self.robot.sound.say('done')
        #    rospy.loginfo('run_behaviors: DONE DRIVING!')
        #elif False:





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

    #    #pdb.set_trace()
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


