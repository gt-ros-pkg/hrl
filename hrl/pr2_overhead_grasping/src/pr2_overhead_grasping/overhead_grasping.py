#! /usr/bin/python

import numpy as np, math
import sys
import os
from threading import RLock
import threading
import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import yaml
sys.path.append("/usr/lib/python2.6/site-packages")
import cv
import arff

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

import actionlib
import tf

from std_msgs.msg import String
from geometry_msgs.msg import  Point, Pose, Quaternion, PoseStamped, PointStamped

import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult

import hrl_lib.util
from hrl_lib.rutils import GenericListener
import hrl_lib.viz as viz
from hrl_lib import tf_utils
from hrl_lib.keyboard_input import KeyboardInput
from hrl_lib.transforms import rotX, rotY, rotZ
from tf.transformations import *
from visualization_msgs.msg import MarkerArray, Marker
import dynamic_reconfigure.client
from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC, CURSOR_TOPIC, MOUSE_R_CLICK_TOPIC, MOUSE_R_DOUBLE_CLICK_TOPIC
import face_detector.msg
from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal, SingleJointPositionAction, SingleJointPositionGoal

from grasper import OverheadGrasper
from perception_monitor import ArmPerceptionMonitor
from helpers import *
from pr2_overhead_grasping.msg import *

class OverheadGraspManager():
    def __init__(self, arm, active = False, ki = None, use_classifiers=False):
        if ki is None:
            ki = KeyboardInput()
            
        self.arm = arm
        self.ki = ki
        self.oger = OverheadGrasper(arm, active=active, ki=ki, 
                                    use_classifiers=use_classifiers)

    ##
    # Checks to see if anything is in the gripper currently (gripper is not fully closed).
    def is_obj_in_gripper(self):
        return self.oger.cm.get_current_gripper_opening() > 0.01

    ##
    # Close gripper. Will continue applying force.
    def close_gripper(self, blocking = False):
        self.oger.cm.command_gripper(0.0, 30.0, False)
        if blocking:
            self.oger.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    ##
    # Open gripper fully.
    def open_gripper(self, blocking = False):
        self.oger.cm.command_gripper(0.08, -1.0, False)
        if blocking:
            self.oger.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    ##
    # Uses the object_detection service detect objects on the table. Returns a list
    # of pairs [ [ x, y, z], rot ] which represent the centers and z orientation of
    # each object detected on the table.
    def detect_tabletop_objects(self):
        DETECT_ERROR = 0.
        cbbf = ClusterBoundingBoxFinder(tf_listener=self.oger.cm.tf_listener)
        object_detector = rospy.ServiceProxy("/object_detection", TabletopDetection)
        detects = object_detector(True, False).detection
        object_detector.close()
        if detects.result != 4:
            err("Detection failed (err %d)" % (detects.result))
            return []
        table_z = detects.table.pose.pose.position.z
        objects = []
        for cluster in detects.clusters:
            (object_points, 
             object_bounding_box_dims, 
             object_bounding_box, 
             object_to_base_link_frame, 
             object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
            # log("bounding box:", object_bounding_box)
            (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
            angs = euler_from_quaternion(object_quat)
            log("angs:", angs)
            # position is half of height
            object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
            log("pos:", object_pos)
            log("table_z:", table_z)
            objects += [[object_pos, angs[2]]]
        return objects

    ##
    # Grasps the object defined by obj, a pair of location and rotation created by
    # detect_tabletop_objects. If collide is False, grasping will not detect collisions.
    def grasp_object(self, obj, use_z=True, is_place=False):
        if not use_z:
            self.oger.perform_grasp(obj[0][0], obj[0][1], is_place=is_place,
                                 gripper_rot=obj[1])
        else:
            self.oger.perform_grasp(obj[0][0], obj[0][1], z=obj[0][2], 
                                    is_place=is_place, gripper_rot=obj[1])

    def get_grasp_loc(self, obj):
        return obj[0][0], obj[0][1], obj[1], obj[0][2]

    ##
    # Given an (x, y) location on a table, grasp the closest object detected.
    # If repeat is True, will keep trying if the grasp fails.
    def grasp_closest_object(self, x, y, repeat=True):
        def dist(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2

        grasped = False
        num_tries = 0

        self.point_head([x,y,-0.2])

        while not grasped and num_tries < 4:
            log("Detect in")
            self.change_projector_mode(True)
            rospy.sleep(0.6)
            detect_tries = 0
            objects = None
            while (objects is None or len(objects) == 0):
                objects = self.detect_tabletop_objects()
                rospy.sleep(0.6)
                detect_tries += 1
                if detect_tries == 3 and (objects is None or len(objects) == 0):
                    err("Cannot detect any objects")
                    return None
            log("Detect out")
            if len(objects) > 0:
                obj = min(objects, key=dist)

                # Get better look
                if True:
                    self.point_head(obj[0])
                    rospy.sleep(0.2)
                    log("Detect2 in")
                    objects = self.detect_tabletop_objects()
                    log("Detect2 out")
                    obj = min(objects, key=dist)

                self.change_projector_mode(False)
                self.grasp_object(obj, use_z=use_z)
                rospy.sleep(0.2)
                grasped = self.is_obj_in_gripper()
                if repeat and not grasped:
                    num_tries += 1
                    continue
                if grasped:
                    log("Grasp success!")
                    return obj
                else:
                    err("Grasp failure")
                    return None
            else:
                log("No objects near point")
                return None

    ##
    # Given an (x, y) location on a table, grasp the closest object detected.
    # If repeat is True, will keep trying if the grasp fails.
    def detect_closest_object(self, x, y, repeat=True):
        def dist(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2

        grasped = False
        num_tries = 0

        self.point_head([x,y,-0.2])

        while not grasped and num_tries < 4:
            log("Detect in")
            self.change_projector_mode(True)
            rospy.sleep(0.6)
            detect_tries = 0
            objects = None
            while (objects is None or len(objects) == 0):
                objects = self.detect_tabletop_objects()
                rospy.sleep(0.6)
                detect_tries += 1
                if detect_tries == 3 and (objects is None or len(objects) == 0):
                    err("Cannot detect any objects")
                    return None
            log("Detect out")
            if len(objects) > 0:
                obj = min(objects, key=dist)

                # Get better look
                self.point_head(obj[0])
                rospy.sleep(0.2)
                log("Detect2 in")
                objects = self.detect_tabletop_objects()
                log("Detect2 out")
                obj = min(objects, key=dist)

                self.change_projector_mode(False)
                return obj
            else:
                log("No objects near point")
                return None
        return None

    ##
    # Object swap demo.
    # TODO Does this still work?
    def grasp_demo(self):
        grasps, avg_lists = [], []
        self.grasp_closest_object(0.45, 0.1)
        grasps += [grasp]
        avg_lists += [avg_list]
        obj1pl = [[obj1[0][0], obj1[0][1] - 0.18], obj1[1]]
        self.grasp_object(obj1pl, is_place=True)
        grasps += [grasp]
        avg_lists += [avg_list]
        self.grasp_closest_object(0.7, 0.1)
        grasps += [grasp]
        avg_lists += [avg_list]
        self.grasp_object(obj1, is_place=True)
        grasps += [grasp]
        avg_lists += [avg_list]
        self.grasp_closest_object(obj1pl[0][0], obj1pl[0][1])
        grasps += [grasp]
        avg_lists += [avg_list]
        self.grasp_object(obj2, is_place=True)
        grasps += [grasp]
        avg_lists += [avg_list]
        display_grasp_data(grasps[0:3], "accelerometer", monitor_data=avg_lists[0:3])
        display_grasp_data(grasps[3:-1], "accelerometer", monitor_data=avg_lists[3:-1])

    ##
    # Waits for a laser double click.  Must have laser_interface nodes running.
    # The waiting loop has two modes.  In the inital mode, the loop will wait until
    # a cursor3d message is posted and the mouse is double clicked delay_time apart.
    # The function then breaks and returns the last cursor3d pt and False.  The second
    # mode has the same functionality and is initialized by double clicking the right
    # button.  When beginning the second mode the head will be pointed up.  To exit,
    # simply double click the right button again and the robot will look back down.
    # The second mode will return a True as the second argument.
    # TODO Fix face detect code.
    def get_laser_dclick(self, tf_listener, frame = "/torso_lift_link", delay_time = 3.0):
        c3d_lis = GenericListener("c3dlisnode", PointStamped, self.oger.CURSOR_TOPIC, 10.0)
        dc_lis = GenericListener("dclisnode", String, self.oger.MOUSE_DOUBLE_CLICK_TOPIC, 10.0)
        rdc_lis = GenericListener("rdclisnode", String, self.oger.MOUSE_R_DOUBLE_CLICK_TOPIC, 10.0)
        # face_detect_cli = actionlib.SimpleActionClient('face_detector_action', face_detector.msg.FaceDetectorAction)
        log("Waiting for laser click...")
        while not rospy.is_shutdown():
            if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                log("Double click heard")
                msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
                log(msg)
                if msg is not None:
                    # @TODO contain this code?
                    if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
                        now = rospy.Time.now()
                        tf_listener.waitForTransform(msg.header.frame_id, frame, 
                                                     now, rospy.Duration(4.0))
                        tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
                        tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
                        pt = tfmat[0:3,3]
                        #(trans, rot) = tf_listener.lookupTransform(msg.header.frame_id, frame, now)
                        #pt = [msg.point.x - trans[0], msg.point.y - trans[1], msg.point.z - trans[2]]
                        log("pt", pt)
                        return [pt[0,0], pt[1,0], pt[2,0]], False

            if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                log("Right double click heard")
                log("Double click on person to hand-off")
                self.point_head([1.0, 0.0, 0.05], block = True)

                # begin waiting for person selection
                while not rospy.is_shutdown():
                    if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                        log("Double click heard")
                        msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
                        log(msg)
                        if msg is not None:
                            if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
                                now = rospy.Time.now()
                                tf_listener.waitForTransform(msg.header.frame_id, frame, 
                                                             now, rospy.Duration(4.0))
                                tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
                                tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
                                lspt = tfmat[0:3,3]
                                lspt = [ lspt[0,0], lspt[1,0], lspt[2,0]]
                                log("pt", lspt)
                    
                                # log("Waiting for face detection server")
                                # face_detect_cli.wait_for_server()
                                # face_req = face_detector.msg.FaceDetectorGoal()
                                # face_detect_cli.send_goal(face_req)
                                # face_detect_cli.wait_for_result()
                                # face_poss = face_detect_cli.get_result().face_positions
                                # log("Got face results:", face_poss)

                                # # transform face positions to our frame
                                # face_pts = []
                                # for face_pos in face_poss:
                                #     now = rospy.Time.now()
                                #     tf_listener.waitForTransform(face_pos.header.frame_id, frame, 
                                #                                  now, rospy.Duration(4.0))
                                #     tfmat = tf_utils.transform(frame, face_pos.header.frame_id, tf_listener)
                                #     tfmat *= np.mat([[face_pos.pos.x], [face_pos.pos.y], [face_pos.pos.z], [1.0]])
                                #     fcpt = tfmat[0:3,3]
                                #     fcpt = [ fcpt[0,0], fcpt[1,0], fcpt[2,0]]
                                #     face_pts.append(fcpt)

                                # log("Face locations", face_pts)
                                # for face_pt in face_pts:
                                #     dist = np.sqrt((face_pt[0] - lspt[0])**2 + (face_pt[1] - lspt[1])**2)
                                #     if dist < 0.35 and face_pt[2] > 0.3:
                                #         return face_pt, True
                                lspt[2] += 0.2
                                return lspt, True

                    if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
                        log("Right double click heard")
                        log("Going back to table")
                        self.point_head([0.5, 0.0, -0.2], block = True)
                        break

                    rospy.sleep(0.01)
                    


            rospy.sleep(0.01)
        return None

    ##
    # Points head at given point in given frame.
    def point_head(self, point, velocity = 0.6, frame="/torso_lift_link", block = True):
        head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
        head_action_client.wait_for_server()
        goal = PointHeadGoal()
        goal.target = cf.create_point_stamped(point, frame)
        goal.pointing_frame = "/narrow_stereo_optical_frame"
        goal.max_velocity = velocity

        head_action_client.send_goal(goal)

        if not block:
            return 0

        finished_within_time = head_action_client.wait_for_result(rospy.Duration(20))
        if not finished_within_time:
            head_action_client.cancel_goal()
            rospy.logerr("timed out when asking the head to point to a new goal")
            return 0

        return 1

    ##
    # Commands the robot to move to a joint configuration directed at the given
    # point as if handing off in the general direction.  The point does not have
    # to be reachable by the gripper and is generally assumed to be outside of the
    # robot's reach.  The final pose is biased so that the arm usually looks natural,
    # i.e., it is similar to the way a human would handoff.
    def hand_over_object(self, x, y, z, offset = 0.2, blocking = True):
        if x < 0.2 or z < -1.0 or z > 1.0 or y > 2. or y < -2.:
            err("Cannot handoff to this location")
            return

        pt = np.array([x,y,z])
        quat = quaternion_about_axis(0.9, (0, 0, 1))
        dist = np.linalg.norm(pt)
        start_angles = self.oger.cm.get_current_arm_angles()
        log(start_angles)
        ptnorm = pt / dist
        dist -= offset
        joints = None
        while not rospy.is_shutdown() and pt[0] > 0.2:
            pt = dist * ptnorm
            log("dist", dist)
            pose = self.oger.create_gripper_pose(pt[0], pt[1], pt[2], quat.tolist())
            log(pose)

            
            self.oger.HANDOFF_BIAS = [0., -.25, -100., 200., 0.0, 200.0, 0.]
            joints = self.oger.cm.ik_utilities.run_biased_ik(pose, self.oger.HANDOFF_BIAS, num_iters=30)
            if joints is not None:
                break
            dist -= 0.1
    #   joints = [-0.1, -0.15, -1.2, -0.7, 0., -0.2, 0.]
        self.oger.cm.command_joint_trajectory([joints], 0.27, blocking = blocking)


    ##
    # Turns the projector_mode to auto and the trigger mode on the narrow to either
    # with the projector (on = True) or without the projector (on = False).
    def change_projector_mode(self, on):
        client = dynamic_reconfigure.client.Client("camera_synchronizer_node")
        node_config = client.get_configuration()
        node_config["projector_mode"] = 2
        if on:
            node_config["narrow_stereo_trig_mode"] = 3
        else:
            node_config["narrow_stereo_trig_mode"] = 4
        client.update_configuration(node_config)

    ##
    # Move arm to setup position, outside of vision area so that the arm is not in
    # the way when looking at the table.
    def move_to_setup(self, blocking = True):
        joints = [-0.62734204881265387, -0.34601608409943324, -1.4620635485239604, -1.2729772622637399, -7.5123303230158518, -1.5570651396529178, -5.5929916630672727] 
        self.oger.cm.command_joint_trajectory([joints], max_joint_vel = 0.62, blocking = blocking)


    ##
    # TODO REMOVE
    def laser_interface_demo(self):
        self.move_to_setup()

        x, y, z = self.get_laser_dclick(self.oger.cm.tf_listener)
    #   x, y, z = 0.7, 0.0, -0.3
        self.point_head([x,y,z], block = True)

        self.grasp_closest_object(x, y)
        # self.change_projector_mode(on=False)

        x, y, z = self.get_laser_dclick(self.oger.cm.tf_listener)
    #   x, y, z = 1.2, 0.0, 0.3
        self.point_head([x,y,z+0.2], block = True)
        # self.hand_over_object(x, y, z, cm, apm)
    
    ##
    # Block until a noticable difference in fingertip pressure has been noticed.
    # Monitor begins by zeroing out the sensors and using the perception monitor
    # to determine when an abnormal pressure event has occured.
    def monitor_pressure(self, arm):
        apm = ArmPerceptionMonitor(arm, percept_mon_list=PRESSURE_LIST)
        rospy.sleep(1.0)

        models = {}
        for k in self.oger.PRESSURE_LIST:
            if "periph" in k:
                models[k] = {}
                models[k]["mean"] = np.array([np.array([0.]*6)])
                models[k]["variance"] = np.array([np.array([40.]*6)])
            if "pad" in k:
                models[k] = {}
                models[k]["mean"] = np.array([np.array([0.]*15)])
                models[k]["variance"] = np.array([np.array([40.]*15)])

        apm.begin_monitoring(models, only_pressure=True)
        log("Monitoring pressure...")
        while not rospy.is_shutdown() and not apm.failure:
            rospy.sleep(0.01)
        log("Pressure triggered")

    def do_grasp_or_place(self, x, y, rot, z=None, is_place=False, repeat=True):
        grasp_result = self.oger.perform_grasp(x, y, gripper_rot=rot, 
                                               is_place=is_place)
        if grasp_result == "No collision":
            log("Didn't detect a collision so not doing anything")
            return False
        elif grasp_result == "Environmental collision":
            log("Environmental collision detected! Press c to retry.")
            self.ki.pause()
            if repeat:
                result = self.do_grasp_or_place(x, y, rot, z=z, is_place=is_place)
                return result
            else:
                return False
        elif grasp_result == "IK failure":
            log("Failed to find IK for the trajectory so grasp has failed")
            return False
        elif grasp_result == "Table collision":
            rospy.sleep(0.3)
            if self.is_obj_in_gripper():
                return True
            else:
                if repeat:
                    log("Unsuccessful grasp. Will reattempt.")
                    result = self.do_grasp_or_place(x, y, rot, z=z, is_place=is_place)
                    return result
                else:
                    log("Unsuccessful grasp. Will not reattempt.")
                    return False
        elif grasp_result == "Table collision":
            rospy.sleep(0.3)
            if self.is_obj_in_gripper():
                return True
            else:
                if repeat:
                    log("Unsuccessful grasp. Will reattempt.")
                    result = self.do_grasp_or_place(x, y, rot, z=z, is_place=is_place)
                    return result
                else:
                    log("Unsuccessful grasp. Will not reattempt.")
                    return False
        else:
            err("Bad grasp result name %s" % grasp_result)
            return False

    def setup_grasp(self, block = False):
        self.open_gripper(blocking = False)
        self.point_head([0.5, 0.0, -0.2], block=False)
        self.move_to_setup(blocking = block)

    ##
    # Runs looping grasping demos. Each of the three modes will continue repeating until
    # the program is exited.  Mode "random" will do random blind grasps in its training
    # region.  Mode "vision" will use vision perception to grasp the object closest
    # to a point roughly in the center of the table.  Mode "laser" requires laser_interface
    # nodes to be running and will allow the user to control the grasps with the mouse.
    def grasping_demos(self, mode):

        self.open_gripper(blocking = True)
        
        num_grasps = 100
        num_collided = 0
        self.move_to_setup()

        for i in range(num_grasps):
            self.point_head([0.5, 0.0, -0.2], block=False)
            if rospy.is_shutdown():
                break

            if mode == "random":
                x, y, rot = self.oger.random_grasp_total()
                if self.do_grasp_or_place(x, y, rot, is_place=False):
                    log("Successful grasp! Now attempting random place")
                    x, y, rot = self.oger.random_grasp_total()
                    if self.do_grasp_or_place(x, y, rot, is_place=True):
                        log("Successful place!")
            elif mode == "vision":
                x, y, rot = self.oger.random_grasp_total()
                obj = self.detect_closest_object(x, y)
                x, y, rot, z = self.get_grasp_loc(obj)
                if True:
                    z = None
                if self.do_grasp_or_place(x, y, rot, z=z, is_place=False):
                    log("Successful grasp! Now attempting random place")
                    x, y, rot = self.oger.random_grasp_total()
                    if self.do_grasp_or_place(x, y, rot, is_place=True):
                        log("Successful place!")
            elif mode == "laser":
                obj = None
                while obj is None:
                    # keep looping until we have grasped an object
                    laser_pt, is_handoff = self.get_laser_dclick(self.oger.cm.tf_listener)
                    x, y, z = laser_pt
                    if not is_handoff:
                        obj = self.grasp_closest_object(x, y)
                # get next location for either grasp or handoff
                laser_pt, is_handoff = self.get_laser_dclick(self.oger.cm.tf_listener)
                x, y, z = laser_pt
                log(" X Y Z")
                log(x, y, z)
                if is_handoff:
                    self.point_head([x, y, z], block = False)
                    self.hand_over_object(x, y, z)
                    self.monitor_pressure()
                    self.open_gripper()
                else:
                    self.oger.perform_grasp(x, y, gripper_rot=obj[1], is_place=True)
            # self.move_to_setup(blocking = False)

            rospy.sleep(1.0)
            # if self.ki.kbhit():
            #     if self.oger.PERCEPT_GRASP_LIST is None:
            #         percept = "joint_efforts"
            #     else:
            #         percept = self.oger.PERCEPT_GRASP_LIST[0]
            #     percept = "accelerometer"
            #     inds = range(3) #[3, 4, 5, 6]
            #     # display_grasp_data(grasps[-3:], percept, monitor_data=monitor_data[-3:], monitor_zeros=monitor_zeros[-3:], indicies=inds)
            #     # log(monitor_data[-3:])
            #     break

        # log("Accuracy: %d collisions out of %d grasps (%1.2f)" % (num_collided, num_grasps, 1. - float(num_collided) / num_grasps))
        # test_data = [grasps, monitor_data, collided_list]
        # for k in z_sum:
        #     log(z_sum[k] / num_grasps)
        return 0
    # _save_pickle(test_data, "test_runs.pickle")

    def start_grasping_server(self):
        self.grasping_server = actionlib.SimpleActionServer('overhead_grasp', OverheadGraspAction, self.execute_grasping_goal, False)
        self.grasping_server.register_preempt_callback(self.oger.kill_arm_movement)
        self.grasping_server.start()
        self.setup_server = actionlib.SimpleActionServer('overhead_grasp_setup', OverheadGraspSetupAction, self.execute_grasping_setup, False)
        self.setup_server.register_preempt_callback(self.oger.kill_arm_movement)
        self.setup_server.start()

    def execute_grasping_setup(self, goal):
        result = OverheadGraspSetupResult()
        self.setup_grasp(block=True)
        log("Finished setup")
        self.setup_server.set_succeeded(result)

    def execute_grasping_goal(self, goal):
        result = OverheadGraspResult()
        
        if goal.grasp_type == goal.MANUAL_GRASP:
            x, y, rot = goal.x, goal.y, goal.rot

        elif goal.grasp_type == goal.VISION_GRASP:
            obj = self.detect_closest_object(goal.x, goal.y)
            if obj is None:
                log("No objects detected")
                self.grasping_server.set_aborted(result)
                return
            x, y, rot, z = self.get_grasp_loc(obj)

        elif goal.grasp_type == goal.RANDOM_GRASP:
            x, y, rot = self.oger.random_grasp_total()

        else:
            err("Bad grasp type")
            self.grasping_server.set_aborted(result)
            return

        grasp_result = self.oger.perform_grasp(x, y, gripper_rot=rot, 
                                               is_place=not goal.is_grasp)
        result.grasp_result = grasp_result
        self.grasping_server.set_succeeded(result)

def testing(arm):
    return
    # ogm = OverheadGraspManager(arm, active = False)

    # ogm.oger.compile_classification_data()
    # return

    # ogm.oger.save_annotated_plots()
    # return

    # ogm.oger.annotate_collision_times()
    # return

    # gname = "grasp_cdata_table1005498-000565-021608.pickle"
    # # ogm.oger.process_signals(gname)
    # # grasp = _load_pickle(gname)
    # ogm.oger.display_grasp_model(grasp, 'gripper_pose', 2, coll_pt = 2.24)
    # # ogm.oger.save_grasp_model_plots(grasp, "test.pdf")
    # return
    # # grasp = _load_pickle("grasp_model0052--002-0039.pickle")
    # # ogm.oger.display_grasp_model(grasp, 'gripper_pose', 0)
    # # return
    # # ogm.oger.fix()
    # # return
    # # ogm.oger.generate_mean_models()
    # ogm.oger.generate_space_models()
    # ogm.oger.write_model_index_file()

##
# Main functionality entry point for stand alone use.
def grasping_main(arm, mode, args):
    if mode == "random":
        ogm = OverheadGraspManager(arm, active = True)
        ogm.grasping_demos(mode)

    elif mode == "vision":
        ogm = OverheadGraspManager(arm, active = True)
        ogm.grasping_demos(mode)

    elif mode == "laser":
        ogm = OverheadGraspManager(arm, active = True)
        ogm.grasping_demos(mode)

    elif mode == "service":
        ogm = OverheadGraspManager(arm, active = True, use_classifiers=True)
        ogm.start_grasping_server()
        log("Grasping server started")
        rospy.spin()

    elif mode == "testing":
        testing(arm)

    else:
        err("Bad mode name")

##
# Main.
def main(args):
    rospy.init_node(node_name)
    # rospy.on_shutdown(die)
 
    ################### Options #######################
    import optparse
    p = optparse.OptionParser()
    p.add_option('-f', '--left', action='store_true', dest='left', default=False,
                 help="Use left arm (else right).")
    p.add_option('-r', '--randgrasp', action='store_true', dest='random_grasps', default=False,
                 help="Grasp demo with random grasps, no vision.")
    p.add_option('-v', '--visiongrasp', action='store_true', dest='vision_grasps', default=False,
                 help="Grasp demo that picks up closest object to a point in the center of the table, with vision.")
    p.add_option('-l', '--lasergrasp', action='store_true', dest='laser_grasps', default=False,
                 help="Grasp demo with laser pointer control, with vision.")
    p.add_option('-s', '--service', action='store_true', dest='service', default=False,
                 help="Launch grasp service.")
    p.add_option('-t', '--testing', action='store_true', dest='testing', default=False,
                 help="Testing (UNSTABLE).")
    opt, args = p.parse_args()
    ####################################################

    mode = ""
    if opt.left:
        arm = 1
    else:
        arm = 0
    args = []
    if opt.random_grasps:
        mode = "random"
    if opt.vision_grasps:
        mode = "vision"
    if opt.laser_grasps:
        mode = "laser"
    if opt.service:
        mode = "service"
    if opt.testing:
        mode = "testing"
    if mode != "":
        grasping_main(arm, mode, args)
        return 0

    err("Must provide mode option")
    return 1

if __name__ == "__main__":
    sys.exit(main(sys.argv))
   
