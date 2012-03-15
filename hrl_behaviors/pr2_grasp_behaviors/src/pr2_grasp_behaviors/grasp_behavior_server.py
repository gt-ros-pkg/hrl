#! /usr/bin/python

import numpy as np, math
import sys

import roslib; roslib.load_manifest('pr2_grasp_behaviors')
import rospy

import actionlib
from tf.transformations import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopSegmentation
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

from pr2_grasp_behaviors.msg import OverheadGraspAction, OverheadGraspResult
from pr2_grasp_behaviors.msg import OverheadGraspSetupAction, OverheadGraspSetupResult
from pr2_grasp_behaviors.msg import OverheadGraspFeedback 

#from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC, CURSOR_TOPIC, MOUSE_R_CLICK_TOPIC, MOUSE_R_DOUBLE_CLICK_TOPIC

class GraspStates:
    ACTIONLIB_CALLED = 'Actionlib grasping goal called'
    PERCEIVING_OBJECTS = 'Perceiving table objects'
    GRASP_SETUP_MOVE = 'Moving to grasp setup'
    EXECUTE_APPROACH = 'Executing grasp motion'
    GRASP_OBJECT = 'Closing gripper on object'
    EXECUTE_RETREAT = 'Retreating grasp motion'
    ACTIONLIB_COMPLETE = 'Actionlib grasping goal completed'

class GraspBehaviorServer(object):
    def __init__(self, arm, grasp_manager):
        self.arm = arm
        self.gman = grasp_manager

    ##
    # Open gripper fully.
    def open_gripper(self, blocking = False):
        self.gman.cm.command_gripper(1.0, -1.0, False)
        if blocking:
            self.gman.cm.gripper_action_client.wait_for_result(rospy.Duration(4.0))

    ##
    # Uses the object_detection service detect objects on the table. Returns a list
    # of pairs [ [ x, y, z], rot ] which represent the centers and z orientation of
    # each object detected on the table.
    def detect_tabletop_objects(self):
        DETECT_ERROR = 0.
        cbbf = ClusterBoundingBoxFinder(tf_listener=self.gman.cm.tf_listener)
        object_detector = rospy.ServiceProxy("/object_detection", TabletopSegmentation)
        try:
            detects = object_detector()
            object_detector.close()
        except ServiceException as se:
            rospy.logerr("Tabletop segmentation crashed")
            return []
        if detects.result != 4:
            rospy.logerr("Detection failed (err %d)" % (detects.result))
            return []
        table_z = detects.table.pose.pose.position.z
        objects = []
        for cluster in detects.clusters:
            (object_points, 
             object_bounding_box_dims, 
             object_bounding_box, 
             object_to_base_link_frame, 
             object_to_cluster_frame) = cbbf.find_object_frame_and_bounding_box(cluster)
            # rospy.loginfo("bounding box:", object_bounding_box)
            (object_pos, object_quat) = cf.mat_to_pos_and_quat(object_to_cluster_frame)
            angs = euler_from_quaternion(object_quat)
            # position is half of height
            object_pos[2] = table_z + object_bounding_box[1][2] / 2. + DETECT_ERROR
            objects += [[object_pos, angs[2]]]
        return objects

    def get_grasp_loc(self, obj):
        return obj[0][0], obj[0][1], obj[1], obj[0][2]

    ##
    # Given an (x, y) location on a table, grasp the closest object detected.
    # If repeat is True, will keep trying if the grasp fails.
    def detect_closest_object(self, x, y, repeat=True, disable_head=False):
        def dist(o):
            return (o[0][0] - x) ** 2 + (o[0][1] - y) ** 2

        grasped = False
        num_tries = 0

        if not disable_head:
            self.point_head([x,y,-0.3])

        while not grasped and num_tries < 4:
            rospy.loginfo("Detect in")
            rospy.sleep(0.6)
            detect_tries = 0
            objects = None
            while (objects is None or len(objects) == 0):
                objects = self.detect_tabletop_objects()
                rospy.sleep(0.6)
                detect_tries += 1
                if detect_tries == 3 and (objects is None or len(objects) == 0):
                    rospy.logerr("Cannot detect any objects")
                    return None
            rospy.loginfo("Detect out")
            if len(objects) > 0:
                try:
                    obj = min(objects, key=dist)

                    # Get better look
                    if not disable_head:
                        obj_pt = obj[0]
                        obj_pt[2] = -0.4
                        self.point_head(obj_pt)
                    rospy.sleep(0.2)
                    rospy.loginfo("Detect2 in")
                    objects = self.detect_tabletop_objects()
                    rospy.loginfo("Detect2 out")
                    obj = min(objects, key=dist)
                except:
                    pass

                return obj
            else:
                rospy.loginfo("No objects near point")
                return None
        return None

    ##
    # Points head at given point in given frame.
    def point_head(self, point, velocity = 0.6, frame="/torso_lift_link", block = True):
        head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
        head_action_client.wait_for_server()
        goal = PointHeadGoal()
        goal.target = cf.create_point_stamped(point, frame)
        goal.pointing_frame = "/openni_rgb_optical_frame"
        goal.max_velocity = velocity

        head_action_client.send_goal(goal)

        if not block:
            return 0

        finished_within_time = head_action_client.wait_for_result(rospy.Duration(10))
        if not finished_within_time:
            head_action_client.cancel_goal()
            rospy.logerr("timed out when asking the head to point to a new goal")
            return 0

        return 1

    ##
    # Move the arm to a suitable setup position for moving to a grasp position
    def setup_grasp(self, block = False, disable_head=False, open_gripper=False):
        if open_gripper:
            self.open_gripper(blocking=False)
        if not disable_head:
            self.point_head([0.3, 0.0, -0.3], block=False)
        self.gman.grasp_preparation_move()
        rospy.sleep(1.)
        if block:
            self.gman.arm_moving_wait(True, 8.0)

    ##
    # Launch actionlib srv calls.
    def start_grasping_server(self, grasp_server_name, setup_server_name):
        self.grasping_server = actionlib.SimpleActionServer(grasp_server_name, OverheadGraspAction, self.execute_grasping_goal, False)
        self.grasping_server.register_preempt_callback(self.gman.kill_arm_movement)
        self.grasping_server.start()
        self.setup_server = actionlib.SimpleActionServer(setup_server_name, OverheadGraspSetupAction, self.execute_grasping_setup, False)
        self.setup_server.register_preempt_callback(self.gman.kill_arm_movement)
        self.setup_server.start()
        rospy.loginfo("Grasping server launched on %s, setup at %s", 
                                         grasp_server_name, setup_server_name)

    ##
    # Wraps setup_grasp
    def execute_grasping_setup(self, goal):
        result = OverheadGraspSetupResult()
        self.setup_grasp(block=True, disable_head=goal.disable_head, open_gripper=goal.open_gripper)
        rospy.loginfo("Finished setup")
        self.setup_server.set_succeeded(result)


    ##
    # Executes grasping goal requested on actionlib srvs. Actions differ based
    # on type of grasp requested.
    def execute_grasping_goal(self, goal):
        result = OverheadGraspResult()
        feedback = OverheadGraspFeedback()
        def publish_state(state):
            feedback.state = state
            self.grasping_server.publish_feedback(feedback)
        publish_state(GraspStates.ACTIONLIB_CALLED)
        
        # User specifies parameters
        if goal.grasp_type == goal.MANUAL_GRASP:
            grasp_params = (goal.x, goal.y, goal.roll, goal.pitch)

        # Robot finds parameters
        elif goal.grasp_type == goal.VISION_GRASP:
            obj = self.detect_closest_object(goal.x, 
                                             goal.y, 
                                             disable_head=goal.disable_head)
            publish_state(GraspStates.PERCEIVING_OBJECTS)
            if obj is None:
                rospy.loginfo("No objects detected")
                result.grasp_result = "No objects detected"
                self.grasping_server.set_aborted(result)
                return
            x, y, rot, z = self.get_grasp_loc(obj)
            grasp_params = (x, y, rot, goal.pitch)

        # execute a random grasp
        elif goal.grasp_type == goal.RANDOM_GRASP:
            grasp_params = self.random_generator()

        else:
            rospy.logerr("Bad grasp type")
            self.grasping_server.set_aborted(result)
            return

        feedback.x, feedback.y = grasp_params[0], grasp_params[1]
        feedback.roll, feedback.pitch = grasp_params[2], grasp_params[3]
        grasp_result = self.gman.perform_grasp(grasp_params, collide=not goal.disable_coll,
                                               behavior_name=goal.behavior_name,
                                               sig_level=goal.sig_level,
                                               is_place=not goal.is_grasp,
                                               publish_state=publish_state)
        result.grasp_result = grasp_result
        if goal.is_grasp:
            if grasp_result == "Object grasped":
                self.grasping_server.set_succeeded(result)
            else:
                self.grasping_server.set_aborted(result)
        else:
            if grasp_result == "Object placed":
                self.grasping_server.set_succeeded(result)
            else:
                self.grasping_server.set_aborted(result)
        publish_state(GraspStates.ACTIONLIB_COMPLETE)
