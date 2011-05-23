#! /usr/bin/python

import numpy as np, math
import sys

import roslib; roslib.load_manifest('pr2_overhead_grasping')
import rospy

import actionlib
from tf.transformations import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import object_manipulator.convert_functions as cf
from object_manipulator.cluster_bounding_box_finder import ClusterBoundingBoxFinder
from tabletop_object_detector.srv import TabletopSegmentation
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

from pr2_overhead_grasping.msg import *
from pr2_overhead_grasping.srv import *

#from laser_interface.pkg import CURSOR_TOPIC, MOUSE_DOUBLE_CLICK_TOPIC, CURSOR_TOPIC, MOUSE_R_CLICK_TOPIC, MOUSE_R_DOUBLE_CLICK_TOPIC

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
        detects = object_detector()
        object_detector.close()
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
#self.change_projector_mode(True)
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

#self.change_projector_mode(False)
                return obj
            else:
                rospy.loginfo("No objects near point")
                return None
        return None

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
    #def get_laser_dclick(self, tf_listener, frame = "/torso_lift_link", delay_time = 3.0):
    #    c3d_lis = GenericListener("c3dlisnode", PointStamped, self.gman.CURSOR_TOPIC, 10.0)
    #    dc_lis = GenericListener("dclisnode", String, self.gman.MOUSE_DOUBLE_CLICK_TOPIC, 10.0)
    #    rdc_lis = GenericListener("rdclisnode", String, self.gman.MOUSE_R_DOUBLE_CLICK_TOPIC, 10.0)
    #    # face_detect_cli = actionlib.SimpleActionClient('face_detector_action', face_detector.msg.FaceDetectorAction)
    #    rospy.loginfo("Waiting for laser click...")
    #    while not rospy.is_shutdown():
    #        if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
    #            rospy.loginfo("Double click heard")
    #            msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
    #            rospy.loginfo(msg)
    #            if msg is not None:
    #                # @TODO contain this code?
    #                if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
    #                    now = rospy.Time.now()
    #                    tf_listener.waitForTransform(msg.header.frame_id, frame, 
    #                                                 now, rospy.Duration(4.0))
    #                    tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
    #                    tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
    #                    pt = tfmat[0:3,3]
    #                    #(trans, rot) = tf_listener.lookupTransform(msg.header.frame_id, frame, now)
    #                    #pt = [msg.point.x - trans[0], msg.point.y - trans[1], msg.point.z - trans[2]]
    #                    rospy.loginfo("pt", pt)
    #                    return [pt[0,0], pt[1,0], pt[2,0]], False

    #        if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
    #            rospy.loginfo("Right double click heard")
    #            rospy.loginfo("Double click on person to hand-off")
    #            self.point_head([1.0, 0.0, 0.05], block = True)

    #            # begin waiting for person selection
    #            while not rospy.is_shutdown():
    #                if dc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
    #                    rospy.loginfo("Double click heard")
    #                    msg = c3d_lis.read(allow_duplication = True, willing_to_wait = False)
    #                    rospy.loginfo(msg)
    #                    if msg is not None:
    #                        if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= delay_time:
    #                            now = rospy.Time.now()
    #                            tf_listener.waitForTransform(msg.header.frame_id, frame, 
    #                                                         now, rospy.Duration(4.0))
    #                            tfmat = tf_utils.transform(frame, msg.header.frame_id, tf_listener)
    #                            tfmat *= np.mat([[msg.point.x], [msg.point.y], [msg.point.z], [1.0]])
    #                            lspt = tfmat[0:3,3]
    #                            lspt = [ lspt[0,0], lspt[1,0], lspt[2,0]]
    #                            rospy.loginfo("pt", lspt)
    #                
    #                            # rospy.loginfo("Waiting for face detection server")
    #                            # face_detect_cli.wait_for_server()
    #                            # face_req = face_detector.msg.FaceDetectorGoal()
    #                            # face_detect_cli.send_goal(face_req)
    #                            # face_detect_cli.wait_for_result()
    #                            # face_poss = face_detect_cli.get_result().face_positions
    #                            # rospy.loginfo("Got face results:", face_poss)

    #                            # # transform face positions to our frame
    #                            # face_pts = []
    #                            # for face_pos in face_poss:
    #                            #     now = rospy.Time.now()
    #                            #     tf_listener.waitForTransform(face_pos.header.frame_id, frame, 
    #                            #                                  now, rospy.Duration(4.0))
    #                            #     tfmat = tf_utils.transform(frame, face_pos.header.frame_id, tf_listener)
    #                            #     tfmat *= np.mat([[face_pos.pos.x], [face_pos.pos.y], [face_pos.pos.z], [1.0]])
    #                            #     fcpt = tfmat[0:3,3]
    #                            #     fcpt = [ fcpt[0,0], fcpt[1,0], fcpt[2,0]]
    #                            #     face_pts.append(fcpt)

    #                            # rospy.loginfo("Face locations", face_pts)
    #                            # for face_pt in face_pts:
    #                            #     dist = np.sqrt((face_pt[0] - lspt[0])**2 + (face_pt[1] - lspt[1])**2)
    #                            #     if dist < 0.35 and face_pt[2] > 0.3:
    #                            #         return face_pt, True
    #                            lspt[2] += 0.2
    #                            return lspt, True

    #                if rdc_lis.read(allow_duplication = False, willing_to_wait = False) is not None:
    #                    rospy.loginfo("Right double click heard")
    #                    rospy.loginfo("Going back to table")
    #                    self.point_head([0.5, 0.0, -0.2], block = True)
    #                    break

    #                rospy.sleep(0.01)
    #                


    #        rospy.sleep(0.01)
    #    return None

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
    # Commands the robot to move to a joint configuration directed at the given
    # point as if handing off in the general direction.  The point does not have
    # to be reachable by the gripper and is generally assumed to be outside of the
    # robot's reach.  The final pose is biased so that the arm usually looks natural,
    # i.e., it is similar to the way a human would handoff.
    def hand_over_object(self, x, y, z, offset = 0.2, blocking = True):
        if x < 0.2 or z < -1.0 or z > 1.0 or y > 2. or y < -2.:
            rospy.logerr("Cannot handoff to this location")
            return

        pt = np.array([x,y,z])
        quat = quaternion_about_axis(0.9, (0, 0, 1))
        dist = np.linalg.norm(pt)
        start_angles = self.gman.cm.get_current_arm_angles()
        rospy.loginfo(start_angles)
        ptnorm = pt / dist
        dist -= offset
        joints = None
        while not rospy.is_shutdown() and pt[0] > 0.2:
            pt = dist * ptnorm
            rospy.loginfo("dist", dist)
            pose = self.gman.create_gripper_pose(pt[0], pt[1], pt[2], quat.tolist())
            rospy.loginfo(pose)

            
            self.gman.HANDOFF_BIAS = [0., -.25, -100., 200., 0.0, 200.0, 0.]
            joints = self.gman.cm.ik_utilities.run_biased_ik(pose, self.gman.HANDOFF_BIAS, num_iters=30)
            if joints is not None:
                break
            dist -= 0.1
    #   joints = [-0.1, -0.15, -1.2, -0.7, 0., -0.2, 0.]
        self.gman.cm.command_joint_trajectory([joints], 0.27, blocking = blocking)

    ##
    # Move the arm to a suitable setup position for moving to a grasp position
    def setup_grasp(self, block = False, disable_head=False):
        #self.open_gripper(blocking=False)
        if not disable_head:
            self.point_head([0.3, 0.0, -0.3], block=False)
        self.gman.grasp_preparation_move(blocking=block)

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
        self.setup_grasp(block=True, disable_head=goal.disable_head)
        rospy.loginfo("Finished setup")
        self.setup_server.set_succeeded(result)

    ##
    # Executes grasping goal requested on actionlib srvs. Actions differ based
    # on type of grasp requested.
    def execute_grasping_goal(self, goal):
        result = OverheadGraspResult()
        
        # User specifies parameters
        if goal.grasp_type == goal.MANUAL_GRASP:
            grasp_params = goal.grasp_params

        # Robot finds parameters
        elif goal.grasp_type == goal.VISION_GRASP:
            obj = self.detect_closest_object(goal.grasp_params[0], 
                                             goal.grasp_params[1], 
                                             disable_head=goal.disable_head)
            if obj is None:
                rospy.loginfo("No objects detected")
                result.grasp_result = "No objects detected"
                self.grasping_server.set_aborted(result)
                return
            x, y, rot, z = self.get_grasp_loc(obj)
            grasp_params = (x, y, rot)

        # execute a random grasp
        elif goal.grasp_type == goal.RANDOM_GRASP:
            grasp_params = self.random_generator()

        else:
            rospy.logerr("Bad grasp type")
            self.grasping_server.set_aborted(result)
            return

        grasp_result = self.gman.perform_grasp(grasp_params, collide=not goal.disable_coll,
                                               behavior_name=goal.behavior_name,
                                               sig_level=goal.sig_level,
                                               is_place=not goal.is_grasp)
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
