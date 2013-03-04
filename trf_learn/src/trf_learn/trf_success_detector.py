#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import rospy
import trf_learn.srv as tm
import dynamic_reconfigure.client as dr
import pypr2.pr2_utils as pru
import hrl_camera.ros_camera as rc
import time
import hrl_pr2_lib.pr2 as pr2
import hrl_pr2_lib.pressure_listener as pm
import numpy as np
import functools as ft
import threading
import hrl_lib.tf_utils as tfu
import tf
import pdb

def image_diff_val2(before_frame, after_frame):
    br = np.asarray(before_frame)
    ar = np.asarray(after_frame)
    max_sum = br.shape[0] * br.shape[1] * br.shape[2] * 255.
    sdiff = np.abs((np.sum(br) / max_sum) - (np.sum(ar) / max_sum))
    return sdiff

class ConnectionCache:

    def __init__(self):
        self.connections = {}
        self.connections_creator = {}

    def add_connection_type(self, name, func):
        if self.connections_creator.has_key(name):
            rospy.loginfo('Connection %s already registered!' % name)
        else:
            self.connections_creator[name] = func

    def get(self, name):
        if not self.connections.has_key(name):
            self.connections[name] = self.connections_creator[name]()
        return self.connections[name]

class TRFSuccessDetector:

    def __init__(self):
        rospy.init_node('trf_success_detector')
        rospy.Service('classify_success_snapshot', tm.ClassifySuccessSnapshot, self.classify_success_snapshot_cb)
        rospy.Service('classify_success', tm.ClassifySuccess, self.classify_success_cb)

        self.req_number = 0
        self.success_detector_dict = {'light_switch':      LightSwitchSuccess,
                                      'drawer_pull_left':  ft.partial(DrawerPullSuccess, 'l'),
                                      'drawer_pull_right': ft.partial(DrawerPullSuccess, 'r'),
                                      'drawer_push_left':  ft.partial(DrawerPushSuccess, 'l'),
                                      'drawer_push_right': ft.partial(DrawerPushSuccess, 'r')}

        self.detectors = {}
        self.connection_cache = ConnectionCache()
        def joint_provider_f():
            return pru.create_joint_provider()

        self.connection_cache.add_connection_type('joint_provider', joint_provider_f)
        self.connection_cache.get('joint_provider')

    def classify_success_snapshot_cb(self, req):
        actionid      = req.actionid
        detector_type = req.success_detector

        rospy.loginfo('TRFSuccessDetector.classify_success_snapshot_cb: called for %s with type %s'\
                % (actionid, detector_type))

        self.req_number = (self.req_number + 1) % 1000000
        request_id = req.actionid + '_' + str(self.req_number)
        detector = self.success_detector_dict[detector_type](request_id, self.connection_cache)
        #pdb.set_trace()
        detector.take_snapshot()
        self.detectors[request_id] = detector
        return tm.ClassifySuccessSnapshotResponse(request_id)

    def classify_success_cb(self, req):
        #pdb.set_trace()
        result = self.detectors[req.request_id].classify_success()
        self.detectors.pop(req.request_id)
        rospy.loginfo(str(req.request_id) + ' success result: ' + str(result).upper())
        return tm.ClassifySuccessResponse(result)


class PressureWatch(threading.Thread):

    def __init__(self, pressure_obj, arm, tf_listener, push_tolerance):
        threading.Thread.__init__(self)    
        self.pressure_obj = pressure_obj
        self.arm = arm
        self.tf_listener = tf_listener
        self.push_tolerance = push_tolerance

        self.result = None
        self.should_run = True

    ##
    # @param frame frame to return the pose in
    # return the cartesian position (no orientation)
    def pose_cartesian(self, frame='base_link'):
        gripper_tool_frame = self.arm + '_gripper_tool_frame'
        mat = tfu.transform(frame, gripper_tool_frame, self.tf_listener)
        return mat[0:3, 3]

    def run(self):
        contact_loc = None

        #Figure out when we made contact
        while self.should_run:
            #if exceeded threshold
            if self.pressure_obj.check_threshold():
                contact_loc = self.pose_cartesian()
                break

        if contact_loc == None:
            rospy.loginfo('PressureWatch: NO CONTACT MADE! Returning failure.')
            self.result = False
            return

        r = rospy.Rate(10)
        while self.should_run:
            r.sleep()

        end_pose = self.pose_cartesian()
        if np.linalg.norm(contact_loc - end_pose) > self.push_tolerance:
            self.result = True 
        else:
            self.result = False

    def get_result(self):
        return self.result

    def stop(self):
        self.should_run = False
        rospy.loginfo('PressureWatch: stopped called! waiting for thread to die...')
        self.join()
        rospy.loginfo('PressureWatch: stopped thread.')

class DrawerPushSuccess:

    PUSH_TOLERANCE = .1

    def __init__(self, arm, requestid, connection_cache):
        self.connection_cache = connection_cache
        self.requestid = requestid
        self.arm = arm
        self.ptopic = '/pressure/' + self.arm + '_gripper_motor'
        if self.arm == 'l':
            self.connection_name = 'left_pressure'
        else:
            self.connection_name = 'right_pressure'

        def pressure_listener_f():
            return pm.PressureListener(self.ptopic, 6000)

        def tf_listener_f():
            return tf.TransformListener()

        self.connection_cache.add_connection_type(self.connection_name, pressure_listener_f)
        self.connection_cache.add_connection_type('tf_listener', tf_listener_f)

    def take_snapshot(self):
        pressure = self.connection_cache.get(self.connection_name)
        tf_listener = self.connection_cache.get('tf_listener')
        pressure.rezero()
        pressure.set_threshold(500)
        self.watcher = PressureWatch(pressure, self.arm, tf_listener, DrawerPushSuccess.PUSH_TOLERANCE)
        self.watcher.start()

    def classify_success(self):
        #pressure = self.connection_cache.get(self.connection_name)
        #exceeded_threshold = pressure.check_threshold()
        self.watcher.stop()
        #pdb.set_trace()
        result = self.watcher.get_result()
        if result == None:
            raise RuntimeError('DrawerPushSuccess: Result should not be NONE!')

        if result:
            return 'success'
        else:
            return 'failed'


class DrawerPullSuccess:

    GRIPPER_CLOSE = .004

    def __init__(self, arm, requestid, connection_cache):
        self.connection_cache = connection_cache
        self.requestid = requestid
        self.arm = arm
        if self.arm == 'l':
            self.connection_name = 'left_gripper'
        else:
            self.connection_name = 'right_gripper'

        def gripper_f():
            return pr2.PR2Gripper(self.arm, self.connection_cache.get('joint_provider'))
        self.connection_cache.add_connection_type(self.connection_name, gripper_f)

    def take_snapshot(self):
        pass

    def classify_success(self):
        gripper = self.connection_cache.get(self.connection_name)
        has_handle = gripper.pose()[0,0] > DrawerPullSuccess.GRIPPER_CLOSE
        rospy.loginfo('DrawerPullSuccess: gripper size %f threshold %f.  result %s' % (gripper.pose()[0,0], DrawerPullSuccess.GRIPPER_CLOSE, str(has_handle)))
        if has_handle:
            return 'success'
        else:
            return 'failed'


class LightSwitchSuccess:

    def __init__(self, requestid, connection_cache, angles=np.matrix([-30., -30.]).T):
        self.threshold = .03
        self.requestid = requestid
        self.connection_cache = connection_cache
        self.angles = angles

        def wide_angle_configure_f():
            return dr.Client('wide_stereo_both')

        def pr2_head_f():
            return pru.PR2Head(self.connection_cache.get('joint_provider'))

        def wide_angle_camera_left_f():
            return rc.ROSCamera('/wide_stereo/left/image_rect_color')

        self.connection_cache.add_connection_type('wide_angle_configure', wide_angle_configure_f)
        self.connection_cache.add_connection_type('pr2_head', pr2_head_f)
        self.connection_cache.add_connection_type('wide_angle_camera_left', wide_angle_camera_left_f)

    def take_snapshot(self):
        pr2_head = self.connection_cache.get('pr2_head')
        wide_angle_configure = self.connection_cache.get('wide_angle_configure')
        wide_angle_camera_left = self.connection_cache.get('wide_angle_camera_left')

        self.config = wide_angle_configure.get_configuration()
        self.config['auto_gain'] = False
        self.config['auto_exposure'] = False
        wide_angle_configure.update_configuration(self.config)

        #take before sensor snapshot
        self.start_pose = pr2_head.pose()
        rospy.loginfo('LightSwitchSuccess: Making head point up at %s !' % (str(self.angles)))
        pr2_head.set_pose(np.radians(self.angles), 1)
        rospy.sleep(4)
        for i in range(7):
            before_frame = wide_angle_camera_left.get_frame()
        self.before_frame = before_frame

    def classify_success(self):
        pr2_head = self.connection_cache.get('pr2_head')
        wide_angle_configure = self.connection_cache.get('wide_angle_configure')
        wide_angle_camera_left = self.connection_cache.get('wide_angle_camera_left')

        rospy.sleep(4)
        for i in range(7):
            after_frame = wide_angle_camera_left.get_frame()

        sdiff = image_diff_val2(self.before_frame, after_frame)
        rospy.loginfo('LightSwitchSuccess: Making head back to where it was at %s !' % (str(self.start_pose)))
        pr2_head.set_pose(self.start_pose, 1)
        rospy.sleep(3)        
        self.config['auto_gain'] = True
        self.config['auto_exposure'] = True
        wide_angle_configure.update_configuration(self.config)

        rospy.loginfo('=======================================')
        rospy.loginfo('=======================================')
        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, self.threshold))
        if sdiff > self.threshold:
            rospy.loginfo('difference detected!')
            return 'success'
        else:
            rospy.loginfo('NO differences detected!')
            return 'failed'
        rospy.loginfo('=======================================')
        rospy.loginfo('=======================================')

if __name__ == '__main__':
    detector = TRFSuccessDetector()
    #d = detector.success_detector_dict['drawer_pull']('sdf', detector.connection_cache)
    #pdb.set_trace()
    #d.classify_success()
    rospy.loginfo('Ready!')
    rospy.spin()

