#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import rospy
import trf_learn.srv as tm
import dynamic_reconfigure.client as dr
import pypr2.pr2_utils as pru
import hrl_camera.ros_camera as rc
import time
import hrl_pr2_lib.pr2 as pr2
import numpy as np
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
        self.success_detector_dict = {'light_switch': LightSwitchSuccess,
                                      'drawer_pull': DrawerPullSuccess}

        self.detectors = {}
        self.connection_cache = ConnectionCache()
        def joint_provider_f():
            return pru.create_joint_provider()
        self.connection_cache.add_connection_type('joint_provider', joint_provider_f)
        self.connection_cache.get('joint_provider')

    def classify_success_snapshot_cb(self, req):
        actionid      = req.actionid
        detector_type = req.success_detector

        self.req_number = (self.req_number + 1) % 1000000
        request_id = req.actionid + '_' + str(self.req_number)
        detector = self.success_detector_dict[detector_type](request_id, self.connection_cache)
        pdb.set_trace()
        detector.take_snapshot()
        self.detectors[request_id] = detector
        return tm.ClassifySuccessSnapshotResponse(request_id)

    def classify_success_cb(self, req):
        pdb.set_trace()
        result = self.detectors[req.request_id].classify_success()
        self.detectors.pop(req.request_id)
        rospy.loginfo(str(req.request_id) + ' success result: ' + str(result))
        return tm.ClassifySuccessResponse(result)


class DrawerPullSuccess:

    GRIPPER_CLOSE = .003

    def __init__(self, requestid, connection_cache):
        self.connection_cache = connection_cache
        self.requestid = requestid

        def left_gripper_f():
            return pr2.PR2Gripper('l', self.connection_cache.get('joint_provider'))

        def right_gripper_f():
            return pr2.PR2Gripper('l', self.connection_cache.get('joint_provider'))

        self.connection_cache.add_connection_type('left_gripper', left_gripper_f)
        self.connection_cache.add_connection_type('right_gripper', right_gripper_f)

    def take_snapshot(self):
        pass

    def classify_success(self):
        gripper = self.connection_cache.get('right_gripper')
        has_handle = gripper.pose()[0,0] > DrawerPullSuccess.GRIPPER_CLOSE
        if has_handle:
            return 'success'
        else:
            return 'failed'


class LightSwitchSuccess:

    def __init__(self, requestid, connection_cache, angles=np.matrix([-30., -30.]).T):
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
        wide_angle_configure.update_configuration(config)

        #take before sensor snapshot
        self.start_pose = pr2_head.pose()
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
        pr2_head.set_pose(self.start_pose, 1)
        rospy.sleep(3)        
        self.config['auto_gain'] = True
        self.config['auto_exposure'] = True
        wide_angle_configure.update_configuration(config)

        rospy.loginfo('=======================================')
        rospy.loginfo('=======================================')
        rospy.loginfo('camera difference %.4f (thres %.3f)' % (sdiff, threshold))
        if sdiff > threshold:
            rospy.loginfo('difference detected!')
            return 'success'
        else:
            rospy.loginfo('NO differences detected!')
            return 'failed'
        rospy.loginfo('=======================================')
        rospy.loginfo('=======================================')

if __name__ == '__main__':
    detector = TRFSuccessDetector()
    rospy.loginfo('Ready!')
    rospy.spin()

