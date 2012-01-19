
import sensor_msgs.msg as sm
import pr2_arm_navigation_perception.srv as snp
#import tf
#import tf.msg
import rospy
import numpy as np
import hrl_lib.rutils as ru


##
# Converts a list of pr2_msgs/PressureState into a matrix
#
# @return left mat, right mat, array
def pressure_state_to_mat(contact_msgs):
    times = np.array([c.header.stamp.to_time() for c in contact_msgs])
    left, right = zip(*[[list(c.l_finger_tip), list(c.r_finger_tip)] for c in contact_msgs])
    
    left = np.matrix(left).T
    right = np.matrix(right).T
    return left, right, times

class LaserScanner:
    def __init__(self, service):
        srv_name = '/%s/single_sweep_cloud' % service
        self.sp = rospy.ServiceProxy(srv_name, snp.BuildCloudAngle)

    def scan_np(self, start, end, duration):
        resp = self.sp(start, end, duration)
        return ru.pointcloud_to_np(resp.cloud)

    def scan(self, start, end, duration):
        resp = self.sp(start, end, duration)
        return resp.cloud

def sweep_param_to_tilt_param(angle_begin, angle_end, duration):
    amplitude = abs(angle_end - angle_begin)/2.0 
    offset = (angle_end + angle_begin)/2.0 
    period = duration*2.0
    return {'amplitude': amplitude, 'offset':offset, 'period': period}

class PointCloudReceiver:
    def __init__(self, topic):
        self.listener = ru.GenericListener('point_cloud_receiver', sm.PointCloud, topic, .2)
        self.topic = topic

    def read(self):
        cur_time = rospy.Time.now().to_sec()
        not_fresh = True

        while not_fresh:
            pcmsg = self.listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
            if not (pcmsg.header.stamp.to_sec() < cur_time):
                not_fresh = False
            tdif = rospy.Time.now().to_sec() - cur_time
            if tdif > 10.:
                rospy.logerr('Have not heard from %s for %.2f seconds.' % (self.topic, tdif))
            rospy.sleep(.1)

        return pcmsg



