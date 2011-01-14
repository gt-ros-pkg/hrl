
import pr2_laser_snapshotter.srv as snp
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

#class TransformBroadcaster:
#    def __init__(self):
#        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)
#
#    ## send transform as a tfmessage.
#    # @param tf_stamped - object of class TransformStamped (rosmsg show TransformStamped)
#    def send_transform(self,tf_stamped):
#        tfm = tf.msg.tfMessage([tf_stamped])
#        self.pub_tf.publish(tfm)
