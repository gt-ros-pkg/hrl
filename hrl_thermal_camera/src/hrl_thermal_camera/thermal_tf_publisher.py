#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_thermal_camera')
import rospy
import tf
# from helper_functions import createBMatrix, Bmat_to_pos_quat
from geometry_msgs.msg import PoseStamped


class ThermalCameraTFBroadcaster(object):
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)

        self.out_trans = [0., -0.07, 0.]
        self.out_rot = [0., 0., 0., 1.]

        print 'Thermal TF broadcaster is up.'
        self.start_broadcast()

    def start_broadcast(self):
        while (not self.tf_listener.canTransform('base_link', 'head_mount_kinect_ir_optical_frame', rospy.Time(0))) \
                and not rospy.is_shutdown():
                print 'Waiting for pr2 to exist in world.'
                rospy.sleep(2)
        rospy.sleep(1)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            # try:
            self.tf_broadcaster.sendTransform(self.out_trans, self.out_rot,
                                              rospy.Time.now(),
                                              'thermal_cam',
                                              'head_mount_kinect_depth_optical_frame')
            rate.sleep()
            # except:
            #         print 'Thermal Camera TF broadcaster crashed while trying to broadcast!'
            #         break
        print 'Thermal Camera TF broadcaster crashed!'


if __name__ == '__main__':
    rospy.init_node('thermal_tf_broadcaster')
    thermal_tf_broadcaster = ThermalCameraTFBroadcaster()
    # rospy.spin()







