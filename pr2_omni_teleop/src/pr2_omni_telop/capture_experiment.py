#! /usr/bin/python
import roslib; roslib.load_manifest('pr2_omni_teleop')
import rospy
import hrl_camera.ros_camera as rc
import cv
import hrl_lib.rutils as ru
import hrl_lib.util as ut
import math

#TF
import tf
import hrl_lib.tf_utils as tfu
import tf.transformations as tr


if __name__ == '__main__':
    import sys

    base_name = sys.argv[1]
    test = 'laser'
    ls = ru.LaserScanner('point_cloud_srv')
    prosilica = rc.Prosilica('prosilica', 'streaming')
    tf_listener = tf.TransformListener()

    rospy.loginfo( 'Getting laser scan.')
    points = ls.scan(math.radians(180.), math.radians(-180.), 20.)
    rospy.loginfo('Size of point cloud: %d' % len(points.points))
    rospy.loginfo( 'Grabbing image.')
    image = prosilica.get_frame()
    rospy.loginfo( 'Grabbing transforms.')

    #transform from tilt_laser => base_footprint (pointcloud is already in base_footprint)
    #transform from base_footprint => (pose of head) prosilica
    pro_T_bf = tfu.transform('/high_def_optical_frame', '/base_footprint', tf_listener)

    #transform from base_footprint => map
    map_T_bf = tfu.transform('/map', '/base_footprint', tf_listener)

    #get camera's P matrix
    rospy.loginfo('Waiting for camera_info.')
    calibration = rc.ROSCameraCalibration('/prosilica/camera_info')
    r = rospy.Rate(10)
    while not rospy.is_shutdown() and calibration.has_msg == False:
        r.sleep()

    rospy.loginfo('Saving.')
    pkl_name = '%s.pkl' % base_name
    img_name = '%s.png' % base_name
    ut.save_pickle({'points': points, 
                    'pro_T_bf': pro_T_bf, 
                    'map_T_bf': map_T_bf, 
                    'camera_info': calibration},  pkl_name)
    cv.SaveImage(img_name, image)
    rospy.loginfo( 'Saved to %s and %s.' % (pkl_name, img_name))


