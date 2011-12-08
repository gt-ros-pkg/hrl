#!/usr/bin/python

import roslib
roslib.load_manifest('rfid_people_following')
import rospy

from std_msgs.msg import Float64
import tf

import time
import numpy as np, math
import functools

class tf_updater():
    def __init__(self, name, 
                 right_pan = '/robotis/servo_right_pan',
                 right_tilt = '/robotis/servo_right_tilt',
                 left_pan = '/robotis/servo_left_pan',
                 left_tilt = '/robotis/servo_left_tilt'):
        try:
            rospy.init_node( name )
        except:
            pass

        # Right pan
        rospy.Subscriber( right_pan, Float64, 
                          functools.partial( self.rpan_cb, bc = tf.TransformBroadcaster() ))

        # Right tilt
        rospy.Subscriber( right_tilt, Float64, 
                          functools.partial( self.rtilt_cb, bc = tf.TransformBroadcaster() ))

        # Left pan
        rospy.Subscriber( left_pan, Float64, 
                          functools.partial( self.lpan_cb, bc = tf.TransformBroadcaster() ))

        # Left tilt
        rospy.Subscriber( left_tilt, Float64, 
                          functools.partial( self.ltilt_cb, bc = tf.TransformBroadcaster() ))




    def rpan_cb( self, ang_msg, bc ):
        # bc is a specific TransformBroadcaster
        bc.sendTransform( (-0.0655, -0.0510, 0.0675),
                          tf.transformations.quaternion_from_euler( 0.0, 0.0, ang_msg.data - math.radians( 60.0 ) ),
                          rospy.Time.now(),
                          'ear_pan_right',
                          'plate_right_base' )

    def rtilt_cb( self, ang_msg, bc ):
        # bc is a specific TransformBroadcaster
        bc.sendTransform( (0.0673, 0.0, 0.0),
                          tf.transformations.quaternion_from_euler( 0.0, ang_msg.data, 0.0 ),
                          rospy.Time.now(),
                          'ear_tilt_right',
                          'ear_pan_right' )

    def lpan_cb( self, ang_msg, bc ):
        # bc is a specific TransformBroadcaster
        bc.sendTransform( (-0.0655, +0.0510, 0.0675),
                          tf.transformations.quaternion_from_euler( 0.0, 0.0, ang_msg.data + math.radians( 60.0 ) ),
                          rospy.Time.now(),
                          'ear_pan_left',
                          'plate_left_base' )

    def ltilt_cb( self, ang_msg, bc ):
        # bc is a specific TransformBroadcaster
        bc.sendTransform( (0.0673, 0.0, 0.0),
                          tf.transformations.quaternion_from_euler( 0.0, -1.0 * ang_msg.data, 0.0 ),
                          rospy.Time.now(),
                          'ear_tilt_left',
                          'ear_pan_left' )


    


if __name__ == '__main__':
    tfs = tf_updater('servo_tf_updater')
    rospy.spin()
