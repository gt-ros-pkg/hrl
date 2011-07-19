#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )
roslib.load_manifest( 'std_msgs' )

import rospy

from std_msgs.msg import Float64, Int8

import numpy as np

rospy.init_node( 'move_joint_node' )
topic = 'r_shoulder_pan_controller/command'

topic = 'r_forearm_roll_controller/command'

pub = rospy.Publisher( topic, Float64 )

update_pub = rospy.Publisher( '/joint_state_viz/update', Int8 )

soft_lower_limit = -np.pi
soft_upper_limit = np.pi

# soft_lower_limit = -2.1353981634
# soft_upper_limit = 0.564601836603

counter = 0

while not rospy.is_shutdown():
    
    counter = counter + 1
    print( '----- %d -----'%counter )
    
    pos = np.random.rand() * ( soft_upper_limit - soft_lower_limit ) + soft_lower_limit
    rospy.loginfo( 'move to pos %6.4f'%pos )
    
    update_pub.publish( 0 )
    rospy.sleep( 0.5 )
    pub.publish( pos )
    rospy.sleep( 1 )
    update_pub.publish( 1 )

    if counter == 500:
        break
