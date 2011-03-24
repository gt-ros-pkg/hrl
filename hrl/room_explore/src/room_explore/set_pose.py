#!/usr/bin/python

import roslib
roslib.load_manifest('room_explore')
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

# header: 
#   seq: 7
#   stamp: 
#     secs: 0
#     nsecs: 0
#   frame_id: /map
# pose: 
#   pose: 
#     position: 
#       x: -0.324687451124
#       y: 0.183924600482
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -0.054315002133
#       w: 0.998523850763
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

if __name__ == '__main__':
    rospy.init_node( 'pose_setter' )

    pub = rospy.Publisher( '/initialpose', PoseWithCovarianceStamped )
    rospy.sleep( 1.0 )

    ps = PoseWithCovarianceStamped()
    ps.header.stamp = rospy.Time(0)
    ps.header.frame_id = '/base_link'
    ps.pose.pose.orientation.w = 1.0
    ps.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.01, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0]
    print ps
    pub.publish( ps )
    

    
