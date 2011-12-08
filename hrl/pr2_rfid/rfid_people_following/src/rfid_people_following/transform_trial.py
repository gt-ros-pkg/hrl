
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest( 'move_base_msgs' )
roslib.load_manifest('tf')
roslib.load_manifest('std_srvs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')
roslib.load_manifest('rfid_people_following')
import rospy

import tf
import tf.transformations as tft
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from std_srvs.srv import Empty

rospy.init_node( 'trial' )

import sim_capture

bp = sim_capture.BasePose()

# listener = tf.TransformListener()
# listener.waitForTransform('/base_link', '/odom_combined',
#                           rospy.Time(0), timeout = rospy.Duration(100) )
# ps = PointStamped()
# ps.header.frame_id = '/base_link'

# while True:
#     ps.header.stamp = rospy.Time(0)
#     ps_map = listener.transformPoint( '/odom_combined', ps )
#     print ps_map.point.x, ps_map.point.y
#     rospy.sleep( 0.2 )
# #     try:
# #         ps_map = self.listener.transformPoint( '/odom_combined', ps )
# #         self.pts.append([ ps_map.point.x, ps.map.point.y ])
# #     except:
# #         rospy.logout( 'sim_capture: Failed transform.' )
# #         pass

        
