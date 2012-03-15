

#
# Steps:
#   1. Run  bin/og_node
#   2. run rviz with the fixed frame = /occupancy_grid_frame
#   3. Run this node.

import roslib; roslib.load_manifest('simple_occupancy_grid')
import rospy

from std_msgs.msg import Empty
from hrl_srvs.srv import FloatArray_None


if __name__ == '__main__':
    viz_cmd_pub = rospy.Publisher('/occupancy_grid_node/cmd/viz_simple', Empty)
    rospy.init_node('occupancy_grid_tester')

    add_pts_srv = rospy.ServiceProxy('/occupancy_grid_node/srv/add_points_unstamped', FloatArray_None)

    raw_input('Hit ENTER to add points')

    pts = [0.6, 0., 1., 0.4, 0., 1., 0.5, 0.1, 1.]
    #pts = [0.27, -0.23, 0.]
    #pts = [0.4, 0., 0.]
    add_pts_srv(pts)


    raw_input('Hit ENTER to visualize')
    viz_cmd_pub.publish(Empty())




