
import roslib; roslib.load_manifest('simple_occupancy_grid')
import rospy

from std_msgs.msg import Empty
from hrl_srvs.srv import FloatArray_None


if __name__ == '__main__':
    viz_cmd_pub = rospy.Publisher('/occupancy_grid_node/cmd/viz_simple', Empty)
    rospy.init_node('occupancy_grid_tester')

    add_pts_srv = rospy.ServiceProxy('/occupancy_grid_node/srv/add_points_unstamped', FloatArray_None)

    raw_input('Hit ENTER to visualize')
    viz_cmd_pub.publish(Empty())


    raw_input('Hit ENTER to send additional points')

    pts = [0.6, 0., 1., 0.4, 0., 1., 0.5, 0.1, 1.]
    add_pts_srv(pts)


    raw_input('Hit ENTER to visualize')
    viz_cmd_pub.publish(Empty())




