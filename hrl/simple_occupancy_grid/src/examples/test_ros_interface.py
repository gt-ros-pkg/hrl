
import roslib; roslib.load_manifest('simple_occupancy_grid')
import rospy

from std_msgs.msg import Empty


if __name__ == '__main__':
    viz_cmd_pub = rospy.Publisher('/occupancy_grid_node/cmd/viz_simple', Empty)
    rospy.init_node('occupancy_grid_tester')

    raw_input('Hit ENTER to visualize')
    viz_cmd_pub.publish(Empty())




