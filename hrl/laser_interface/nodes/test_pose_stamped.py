import roslib
roslib.load_manifest('laser_interface')
import rospy

from geometry_msgs.msg import PointStamped

rospy.init_node('marker_publish')
pub = rospy.Publisher('test_marker', PointStamped)

ps = PointStamped()
ps.header.stamp = rospy.get_rostime()
ps.header.frame_id = '/wide_stereo_optical'
ps.point.x = 1.0
ps.point.y = 2.0
ps.point.z = 3.0

while not rospy.is_shutdown():
    pub.publish(ps)
    rospy.sleep(1.0)
