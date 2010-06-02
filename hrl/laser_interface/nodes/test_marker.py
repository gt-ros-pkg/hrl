import roslib
roslib.load_manifest('laser_interface')
import rospy

from visualization_msgs.msg import Marker

rospy.init_node('marker_publish')
pub = rospy.Publisher('test_marker', Marker)

m = Marker()
m.header.frame_id = '/wide_stereo_optical' #check this
m.header.stamp = rospy.get_rostime()
m.ns = 'test_shapes'
m.id = 0
m.type = Marker.SPHERE
m.action = Marker.ADD
m.pose.position.x = 0.0
m.pose.position.y = 0.0
m.pose.position.z = 0.0
m.pose.orientation.x = 0.0
m.pose.orientation.y = 0.0
m.pose.orientation.z = 0.0
m.pose.orientation.w = 1.0
m.scale.x = 1.0
m.scale.y = 1.0
m.scale.z = 1.0
m.color.r = 0.0
m.color.g = 1.0
m.color.b = 0.0
m.color.a = 1.0
m.lifetime = rospy.Duration()

while not rospy.is_shutdown():
    pub.publish(m)
    rospy.sleep(1.0)










