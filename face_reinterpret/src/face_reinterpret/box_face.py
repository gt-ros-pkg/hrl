import roslib; roslib.load_manifest('face_reinterpret')
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
import rospy

class FaceBox:
    def __init__(self):
        rospy.Subscriber('/face_detector/faces_cloud', PointCloud, self.callback)
        self.box_pub = rospy.Publisher('/face_detector/faces_boxes', Marker)

    def callback(self, msg):
        if len(msg.points) <= 0:
            return
        m = Marker()
        m.header = msg.header
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = msg.points[0].x
        m.pose.position.y = msg.points[0].y
        m.pose.position.z = msg.points[0].z
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 0.5
        m.lifetime = rospy.Duration(10.0)
        print m
        self.box_pub.publish(m)
    

if __name__ == '__main__':
    rospy.init_node('face_box')
    f = FaceBox()
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
