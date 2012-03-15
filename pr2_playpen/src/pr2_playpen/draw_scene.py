import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
import rospy
from visualization_msgs.msg import Marker
import numpy as np

class SceneDraw:
    def __init__(self, topic='contact_visualization', node='contact_sim', frame='/world_frame'):
        self.pub = rospy.Publisher(topic+'_marker', Marker)
        self.frame = frame
        self.Marker = Marker

    def pub_body(self, pos, quat, scale, color, num, shape, text = ''):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = num

        marker.type = shape
        marker.action = Marker.ADD

        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration()
        marker.text = text
        self.pub.publish(marker)

    def get_rot_mat(self, rot):
#        rot_mat = np.matrix([[rot[0], rot[3], rot[6]],[rot[1], rot[4], rot[7]], [rot[2], rot[5], rot[8]]])
        rot_mat = np.matrix([[rot[0], rot[4], rot[8]],[rot[1], rot[5], rot[9]], [rot[2], rot[6], rot[10]]])
        return rot_mat



                        
