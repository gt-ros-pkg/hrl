
import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import CollisionObjectOperation
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose


def add_cylinder(id, bottom, radius, height, frameid, pub):
    cylinder_object = CollisionObject()
    cylinder_object.id = id
    cylinder_object.operation.operation = CollisionObjectOperation.ADD
    cylinder_object.header.frame_id = frameid
    cylinder_object.header.stamp = rospy.Time.now()

    shp = Shape()
    shp.type = Shape.CYLINDER
    shp.dimensions = [0, 0]
    shp.dimensions[0] = radius
    shp.dimensions[1] = height

    pose = Pose()
    pose.position.x = bottom[0]
    pose.position.y = bottom[1]
    pose.position.z = bottom[2] + height/2.
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    cylinder_object.shapes.append(shp)
    cylinder_object.poses.append(pose)

    pub.publish(cylinder_object)


if __name__ == '__main__':
    rospy.init_node('add_cylinder_python')

    pub = rospy.Publisher('collision_object', CollisionObject)
    rospy.sleep(2.)

    add_cylinder('pole', (0.6, -0.6, 0.), 0.1, 0.75, 'base_link', pub)
    rospy.loginfo('Should have published')
    rospy.sleep(2.)



