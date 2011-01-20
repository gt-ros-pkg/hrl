
import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

from mapping_msgs.msg import CollisionObject
from mapping_msgs.msg import CollisionObjectOperation
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose


if __name__ == '__main__':
    rospy.init_node('add_cylinder_python')

    pub = rospy.Publisher('collision_object', CollisionObject)
    rospy.sleep(2.)

    cylinder_object = CollisionObject()
    cylinder_object.id = 'pole'
    cylinder_object.operation.operation = CollisionObjectOperation.ADD
    cylinder_object.header.frame_id = 'base_link'
    cylinder_object.header.stamp = rospy.Time.now()

    shp = Shape()
    shp.type = Shape.CYLINDER
    shp.dimensions = [0, 0]
    shp.dimensions[0] = .1
    shp.dimensions[1] = .75

    pose = Pose()
    pose.position.x = .6
    pose.position.y = -.6
    pose.position.z = .375
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    cylinder_object.shapes.append(shp)
    cylinder_object.poses.append(pose)

    pub.publish(cylinder_object)
    rospy.loginfo('Should have published')
    rospy.sleep(2.)



