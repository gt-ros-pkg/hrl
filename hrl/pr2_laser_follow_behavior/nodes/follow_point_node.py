import roslib
roslib.load_manifest('pr2_laser_follow_behavior')
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import hrl_lib.tf_utils as tfu
import tf

class FollowPointBehavior:

    def __init__(self):
        rospy.Subscriber('cursor3dcentered', PointStamped, follow_point_cb)
        self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
       self.tflistener = tf.TransformListener()

    def follow_point_cb(self, point_stamped):
        point_head = point_stamped.point

        base_T_head = tfu.transform('/base_link', point_stamped.header.frame_id, self.tflistener)
        point_mat_head = tfu.translation_matrix([point.x, point.y, point.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, o_base = tfu.matrix_as_tf(point_mat_base)
        x = t_base[0]
        y = t_base[1]
        angle = math.atan2(y, x)

        ps = PoseStamped()
        ps.header.frame_id = '/base_link'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.

        q = tf.transformations.quaternion_from_euler(angle, 0, 0)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        self.move_pub.publish(ps)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('follow_point_node', anonymous=True)


#Subscribes to laser point
#sends point out to move_base_simple/goal
#header: 
#  seq: 0
#  stamp: 
#    secs: 1278624411
#    nsecs: 326550373
#  frame_id: /map
#pose: 
#  position: 
#    x: 1.49216187
#    y: -0.0629254132509
#    z: 0.0
#  orientation: 
#    x: 0.0
#    y: 0.0
#    z: 0.127143523912
#    w: 0.991884330115

