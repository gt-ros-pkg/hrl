#! /usr/bin/python
import roslib
roslib.load_manifest('hrl_generic_arms')
import rospy
import tf
import tf.transformations as tf_trans
from hrl_generic_arms.pose_converter import PoseConverter

def main():
    rospy.init_node("tf_link_flipper")

    child_frame = rospy.get_param("~child_frame")
    parent_frame = rospy.get_param("~parent_frame")
    link_frame = rospy.get_param("~link_frame")
    rate = rospy.get_param("~rate", 100)
    link_trans = rospy.get_param("~link_transform")

    l_B_c = PoseConverter.to_homo_mat(link_trans['pos'], link_trans['quat'])

    tf_broad = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    rospy.sleep(1)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        time = rospy.Time.now()
        tf_listener.waitForTransform(child_frame, parent_frame, rospy.Time(0), rospy.Duration(1))
        pos, quat = tf_listener.lookupTransform(child_frame, parent_frame, rospy.Time(0))
        c_B_p = PoseConverter.to_homo_mat(pos, quat)
        l_B_p = l_B_c * c_B_p
        tf_pos, tf_quat = PoseConverter.to_pos_quat(l_B_p)
        tf_broad.sendTransform(tf_pos, tf_quat, time, parent_frame, link_frame)
        r.sleep()

        

if __name__ == "__main__":
    main()
