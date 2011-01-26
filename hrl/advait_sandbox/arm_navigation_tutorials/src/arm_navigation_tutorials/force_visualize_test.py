
import sys
import numpy as np, math

import add_cylinder as ac
import online_collision_detection as ocd

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import rospy
from mapping_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker

import hrl_lib.transforms as tr
import hrl_lib.viz as hv

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa

roslib.load_manifest('force_torque') # hack by Advait
import force_torque.FTClient as ftc
import tf

class object_ft_sensors():
    def __init__(self):
        self.obj1_ftc = ftc.FTClient('force_torque_ft2')
        self.tf_lstnr = tf.TransformListener()

    def get_forces(self, bias = True):
        # later I might be looping over all the different objects,
        # returning a dictionary of <object_id: force_vector>
        f = self.obj1_ftc.read(without_bias = not bias)
        f = f[0:3, :]

        trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                    '/ft2',
                                                    rospy.Time(0))
        rot = tr.quaternion_to_matrix(quat)
        f = rot * f
        return -f # the negative is intentional (Advait, Nov 24. 2010.)

    def bias_fts(self):
       self.obj1_ftc.bias()


def get_arrow_text_markers(p, f, frame, m_id, duration):
    t_now = rospy.Time.now()
    q = hv.arrow_direction_to_quat(f)
    arrow_len = np.linalg.norm(f) * 0.04

    scale = (arrow_len, 0.2, 0.2)
    m1 = hv.single_marker(p, q, 'arrow', frame, scale, m_id = m_id,
                          duration = duration)
    m1.header.stamp = t_now

    m2 = hv.single_marker(p, q, 'text_view_facing', frame,
                          (0.1, 0.1, 0.1), m_id = m_id+1,
                          duration = duration, color=(1.,0.,0.,1.))
    m2.text = '%.1f N'%(np.linalg.norm(f))
    m2.header.stamp = t_now
    return m1, m2


if __name__ == '__main__':
    rospy.init_node('force_visualize_test')
    marker_pub = rospy.Publisher('/skin/viz_marker', Marker)

    fts = object_ft_sensors()
    fts.bias_fts()

    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    while not rospy.is_shutdown():
        f = fts.get_forces()
        p, r = pr2_arms.end_effector_pos(arm)
        m1, m2 = get_arrow_text_markers(p, f, 'torso_lift_link', 0, 1.)
        marker_pub.publish(m1)
        marker_pub.publish(m2)
        rospy.sleep(0.1)









