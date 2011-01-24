
import sys
import numpy as np, math

import add_cylinder as ac
import online_collision_detection as ocd

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import rospy
from mapping_msgs.msg import CollisionObject
from visualization_msgs.msg import Marker

import hrl_lib.viz as hv

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa



if __name__ == '__main__':
    rospy.init_node('force_visualize_test')
    marker_pub = rospy.Publisher('/skin/viz_marker', Marker)

    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    direc_list = [(1.,0.,0.), (0.,1.,0.), (0.,0.,1.), (-1,0.,0.),
                  (0.,-1,0.), (0.,0.,-1.), (1.,1.,0), (-1.,-1,-1)]
    i = 0

    while not rospy.is_shutdown():
        #f = pr2_arms.get_wrist_force(arm)
        #print 'force:', f.A1
        rospy.sleep(0.1)

        p, r = pr2_arms.end_effector_pos(arm)
        
        idx = i % len(direc_list)
        i += 1
        d = np.matrix(direc_list[idx]).T
        q = hv.arrow_direction_to_quat(d)
        m = hv.single_marker(p, q, 'arrow', 'torso_lift_link', m_id=idx)
        t_now = rospy.Time.now()
        m.header.stamp = t_now
        marker_pub.publish(m)









