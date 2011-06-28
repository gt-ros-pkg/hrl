
import numpy as np, math

import roslib; roslib.load_manifest('hrl_cody_arms')
import rospy
import hrl_lib.viz as hv

from visualization_msgs.msg import Marker



def vel_vec_rviz_marker(p, v):
    quat = hv.arrow_direction_to_quat(v)
    am = hv.single_marker(p, quat, 'arrow', 'torso_lift_link', m_id=1,
                          duration=0.1)
    return am



if __name__ == '__main__':
    import cody_arm_client as cac

    rospy.init_node('jacobian_tester')
    marker_pub = rospy.Publisher('/test_jacobian/viz/markers', Marker)

    robot = cac.CodyArmClient('r')
    robot.kinematics.set_tooltip(np.matrix([0., 0., -0.16]).T)
    while not rospy.is_shutdown():
        q = robot.get_joint_angles()
        rospy.sleep(0.1)
        if q != None:
            break
        else:
            rospy.loginfo('q is None')

    jt = 0
    point_jt_idx = 3

    rt = rospy.Rate(20)

    while not rospy.is_shutdown():
        q = robot.get_joint_angles()
        pos, _ = robot.kinematics.FK(q, point_jt_idx)
        J = robot.kinematics.Jacobian(q, pos)
        vel_vec = J[0:3, jt]
        marker_pub.publish(vel_vec_rviz_marker(pos, vel_vec))
        rt.sleep()




