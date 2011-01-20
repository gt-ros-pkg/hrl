
import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

import hrl_lib.transforms as tr

from planning_environment_msgs.srv import GetStateValidity
from planning_environment_msgs.srv import GetStateValidityRequest
from sensor_msgs.msg import JointState


def joint_states_cb(data):
    joint_nm_list = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint',
                     'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
                     'r_forearm_roll_joint', 'r_wrist_flex_joint',
                     'r_wrist_roll_joint']
    ang_list = []
    for i, jt_nm in enumerate(joint_nm_list):
        idx = data.name.index(jt_nm)
        ang = tr.angle_within_mod180(data.position[idx])
        ang_list.append(ang)

    req = GetStateValidityRequest()
    req.robot_state.joint_state.name = joint_nm_list
    req.robot_state.joint_state.position = ang_list

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions = True

    print 'Before Service call'
    res = get_state_validity.call(req)
    print 'After'

    if res.error_code.val == res.error_code.SUCCESS:
        rospy.loginfo('Requested state is not in collision')
    else:
        rospy.loginfo('Requested state is in collision. Error code: %d'%(res.error_code.val))



if __name__ == '__main__':
    rospy.init_node('get_state_validity_python')

    srv_nm = 'environment_server_right_arm/get_state_validity'
    rospy.wait_for_service(srv_nm)
    get_state_validity = rospy.ServiceProxy(srv_nm, GetStateValidity,
                                            persistent=True)

    rospy.Subscriber('/joint_states', JointState, joint_states_cb)

    rospy.spin()
    get_state_validity.close()

    


