
import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

from planning_environment_msgs.srv import GetStateValidity
from planning_environment_msgs.srv import GetStateValidityRequest


if __name__ == '__main__':
    rospy.init_node('get_state_validity_python')

    srv_nm = 'environment_server_right_arm/get_state_validity'
    rospy.wait_for_service(srv_nm)
    get_state_validity = rospy.ServiceProxy(srv_nm, GetStateValidity)

    req = GetStateValidityRequest()
    req.robot_state.joint_state.name = ['r_shoulder_pan_joint',
                                        'r_shoulder_lift_joint',
                                        'r_upper_arm_roll_joint',
                                        'r_elbow_flex_joint',
                                        'r_forearm_roll_joint',
                                        'r_wrist_flex_joint',
                                        'r_wrist_roll_joint']
    req.robot_state.joint_state.position = [0.] * 7
    req.robot_state.joint_state.position[0] = 0.4
    req.robot_state.joint_state.position[3] = -0.4

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions = True

    res = get_state_validity.call(req)
    
    if res.error_code.val == res.error_code.SUCCESS:
        rospy.loginfo('Requested state is not in collision')
    else:
        rospy.loginfo('Requested state is in collision. Error code: %d'%(res.error_code.val))



