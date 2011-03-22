
import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

import hrl_lib.transforms as tr

from planning_environment_msgs.srv import GetStateValidity
from planning_environment_msgs.srv import GetStateValidityRequest
from sensor_msgs.msg import JointState

roslib.load_manifest('hrl_pr2_lib')
import hrl_pr2_lib.pr2_arms as pa


class online_collision_detector():

    def __init__(self):
        srv_nm = 'environment_server_right_arm/get_state_validity'
        rospy.wait_for_service(srv_nm)
        self.state_validator = rospy.ServiceProxy(srv_nm, GetStateValidity,
                                                persistent=True)

    def check_validity(self, pr2_arms, arm):
        q = pr2_arms.get_joint_angles(arm)

        joint_nm_list = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint',
                         'r_upper_arm_roll_joint', 'r_elbow_flex_joint',
                         'r_forearm_roll_joint', 'r_wrist_flex_joint',
                         'r_wrist_roll_joint']

        req = GetStateValidityRequest()
        req.robot_state.joint_state.name = joint_nm_list
        req.robot_state.joint_state.position = q

        req.robot_state.joint_state.header.stamp = rospy.Time.now()
        req.check_collisions = True

        res = self.state_validator.call(req)
        return res
    

if __name__ == '__main__':
    rospy.init_node('get_state_validity_python')

    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm
    col_det = online_collision_detector()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        res = col_det.check_validity(pr2_arms, arm)
        if res.error_code.val == res.error_code.SUCCESS:
            rospy.loginfo('Requested state is not in collision')
        else:
            rospy.loginfo('Requested state is in collision. Error code: %d'%(res.error_code.val))



