
import roslib; roslib.load_manifest('hrl_pr2_kinematics_tutorials')
import rospy

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction
from trajectory_msgs.msg import JointTrajectoryPoint

class HRL_PR2():

    def __init__(self):
        self.joint_action_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        self.gripper_action_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.joint_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()

    ## go to a joint configuration.
    # @param q - list of 7 joint angles in RADIANS.
    # @param duration - how long (SECONDS) before reaching the joint angles.
    def go_jointangles(self, q, duration):
        jtg = JointTrajectoryGoal()
        jtg.trajectory.joint_names = ['r_shoulder_pan_joint',
                           'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                           'r_elbow_flex_joint', 'r_forearm_roll_joint',
                           'r_wrist_flex_joint', 'r_wrist_roll_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = q
        jtp.velocities = [0 for i in range(len(q))]
        jtp.accelerations = [0 for i in range(len(q))]
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)


if __name__ == '__main__':

    rospy.init_node('hrl_pr2', anonymous = True)
    rospy.logout('hrl_pr2: ready')

    hrl_pr2 = HRL_PR2()
    q = [0, 0, 0, 0, 0, 0, 0]
    hrl_pr2.go_jointangles(q, duration = 1.)

    rospy.spin()




