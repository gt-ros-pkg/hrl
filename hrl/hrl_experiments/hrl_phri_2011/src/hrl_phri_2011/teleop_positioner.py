#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
import rospy
import rosbag

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Bool, Float32, Float64MultiArray
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3
from actionlib_msgs.msg import GoalStatus

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

def main():
    rospy.init_node("teleop_positioner")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-r', '--rate', dest="rate", default=10,
                 help="Set rate.")
    (opts, args) = p.parse_args()

    arm = create_pr2_arm('l', PR2ArmHybridForce)
    rospy.sleep(0.1)

    arm.zero_sensor()
    cur_pose = arm.get_end_effector_pose()
    arm.set_ep(cur_pose, 1)
    arm.set_force_directions([])
    arm.set_force_gains(p_trans=[3, 1, 1], p_rot=0.5, i_trans=[0.002, 0.001, 0.001], i_max_trans=[10, 5, 5], i_rot=0, i_max_rot=0)
    arm.set_motion_gains(p_trans=400, d_trans=[16, 10, 10], p_rot=[10, 10, 10], d_rot=0)
    arm.set_tip_frame("/l_gripper_tool_frame")
    arm.update_gains()
    arm.set_force(6 * [0])

    r = rospy.Rate(float(opts.rate))
    while not rospy.is_shutdown():
        ep_pose = arm.get_ep()
        cur_pose = arm.get_end_effector_pose()
        err_ep = arm.ep_error(cur_pose, ep_pose)
        if np.linalg.norm(err_ep[0:3]) > 0.012 or np.linalg.norm(err_ep[3:]) > np.pi / 8.:
            arm.set_ep(cur_pose, 1)
        r.sleep()
    cur_pose = arm.get_end_effector_pose()
    arm.set_ep(cur_pose, 1)
    q = arm.get_joint_angles()
    q_posture = q.tolist()[0:3] + 4 * [9999]
    arm.set_posture(q_posture)
    arm.set_motion_gains(p_trans=400, d_trans=[16, 10, 10], p_rot=[20, 50, 50], d_rot=0)
    arm.update_gains()
    print PoseConverter.to_pos_quat(cur_pose)
    pkg_dir = roslib.rospack.rospackexec(["find", "hrl_phri_2011"])
    bag = rosbag.Bag(pkg_dir + "/data/saved_teleop_pose.bag", 'w')
    bag.write("/teleop_pose", PoseConverter.to_pose_msg(cur_pose))
    q_posture_msg = Float64MultiArray()
    q_posture_msg.data = q_posture
    bag.write("/teleop_posture", q_posture_msg)
    bag.close()

if __name__ == "__main__":
    main()
