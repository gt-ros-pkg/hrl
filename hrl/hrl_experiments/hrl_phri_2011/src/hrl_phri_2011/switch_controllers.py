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
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from actionlib_msgs.msg import GoalStatus

from hrl_generic_arms.pose_converter import PoseConverter
from hrl_pr2_arms.pr2_arm import create_pr2_arm
from hrl_pr2_arms.pr2_arm_hybrid import PR2ArmHybridForce

def main():
    rospy.init_node("switch_controller")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-s', '--stiff', dest="stiff", default=False,
                 action="store_true", help="Enable stiff controller.")
    p.add_option('-f', '--force', dest="force", default=False,
                 action="store_true", help="Enable force controller.")
    p.add_option('-m', '--force_mag', dest="force_mag", default=2,
                 help="Specify force magnitude.")
    p.add_option('-x', '--max_force', dest="max_force", default=-1,
                 help="Specify max force magnitude.")
    p.add_option('-i', '--impedance', dest="impedance", default=False,
                 action="store_true", help="Enable impedance controller.")
    p.add_option('-c', '--compliance', dest="compliance", default=-1,
                 help="Enable impedance controller.")
    p.add_option('-t', '--tip_frame', dest="tip_frame", default="/l_gripper_tool_frame",
                 help="Set tip to this frame.")
    p.add_option('-z', '--zero_sensor', dest="zero_sensor", default=False,
                 action="store_true", help="Just zero the sensor.")
    p.add_option('-r', '--reset_pose', dest="reset_pose", default=False,
                 action="store_true", help="Use the saved position in the data file.")
    (opts, args) = p.parse_args()

    arm = create_pr2_arm('l', PR2ArmHybridForce)
    rospy.sleep(0.1)

    # reset arm
    arm.zero_sensor()
    if opts.zero_sensor:
        return
    arm.set_force(6 * [0])

    # 
    if opts.reset_pose:
        pkg_dir = roslib.rospack.rospackexec(["find", "hrl_phri_2011"])
        bag = rosbag.Bag(pkg_dir + "/data/saved_teleop_pose.bag", 'r')
        for topic, msg, stamp in bag.read_messages("/teleop_pose"):
            pose = PoseConverter.to_pos_rot([msg.position.x, msg.position.y, msg.position.z],
                                            [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                             msg.orientation.w])
        for topic, msg, stamp in bag.read_messages("/teleop_posture"):
            posture = msg.data
        bag.close()
        arm.set_posture(posture)
        i_poses = arm.interpolate_ep(arm.get_end_effector_pose(), pose, 100)
        for cur_pose in i_poses:
            arm.set_ep(cur_pose, 1)
            rospy.sleep(0.1)
        return

    # set common parameters
    arm.set_force_max([float(opts.max_force), -1, -1, -1, -1, -1])
    arm.set_tip_frame(opts.tip_frame)

    if opts.stiff:
        compliance = float(opts.compliance)
        if compliance < 0:
            compliance = 1300
#arm.set_force_gains(p_trans=[1, 1, 1], p_rot=0.1, i_trans=[0.002, 0.001, 0.001], i_max_trans=[10, 5, 5], i_rot=0, i_max_rot=0)
        arm.set_force_gains(p_trans=[1, 0, 0], p_rot=0.1, i_trans=[0.002, 0, 0], i_max_trans=[10, 0, 0], i_rot=0, i_max_rot=0)
        arm.set_motion_gains(p_trans=[compliance, 1300, 1300], d_trans=[16, 10, 10], p_rot=120, d_rot=0)
        arm.set_force_directions([])
        arm.set_force(6 * [0])
    elif opts.impedance:
        compliance = float(opts.compliance)
        if compliance < 0:
            compliance = 80
        arm.set_force_gains(p_trans=[3, 1, 1], p_rot=0.1, i_trans=[0.002, 0.001, 0.001], i_max_trans=[10, 5, 5], i_rot=0, i_max_rot=0)
        arm.set_motion_gains(p_trans=[compliance, 1300, 1300], d_trans=[16, 10, 10], p_rot=120, d_rot=0)
        arm.set_force_directions([])
        arm.set_force(6 * [0])
    elif opts.force:
        arm.set_force_gains(p_trans=[3, 1, 1], p_rot=0.1, i_trans=[0.002, 0.001, 0.001], i_max_trans=[10, 5, 5], i_rot=0, i_max_rot=0)
        arm.set_motion_gains(p_trans=[float(opts.compliance), 1300, 1300], d_trans=[16, 10, 10], p_rot=120, d_rot=0)
        arm.set_force_directions(['x'])
        arm.set_force([float(opts.force_mag), 0, 0, 0, 0, 0])
    else:
        p.print_help()
        return
    arm.update_gains()

if __name__ == "__main__":
    main()
