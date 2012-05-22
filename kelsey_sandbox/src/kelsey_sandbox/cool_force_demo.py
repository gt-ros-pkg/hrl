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
    rospy.init_node("cool_force_demo")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-f', '--force_mag', dest="force_mag", default=2,
                 help="Specify force magnitude.")
    p.add_option('-x', '--max_force', dest="max_force", default=-1,
                 help="Specify max force magnitude.")
    p.add_option('-c', '--compliance', dest="compliance", default=-1,
                 help="Compliance to maintain.")
    p.add_option('-t', '--tip_frame', dest="tip_frame", default="/l_gripper_tool_frame",
                 help="Set tip to this frame.")
    p.add_option('-z', '--zero_sensor', dest="zero_sensor", default=False,
                 action="store_true", help="Just zero the sensor.")
    p.add_option('-l', '--force_line', dest="force_line", default=False,
                 action="store_true", help="Move in a line with zero force.")
    p.add_option('-p', '--force_plane', dest="force_plane", default=False,
                 action="store_true", help="Move in a plane with zero force.")
    p.add_option('-o', '--force_point', dest="force_point", default=False,
                 action="store_true", help="Move about a point with zero torque.")
    p.add_option('-r', '--force_roll', dest="force_roll", default=False,
                 action="store_true", help="Move the wrist with zero torque.")
    p.add_option('-a', '--force_all', dest="force_all", default=False,
                 action="store_true", help="All DOF are trying to set zero force.")
    p.add_option('-n', '--force_none', dest="force_none", default=False,
                 action="store_true", help="Return to position control.")
    p.add_option('-k', '--kill_controller', dest="kill_controller", default=False,
                 action="store_true", help="Render controller dead.")
    (opts, args) = p.parse_args()

    arm = create_pr2_arm('l', PR2ArmHybridForce)
    rospy.sleep(1)

    if opts.zero_sensor:
        arm.zero_sensor()

    # reset arm
    arm.set_ep(arm.get_end_effector_pose(), 0)
    arm.set_force(6 * [0])

    fp_trans_major = 3
    fp_trans_minor = 1
    fp_rot = 0.1
    fi_trans_major = 0.002
    fi_trans_minor = 0.001
    fi_max_trans_major = 10
    fi_max_trans_minor = 5
    fi_rot = 0
    fi_max_rot = 0
    pp_trans = 1000
    pd_trans_major = 16
    pd_trans_minor = 4
    pp_rot = 120
    pd_rot = 0

    if opts.force_line:
        arm.set_force_max([float(opts.max_force), -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[fp_trans_major, fp_trans_minor, fp_trans_minor], 
                            p_rot=fp_rot, i_trans=[fi_trans_major, fi_trans_minor, fi_trans_minor],
                            i_max_trans=[fi_max_trans_major, fi_max_trans_minor, fi_max_trans_minor], 
                            i_rot=fi_rot, i_max_rot=fi_max_rot)
        arm.set_motion_gains(p_trans=[float(opts.compliance), pp_trans, pp_trans], 
                             d_trans=[pd_trans_major, pd_trans_minor, pd_trans_minor],
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions(['x'])
        arm.set_force([float(opts.force_mag), 0, 0, 0, 0, 0])
        arm.update_gains()
        return

    if opts.force_plane:
        arm.set_force_max([-1, float(opts.max_force), float(opts.max_force), -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[fp_trans_minor, fp_trans_major, fp_trans_major], 
                            p_rot=fp_rot, i_trans=[fi_trans_minor, fi_trans_major, fi_trans_major],
                            i_max_trans=[fi_max_trans_minor, fi_max_trans_major, fi_max_trans_major], 
                            i_rot=fi_rot, i_max_rot=fi_max_rot)
        arm.set_motion_gains(p_trans=[pp_trans, float(opts.compliance), float(opts.compliance)], 
                             d_trans=[pd_trans_minor, pd_trans_major, pd_trans_major], 
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions(['y', 'z'])
        arm.set_force([0, float(opts.force_mag), float(opts.force_mag), 0, 0, 0])
        arm.update_gains()
        return

    if opts.force_point:
        arm.set_force_max([-1, -1, -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[fp_trans_minor, fp_trans_minor, fp_trans_minor], 
                            p_rot=0.8, i_trans=[fi_trans_minor, fi_trans_minor, fi_trans_minor],
                            i_max_trans=[fi_max_trans_minor, fi_max_trans_minor, fi_max_trans_minor], 
                            i_rot=fi_rot, i_max_rot=fi_max_rot)
        arm.set_motion_gains(p_trans=[pp_trans, pp_trans, pp_trans], 
                             d_trans=[pd_trans_minor, pd_trans_minor, pd_trans_minor], 
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions([0, 0, 0, 1, 1, 1])
        arm.set_force([0, 0, 0, float(opts.force_mag), float(opts.force_mag), float(opts.force_mag)])
        arm.update_gains()
        return

    if opts.force_roll:
        arm.set_force_max([-1, -1, -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[fp_trans_minor, fp_trans_minor, fp_trans_minor], 
                            p_rot=1.8, i_trans=[fi_trans_minor, fi_trans_minor, fi_trans_minor],
                            i_max_trans=[fi_max_trans_minor, fi_max_trans_minor, fi_max_trans_minor], 
                            i_rot=fi_rot, i_max_rot=fi_max_rot)
        arm.set_motion_gains(p_trans=[pp_trans, pp_trans, pp_trans], 
                             d_trans=[pd_trans_minor, pd_trans_minor, pd_trans_minor], 
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions([0, 0, 0, 1, 0, 0])
        arm.set_force([0, 0, 0, float(opts.force_mag), 0, 0])
        arm.update_gains()
        return

    if opts.force_all:
        arm.set_force_max([-1, -1, -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=6, 
                            p_rot=1.8, i_trans=[fi_trans_major, fi_trans_major, fi_trans_major],
                            i_max_trans=[fi_max_trans_major, fi_max_trans_major, fi_max_trans_major], 
                            i_rot=fi_rot, i_max_rot=fi_max_rot)
        arm.set_motion_gains(p_trans=[pp_trans, pp_trans, pp_trans], 
                             d_trans=[pd_trans_major, pd_trans_major, pd_trans_major], 
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions([1, 1, 1, 1, 1, 1])
        arm.set_force([0, 0, 0, 0, 0, 0])
        arm.update_gains()
        return

    if opts.force_none:
        arm.set_force_max([-1, -1, -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[0, 0, 0], 
                            p_rot=0, i_trans=[0, 0, 0],
                            i_max_trans=[0, 0, 0], 
                            i_rot=0, i_max_rot=0)
        arm.set_motion_gains(p_trans=[pp_trans, pp_trans, pp_trans], 
                             d_trans=[pd_trans_minor, pd_trans_minor, pd_trans_minor], 
                             p_rot=pp_rot, d_rot=pd_rot)
        arm.set_force_directions([0, 0, 0, 0, 0, 0])
        arm.set_force([0, 0, 0, 0, 0, 0])
        arm.update_gains()
        return

    if opts.kill_controller:
        arm.set_force_max([-1, -1, -1, -1, -1, -1, -1])
        arm.set_tip_frame(opts.tip_frame)
        arm.set_force_gains(p_trans=[0, 0, 0], 
                            p_rot=0, i_trans=[0, 0, 0],
                            i_max_trans=[0, 0, 0], 
                            i_rot=0, i_max_rot=0)
        arm.set_motion_gains(p_trans=0, 
                             d_trans=0, 
                             p_rot=0, d_rot=0)
        arm.set_force_directions([0, 0, 0, 0, 0, 0])
        arm.set_force([0, 0, 0, 0, 0, 0])
        arm.update_gains()
        return

if __name__ == "__main__":
    main()
