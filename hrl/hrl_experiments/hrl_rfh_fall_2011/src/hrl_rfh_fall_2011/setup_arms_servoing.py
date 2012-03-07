#! /usr/bin/python

import sys
import numpy as np
import copy

import roslib
roslib.load_manifest('hrl_pr2_arms')
roslib.load_manifest('hrl_rfh_summer_2011')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('kelsey_sandbox')
import rospy

import smach
import actionlib
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from std_srvs.srv import Empty, EmptyResponse

from hrl_rfh_fall_2011.sm_register_head_ellipse import SMEllipsoidRegistration
from hrl_trajectory_playback.srv import TrajPlaybackSrv, TrajPlaybackSrvRequest
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor
from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from kelsey_sandbox.srv import TrajectoryPlay, TrajectoryPlayRequest, TrajectoryPlayResponse

class SetupArmsServoing():
        def __init__(self):
            self.traj_playback = rospy.ServiceProxy('/trajectory_playback', TrajectoryPlay)
            rospy.loginfo("[setup_arms_servoing] SetupArmsServoing ready.")

        def run(self, req):
            traj = TrajectoryPlayRequest()
            traj.mode = traj.SETUP_AND_TRAJ
            traj.reverse = False
            traj.setup_velocity = 0.1
            traj.traj_rate_mult = 0.8

            l_traj = copy.copy(traj)
            l_traj.filepath = "$(find hrl_rfh_fall_2011)/data/l_arm_servo_setup.pkl"
            l_traj.blocking = False

            r_traj = copy.copy(traj)
            r_traj.filepath = "$(find hrl_rfh_fall_2011)/data/r_arm_servo_setup.pkl"
            r_traj.blocking = True

            self.traj_playback(l_traj)
            self.traj_playback(r_traj)
            return EmptyResponse()

def main():
    rospy.init_node("setup_arms_servoing")
    assert(len(sys.argv) > 1)
    sas = SetupArmsServoing()
    if sys.argv[1] == "-s":
        rospy.Service("/pr2_ar_servo/arms_setup", Empty, sas.run)
        rospy.spin()
    elif sys.argv[1] == "-p":
        sas.run(None)
        
if __name__ == "__main__":
    main()
