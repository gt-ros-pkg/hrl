#! /usr/bin/python

import sys
from threading import Lock

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('actionlib')
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('pr2_controllers_msgs')
import roslib.substitution_args
import rospy
import rosbag
from std_srvs.srv import Empty, EmptyResponse
import actionlib

from geometry_msgs.msg import TransformStamped
from hrl_phri_2011.srv import LoadEllipsoidParams
from hrl_generic_arms.pose_converter import PoseConverter
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

class SetupRegistration(object):
    def __init__(self):
        rospy.loginfo("[setup_registration] SetupRegistration ready.")
        self.load_ell_params = rospy.ServiceProxy("/load_ellipsoid_params", LoadEllipsoidParams)
        self.head_point_sac = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',
                                                           PointHeadAction)
        self.head_point_sac.wait_for_server()
        self.lock = Lock()

    def run(self, req):
        ############################################################
        self.lock.acquire(False)

        shaving_side = rospy.get_param("/shaving_side")
        assert(shaving_side in ["r", "l"])

        filepath = "$(find hrl_rfh_fall_2011)/data/registration_ell_params_%s.pkl" % shaving_side
        ell_params_bag = rosbag.Bag(roslib.substitution_args.resolve_args(filepath), 'r')
        for topic, ell_param, time in ell_params_bag:
            cur_ell_param = ell_param
        self.load_ell_params(cur_ell_param)
        cur_ell_param.e_frame.__class__ = TransformStamped
        
        head_goal = PointHeadGoal()
        head_goal.target = PoseConverter.to_point_stamped_msg(cur_ell_param.e_frame)
        head_goal.target.point.z -= 0.15
        head_goal.target.header.stamp = rospy.Time()
        head_goal.min_duration = rospy.Duration(1.)
        head_goal.max_velocity = 1.
        self.head_point_sac.send_goal_and_wait(head_goal)

        self.lock.release()
        ############################################################

        return EmptyResponse()

def main():
    rospy.init_node("setup_registration")
    assert(len(sys.argv) > 1)
    sreg = SetupRegistration()
    if sys.argv[1] == "-s":
        rospy.Service("/setup_ell_registration", Empty, sreg.run)
        rospy.spin()
    elif sys.argv[1] == "-p":
        sreg.run(None)
        
if __name__ == "__main__":
    main()
