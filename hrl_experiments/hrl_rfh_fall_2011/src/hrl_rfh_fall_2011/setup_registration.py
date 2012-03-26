#! /usr/bin/python

import sys
from threading import Lock

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
import roslib.substitution_args
import rospy
import rosbag
from std_srvs.srv import Empty, EmptyResponse

from hrl_phri_2011.srv import LoadEllipsoidParams

class SetupRegistration(object):
    def __init__(self):
        rospy.loginfo("[setup_registration] SetupRegistration ready.")
        self.load_ell_params = rospy.ServiceProxy("/load_ellipsoid_params", LoadEllipsoidParams)
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
        print cur_ell_param, type(cur_ell_param)
        self.load_ell_params(cur_ell_param)

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
