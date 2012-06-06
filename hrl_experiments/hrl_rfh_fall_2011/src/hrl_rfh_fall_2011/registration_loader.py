
import numpy as np

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('hrl_ellipsoidal_control')
roslib.load_manifest('hrl_head_tracking')
import rospy
import rosbag

from hrl_ellipsoidal_control.msg import EllipsoidParams
from hrl_rfh_fall_2011.srv import InitializeRegistration

from hrl_head_tracking.srv import HeadRegistration
from hrl_generic_arms.pose_converter import PoseConverter

class RegistrationLoader(object):
    def __init__(self, e_params):
        self.e_params = e_params
        self.init_reg_srv = rospy.Service("/initialize_registration", InitializeRegistration, 
                                          self.init_reg_cb)
        self.head_registration = rospy.ServiceProxy("/head_registration", HeadRegistration)
        self.load_ell_params = rospy.ServiceProxy("/load_ellipsoid_params", LoadEllipsoidParams)

    def init_reg_cb(self, req):
        head_reg = PoseConverter.to_homo_mat(self.head_registration(req.u, req.v))
        ell_reg = PoseConverter.to_homo_mat(Transform(self.e_params.e_frame.transform.translation,
                                                      self.e_params.e_frame.transform.rotation))
        reg_e_params = EllipsoidParams()
        reg_e_params.e_frame = PoseConverter.to_tf_stamped_msg(head_reg * ell_reg)
        reg_e_params.e_frame.header.frame_id = e_params.e_frame.header.frame_id
        reg_e_params.height = e_params.height
        reg_e_params.E = e_params.E
        self.load_ell_params(reg_e_params)

def main():
    rospy.init_node("registration_loader")
    bag = rosbag.Bag(sys.argv[1], 'r')
    for topic, msg, ts in bag.read_messages():
        e_params = msg
    rl = RegistrationLoader(e_params)
    rospy.spin()
        
if __name__ == "__main__":
    main()
