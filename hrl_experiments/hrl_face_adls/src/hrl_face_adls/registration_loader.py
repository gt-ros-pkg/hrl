#! /usr/bin/python

import sys
import numpy as np

import roslib
roslib.load_manifest('hrl_face_adls')
import rospy
import rosbag
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Pose

from hrl_ellipsoidal_control.msg import EllipsoidParams
from hrl_ellipsoidal_control.srv import LoadEllipsoidParams
from hrl_face_adls.srv import InitializeRegistration, InitializeRegistrationResponse
from hrl_head_tracking.srv import HeadRegistration
from hrl_generic_arms.pose_converter import PoseConverter

class RegistrationLoader(object):
    def __init__(self, e_params_r, e_params_l):
        self.e_params_r = e_params_r
        self.e_params_l = e_params_l
        self.init_reg_srv = rospy.Service("/initialize_registration", InitializeRegistration, 
                                          self.init_reg_cb)
        self.head_registration_r = rospy.ServiceProxy("/head_registration_r", HeadRegistration) # TODO
        self.head_registration_l = rospy.ServiceProxy("/head_registration_l", HeadRegistration) # TODO
        self.load_ell_params = rospy.ServiceProxy("/load_ellipsoid_params", LoadEllipsoidParams)
        self.feedback_pub = rospy.Publisher("/feedback", String)

    def publish_feedback(self, msg):
        rospy.loginfo("[registration_loader] %s" % msg)
        self.feedback_pub.publish(msg)

    def init_reg_cb(self, req):
        # TODO REMOVE THIS SHAVING SIDE MESS
        self.shaving_side = rospy.get_param("/shaving_side", 'r')
        if self.shaving_side == 'r':
            e_params = self.e_params_r
            head_registration = self.head_registration_r
        else:
            e_params = self.e_params_l
            head_registration = self.head_registration_l
        # TODO

        try:
            head_reg_tf = head_registration(req.u, req.v).tf_reg
        except:
            self.publish_feedback("Registration failed.")
            return None

        head_reg_mat = PoseConverter.to_homo_mat(head_reg_tf)
        ell_reg = PoseConverter.to_homo_mat(Transform(e_params.e_frame.transform.translation,
                                                      e_params.e_frame.transform.rotation))
        reg_e_params = EllipsoidParams()
        reg_e_params.e_frame = PoseConverter.to_tf_stamped_msg(head_reg_mat**-1 * ell_reg)
        reg_e_params.e_frame.header.frame_id = head_reg_tf.header.frame_id
        reg_e_params.height = e_params.height
        reg_e_params.E = e_params.E
        self.load_ell_params(reg_e_params)

        if self.shaving_side == 'r':
            self.publish_feedback("Registered head using right cheek model, please visually confirm.")
        else:
            self.publish_feedback("Registered head using left cheek model, please visually confirm.")
        return InitializeRegistrationResponse()

def main():
    rospy.init_node("registration_loader")
    bag_r = rosbag.Bag(sys.argv[1], 'r')
    for topic, msg, ts in bag_r.read_messages():
        e_params_r = msg
    bag_l = rosbag.Bag(sys.argv[2], 'r')
    for topic, msg, ts in bag_l.read_messages():
        e_params_l = msg
    rl = RegistrationLoader(e_params_r, e_params_l)
    rospy.spin()
        
if __name__ == "__main__":
    main()
