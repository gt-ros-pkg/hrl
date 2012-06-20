#!/usr/bin/python

import roslib
roslib.load_manifest("hrl_face_adls")
import rospy

from hrl_ellipsoidal_control.controller_base import CartesianStepController
from pykdl_utils.pr2_kin import kin_from_param

class CartesianControllerManager(object):
    def __init__(self):
        self.kin = None

    def _command_move_cb(self, msg):
        if self.kin is None or msg.header.frame_id not in self.kin.get_segment_names():
            self.kin = kin_from_param("base_link", msg.header.frame_id)
        self.kin.forward_filled(base_segment="torso_lift_link")

