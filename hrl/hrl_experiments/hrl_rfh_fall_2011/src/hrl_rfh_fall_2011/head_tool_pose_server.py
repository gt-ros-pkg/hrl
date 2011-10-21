#! /usr/bin/python

import numpy as np
import copy

import roslib
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('hrl_generic_arms')
import rospy

from hrl_phri_2011.msg import EllipsoidParams
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3
from hrl_generic_arms.pose_converter import PoseConverter
from hrl_rfh_fall_2011.ellipsoid_space import EllipsoidSpace
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def create_arrow_marker(pose, m_id, color=ColorRGBA(1., 0., 0., 1.)):
    m = Marker()
    m.header.frame_id = "/base_link"
    m.header.stamp = rospy.Time.now()
    m.ns = "ell_pose"
    m.id = m_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.scale = Vector3(0.3, 0.3, 0.1)
    m.color = color
    m.pose = PoseConverter.to_pose_msg(pose)
    return m

head_poses = {
    #             lat   lon    height    roll   pitch   yaw
    "cheek1" : [(0,    0,     0),      (0,     0,      0)],
    "cheek2" : [(0,    0,     0),      (0,     0,      0)]
}

class HeadToolPoseServer(object):
    def __init__(self, ell_space):
        self.ell_space = ell_space

    def pub_vectors(self):
        arrows = MarkerArray()
        coords = []
        i = 0
        color = ColorRGBA(1., 0., 0., 1.)
        for lat in np.linspace(0, np.pi, 10):
            color.g += 0.1
            color.b = 0
            for lon in np.linspace(0, 2 * np.pi, 10):
                color.b += 0.1
                coords.append((lat, lon, 1, i, copy.copy(color)))
                i += 1
        arrows.markers = [create_arrow_marker(self.ell_space.ellipsoidal_to_pose(lat, lon, height), i, clr)
                          for lat, lon, height, i, clr in coords] 
        pub_arrows = rospy.Publisher("visualization_markers_array", MarkerArray)
        while not rospy.is_shutdown():
            pub_arrows.publish(arrows)
            rospy.sleep(1)

    def get_head_pose(self, name):


def main():
    rospy.init_node("head_tool_pose_server")
    ell_space = EllipsoidSpace(1)
    htps = HeadToolPoseServer(ell_space)
    htps.pub_vectors()
    

if __name__ == "__main__":
    main()
