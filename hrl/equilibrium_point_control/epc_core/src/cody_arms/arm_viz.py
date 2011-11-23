#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Author: Advait Jain


import math, numpy as np

import arm_client as ac
import arms as ar
import roslib; roslib.load_manifest('epc_core')
import rospy

import tf.broadcaster as tfb
import hrl_lib.transforms as tr

from hrl_msgs.msg import FloatArray
from roslib.msg import Header
from visualization_msgs.msg import Marker

def publish_cartesian_markers(arm, time_stamp, cep, rot, c1, c2, marker_id):
    marker = Marker()
    marker.header.frame_id = ar.link_tf_name(arm, 0)
    marker.header.stamp = time_stamp
    marker.ns = arm
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = cep[0,0]
    marker.pose.position.y = cep[1,0]
    marker.pose.position.z = cep[2,0]
    marker.scale.x = 0.1
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.lifetime = rospy.Duration()

    marker.id = marker_id*100 + 0
    #rot1 = tr.Ry(math.radians(90.)) * rot.T
    rot1 = rot * tr.rotY(math.pi/2)
    quat = tr.matrix_to_quaternion(rot1)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.color.r = c1[0]
    marker.color.g = c1[1]
    marker.color.b = c1[2]
    marker.color.a = 1.
    marker_pub.publish(marker)

    marker.id = marker_id*100 + 1
    if arm == 'left_arm':
        #rot2 = tr.Rz(math.radians(90.)) * rot.T
        rot2 = rot * tr.rotZ(-math.pi/2)
    else:
        #rot2 = tr.Rz(math.radians(-90.)) * rot.T
        rot2 = rot * tr.rotZ(math.pi/2)

    quat = tr.matrix_to_quaternion(rot2)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.color.r = c2[0]
    marker.color.g = c2[1]
    marker.color.b = c2[2]
    marker.color.a = 1.
    marker_pub.publish(marker)


def publish_sphere_marker(color, size, frameid, time_stamp, ns,
                          marker_id):
    marker = Marker()
    marker.header.frame_id = frameid
    marker.header.stamp = time_stamp
    marker.ns = ns
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    marker.lifetime = rospy.Duration()

    marker.id = marker_id
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.
    marker_pub.publish(marker)


if __name__ == '__main__':
    arms = ar.M3HrlRobot()
    arm_client = ac.MekaArmClient(arms)

    force_r_pub = rospy.Publisher('/r_arm/force_base', FloatArray)
    force_l_pub = rospy.Publisher('/l_arm/force_base', FloatArray)
    marker_pub = rospy.Publisher('/cody_arms/viz_marker', Marker)

    rospy.logout('Sleeping ...')
    rospy.sleep(1.0)
    rospy.logout('... begin')

    r_arm = 'right_arm'
    l_arm = 'left_arm'

    transform_bcast = tfb.TransformBroadcaster()
    torso_link_name = ar.link_tf_name(r_arm, 0)
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        f_r = arm_client.get_wrist_force(r_arm, base_frame=True)
        f_l = arm_client.get_wrist_force(l_arm, base_frame=True)
        time_stamp = rospy.Time.now()
        h = Header()
        h.stamp = time_stamp
        force_r_pub.publish(FloatArray(h, f_r))
        force_l_pub.publish(FloatArray(h, f_l))

        publish_sphere_marker((0.5,0.5,0.5), 0.08, torso_link_name,
                              time_stamp, 'both_arms', 0)

        for arm in [r_arm, l_arm]:
            q = arm_client.get_joint_angles(arm)
            links = [2, 3, 7]
            for i in links:
                p, rot = arms.FK_all(arm, q, i)
                qaut = tr.matrix_to_quaternion(rot)
                frameid = ar.link_tf_name(arm, i)
                transform_bcast.sendTransform(p.A1.tolist(), qaut, time_stamp,
                                              frameid, torso_link_name)
                publish_sphere_marker((0.5,0.1,0.5), 0.05, frameid,
                                      time_stamp, arm, i)

            c1 = (0.5, 0.1, 0.5)
            c2 = (0.5, 0.5, 0.1)
            p, rot = arms.FK_all(arm, q)
            publish_cartesian_markers(arm, time_stamp, p, rot, c1, c2,
                                      marker_id=76)

            c1 = (0.2, 0.2, 0.2)
            c2 = (0.6, 0.6, 0.6)
            jep = arm_client.get_jep(arm)
            jep = arms.clamp_to_physical_joint_limits(arm, jep)
            cep, rot = arms.FK_all(arm, jep)
            publish_cartesian_markers(arm, time_stamp, cep, rot, c1, c2,
                                marker_id = 77)




