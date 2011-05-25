#!/usr/bin/env python
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

import roslib; roslib.load_manifest('epc_door_opening')
import rospy

from epc_door_opening.msg import MechanismKinematicsRot
from geometry_msgs.msg import Point32

import doors_forces_kinematics.arm_trajectories as at

from threading import RLock
import numpy as np
import time

##
# fit circle to the trajectory, publish the computed kinematics.
# @param cartesian_pts_list - list of 3-tuples. trajectory of the mechanism
# @param pbshr - publisher for the MechanismKinematics message
# @param lock - to make list operations thread safe. (there is a callback too.)
def circle_estimator(cartesian_pts_list, pbshr, lock):
    lock.acquire()
    n_pts = len(cartesian_pts_list)
    pts_2d = (np.matrix(cartesian_pts_list).T)[0:2,:]
    lock.release()

    if n_pts<2:
        time.sleep(0.1)
        #pbshr.publish(mk) # don't publish anything.
        return

    st = pts_2d[:,0]
    now = pts_2d[:,-1]

    mk = MechanismKinematicsRot()
    mk.cx = 0.5
    mk.cy = -3.5
    mk.cz = cartesian_pts_list[0][2]
    mk.rad = 10.

    dist_moved = np.linalg.norm(st-now)

#    if dist_moved<=0.05:
#        reject_pts_num = n_pts
#    else:
#        reject_pts_num = 1
    reject_pts_num = 1

    if dist_moved<=0.15:
        time.sleep(0.1)
        pbshr.publish(mk)
        return

    pts_2d = pts_2d[:,reject_pts_num:]

    #rad = 0.3
    rad = 1.1
    start_pos = st
    rad,cx,cy = at.fit_circle(rad, start_pos[0,0], start_pos[1,0]-rad,
                              pts_2d, method='fmin_bfgs',
                              verbose=False, rad_fix = False)
                              #verbose=False, rad_fix = True)
    #print 'rad, cx, cy:', rad, cx, cy
    #print 'n_pts:', n_pts
    mk.cx = cx
    mk.cy = cy
    mk.rad = rad
    pbshr.publish(mk)
    #rospy.logout('oye hoye hoye')

# append the point to the trajectory
def trajectory_cb(pt32, tup):
    cp_list, lock = tup
    lock.acquire()
    cp_list.append([pt32.x, pt32.y, pt32.z])
    lock.release()


if __name__ == '__main__':

    cartesian_points_list = []
    lock = RLock()
    rospy.init_node('kinematics_estimator_least_sq')
    mech_kin_pub = rospy.Publisher('mechanism_kinematics_rot',
                                    MechanismKinematicsRot)
    rospy.Subscriber('mechanism_trajectory', Point32, trajectory_cb,
                     (cartesian_points_list, lock))

    rospy.logout('Begin')
    while not rospy.is_shutdown():
        circle_estimator(cartesian_points_list, mech_kin_pub, lock)
        rospy.sleep(0.01)
    rospy.logout('End')


