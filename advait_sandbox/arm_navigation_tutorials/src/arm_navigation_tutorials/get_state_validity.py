#
# Copyright (c) 2010, Georgia Tech Research Corporation
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

# Author: Advait Jain (advait@cc.gatech.edu), Healthcare Robotics Lab, Georgia Tech

import roslib; roslib.load_manifest('arm_navigation_tutorials')
import sys
import rospy

from planning_environment_msgs.srv import GetStateValidity
from planning_environment_msgs.srv import GetStateValidityRequest


if __name__ == '__main__':
    rospy.init_node('get_state_validity_python')

    srv_nm = 'environment_server_right_arm/get_state_validity'
    rospy.wait_for_service(srv_nm)
    get_state_validity = rospy.ServiceProxy(srv_nm, GetStateValidity)

    req = GetStateValidityRequest()
    req.robot_state.joint_state.name = ['r_shoulder_pan_joint',
                                        'r_shoulder_lift_joint',
                                        'r_upper_arm_roll_joint',
                                        'r_elbow_flex_joint',
                                        'r_forearm_roll_joint',
                                        'r_wrist_flex_joint',
                                        'r_wrist_roll_joint']
    req.robot_state.joint_state.position = [0.] * 7
    req.robot_state.joint_state.position[0] = 0.4
    req.robot_state.joint_state.position[3] = -0.4

    req.robot_state.joint_state.header.stamp = rospy.Time.now()
    req.check_collisions = True

    res = get_state_validity.call(req)
    
    if res.error_code.val == res.error_code.SUCCESS:
        rospy.loginfo('Requested state is not in collision')
    else:
        rospy.loginfo('Requested state is in collision. Error code: %d'%(res.error_code.val))



