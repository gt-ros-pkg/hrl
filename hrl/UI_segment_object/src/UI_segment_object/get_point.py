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

#  \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('UI_segment_object')

import rospy
from UI_segment_object.srv import GetPt
from UI_segment_object.srv import None_Bool

if __name__ == '__main__':
    print 'hello world'
    rospy.init_node('point_and_click_client')

    reset_srv_name = 'UI_reset'
    srv_name = 'get_3D_pt'

    rospy.loginfo('waiting for service: %s'%reset_srv_name)
    rospy.wait_for_service(reset_srv_name)
    rospy.loginfo('waiting for service: %s'%srv_name)
    rospy.wait_for_service(srv_name)
    rospy.loginfo('Done')

    reset_ui = rospy.ServiceProxy(reset_srv_name, None_Bool)
    get_3d_point = rospy.ServiceProxy(srv_name, GetPt)

    print 'Reset result:', reset_ui()
    # this will display an image from the Kinect. left click to
    # select point, right click to close the image and get the 3D
    # coordinate.
    resp = get_3d_point()
    print 'resp:', resp.pt.x, resp.pt.y, resp.pt.z



