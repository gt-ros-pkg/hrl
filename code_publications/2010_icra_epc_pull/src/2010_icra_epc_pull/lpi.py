#
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

# \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)

import sys,os
sys.path.append(os.environ['HRLBASEPATH']+'/usr/advait/LPI')
import cam_utm_lpi as cul

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import mekabot.coord_frames as mcf
import math, numpy as np
import util as uto

import tilting_hokuyo.processing_3d as p3d
import camera_config as cc

## returns selected location in global coord frame.
# @param angle - angle at which to take image, and about which to take
# a 3D scan.
def select_location(c,thok,angle):
    thok.servo.move_angle(angle)
    cvim = c.get_frame()
    cvim = c.get_frame()
    cvim = c.get_frame()
    im_angle = thok.servo.read_angle()

    tilt_angles = (math.radians(-20)+angle,math.radians(30)+angle)
    pos_list,scan_list = thok.scan(tilt_angles,speed=math.radians(10))
    m = p3d.generate_pointcloud(pos_list,scan_list,math.radians(-60), math.radians(60),
                                0.0,-0.055)
    pts = mcf.utmcam0Tglobal(mcf.globalTthok0(m),im_angle)

    cam_params = cc.camera_parameters['mekabotUTM']
    fx = cam_params['focal_length_x_in_pixels']
    fy = cam_params['focal_length_y_in_pixels']
    cx,cy = cam_params['optical_center_x_in_pixels'],cam_params['optical_center_y_in_pixels']
    cam_proj_mat =  np.matrix([[fx, 0, cx],
                               [0, fy, cy],
                               [0,  0,  1]])

    cvim,pts2d = cul.project_points_in_image(cvim,pts,cam_proj_mat)
    cp = cul.get_click_location(cvim)
    print 'Clicked location:', cp
    if cp == None:
        return None
    idx = cul.get_index(pts2d.T,cp)
    pt3d = pts[:,idx]
    pt_interest = mcf.globalTutmcam0(pt3d,im_angle)

    hl_thok0 = mcf.thok0Tglobal(pt_interest)
    l1,l2 = 0.0,-0.055
    d = {}
    d['pt'] = hl_thok0
    d['pos_list'],d['scan_list'] = pos_list,scan_list
    d['l1'],d['l2'] = l1,l2
    d['img'] = uto.cv2np(cvim)
    ut.save_pickle(d,'hook_plane_scan_'+ut.formatted_time()+'.pkl')

    return pt_interest


if __name__ == '__main__':
    import camera
    import hokuyo.hokuyo_scan as hs
    import tilting_hokuyo.tilt_hokuyo_servo as ths

    hok = hs.Hokuyo('utm',0,flip=True,ros_init_node=True)
    thok = ths.tilt_hokuyo('/dev/robot/servo0',5,hok,l1=0.,l2=-0.055)
    cam = camera.Camera('mekabotUTM')
    for i in range(10):
        cam.get_frame()

    pt = select_location(cam,thok)
    print 'Selected location in global coordinates:', pt.A1.tolist()


