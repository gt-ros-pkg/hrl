#!/usr/bin/env python
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

#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)



##import roslib; roslib.load_manifest('laser_camera_segmentation')
##import rospy

##from std_msgs.msg import String


import hrl_tilting_hokuyo.processing_3d as p3d
from enthought.mayavi import mlab


import hrl_lib.util as ut
import numpy as np, math #import numpy.core as np??
import scipy
from scipy import stats

##import scanr as scanr
##import nearesNeighbourGather.NearestLaserPoint as NearestLaserPoint

if __name__ == '__main__':
		
##	print 'test'
##	scanr = scanr.scanr()
##	scanr.verify_laser_cam_callib()


	dict = ut.load_pickle('../../data/2009Aug31_172113_dict.pkl')
	pts = p3d.generate_pointcloud(dict['pos_list'],dict['scan_list'], math.radians(-60),math.radians(60),dict['l1'],dict['l2'], min_tilt=math.radians(-20),max_tilt=math.radians(20))
	
	hist = scipy.stats.histogram(pts[2],30)
	hist_max_index = hist[0].argmax()
	z_min = hist[1] + hist_max_index * hist[2]
	z_max = z_min + hist[2]
	scalar_list = list()
	for x,y,z in np.asarray(pts.T):
		#scalar_list.append(x)
		if z_min < z < z_max:
			scalar_list.append(29)
		else:
			scalar_list.append(x)
		
	mlab.points3d(pts[0,:].A1,pts[1,:].A1,pts[2,:].A1,scalar_list,mode='point',scale_factor=0.01)#,colormap='winter'
	mlab.colorbar()
	
	#scipy.stats.histogram(pts[2],30)[0].argmax()

	##mlab.triangular_mesh([[0,0,0]], [[0,1,0]], [[0,1,1]], [(0,1,2)])
	mlab.show()