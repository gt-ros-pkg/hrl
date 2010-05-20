# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
import laser_detector as ld
#import opencv as cv
#import opencv.highgui as hg
import util as ut

dataset = ld.load_pickle('PatchClassifier.dataset.pickle')

























































#PKG = 'laser_interface'
#import sys, os, subprocess
#try:
#    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
#    sys.path.append(os.path.join(rostoolsDir,'src'))
#    import rostools.launcher
#    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
#except ImportError:
#    print >> sys.stderr, "\nERROR: Cannot locate rostools"
#    sys.exit(1)  
#
##from pkg import *
#import rospy
#from std_msgs.msg import Position
#from std_msgs.msg import RobotBase2DOdom
#import sys
#
#def debug_me(p):
#    print 'received', p.pos.x, p.pos.y, p.pos.th
#rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
#rospy.ready('test')
#rospy.spin()
#def swap_br(npimage):
#    b = npimage[:,:,0].copy()
#    r = npimage[:,:,2].copy()
#    npimage[:,:,0] = r
#    npimage[:,:,2] = b
#    return npimage
#
#image   = hg.cvLoadImage('1frame489.png')
#npimg   = ut.cv2np(image)
#swapped = swap_br(npimg.copy())
#cvimg   = ut.np2cv(swapped)
#
#hg.cvNamedWindow('hai', 1)
#hg.cvShowImage('hai', cvimg)
#hg.cvWaitKey(10)
#
#





















