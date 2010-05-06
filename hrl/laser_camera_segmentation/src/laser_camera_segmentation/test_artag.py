#!/usr/bin/python
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


'''
Created on Oct 12, 2009

@author: martin
'''
import opencv.cv as cv
import opencv.highgui as hg
import webcamera
import util as ut    



if __name__ == '__main__':
    
    wc = webcamera.Webcamera()
    img = wc.get_frame()
    grey = cv.cvCreateImage((img.width, img.height), 8,1)
    hg.cvConvertImage(img, grey)
    
#    img = hg.cvLoadImage('test.jpg')
#    grey = cv.cvCreateImage((img.width, img.height), 8,1)
#    hg.cvConvertImage(img, grey)
    
    file = open('test.raw', 'wb')
    

    
    for i in range(grey.height):
        for j in range(grey.width):
            file.write(chr(grey[i][j]))
            
    file.close()
    
    #ut.display_images([img])

    import subprocess

    output = subprocess.Popen(["/home/martin/artags/ARToolKitPlus_2.1.1/bin/simple", ""], stdout=subprocess.PIPE, env={"LD_LIBRARY_PATH": "/home/martin/artags/ARToolKitPlus_2.1.1/lib:/usr/local/lib:"}).communicate()[0]
    import re
    try:
        m = re.search('getARMatrix.*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*).*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*).*\n([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)  ([0-9\-\.]*)', output)
    except: 
        #sys.exc_info()[0]
        print "ERROR parsing ARToolKitPlus output"
        
    import numpy as np
    R = np.array(m.groups(),dtype=np.float)
    R = R.reshape(3,4)
    
    if False == np.any(R): #all elements are 0
        print "ERROR: failed to detect AR tag"
    
    print output
    print m.groups()
    print R
    
        
    from enthought.mayavi import mlab
    #displaz bounded 3d-points
    #mlab.points3d(self.pts3d_bound[0,:].A1,self.pts3d_bound[1,:].A1,self.pts3d_bound[2,:].A1,self.pts3d_bound[0,:].A1,mode='point',scale_factor=0.01,colormap='jet')#,colormap='winter'
   
    #mlab.triangular_mesh([[0,-1,0]], [[0,1,0]], [[10,1,0]], [[10,-1,0]])
    #mlab.mesh([0,0,10,10],[-1,1,-1,1],[0,0,0,0])
    
    import numpy as np
#    v1 = np.array([0,0,0])
#    v2 = np.array([100,0,0])
#    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)])
#    v1 = np.array([0,0,0])
#    v2 = np.array([0,100,0])
#    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)])
#    v1 = np.array([0,0,0])
#    v2 = np.array([0,0,100])
#    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)])



    v1 = np.array([0,0,0])
    v2 = np.array([80,0,0])
    v2 = np.dot(R[:,0:3],v2)
    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)],color=(1,1,1))

    v1 = np.array([0,0,0])
    v2 = np.array([0,80,0])
    v2 = np.dot(R[:,0:3],v2)
    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)],color=(1,1,1))

    v1 = np.array([0,0,0])
    v2 = np.array([0,0,80])
    v2 = np.dot(R[:,0:3],v2)
    mlab.triangular_mesh([[v1[0]-1,v1[0]+1,v2[0]-1,v2[0]+1]], [[v1[1]-1,v1[1]+1,v2[1]-1,v2[1]+1]], [[v1[2]-1,v1[2]+1,v2[2]-1,v2[2]+1]], [(0,1,2),(1,2,3)],color=(1,1,1))


    print 
    #axis
    mlab.triangular_mesh([[0,0,100]], [[-0.3,0.3,0]], [[0,0,0]], [(0,1,2)])
    mlab.triangular_mesh([[-0.3,0.3,0]], [[0,0,100]], [[0,0,0]], [(0,1,2)])
    mlab.triangular_mesh([[-0.3,0.3,0]], [[0,0,0]], [[0,0,100]], [(0,1,2)])
    mlab.show()    


    print 'done.'