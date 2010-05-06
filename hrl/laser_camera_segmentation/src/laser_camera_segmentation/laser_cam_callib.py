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

#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)
#  \author Martin Schuster (Healthcare Robotics Lab, Georgia Tech.)


import time
import hrl_lib.util as ut
import opencv as cv
import opencv.highgui as hg
import pycessing as pyc
import transforms as tr
import numpy as np,math
import tilting_hokuyo.processing_3d as p3d
#import math_util as mu

def scale(image, s):
    scaled = cv.cvCreateImage(cv.cvSize(int(image.width * s), int(image.height * s)), image.depth, image.nChannels)
    cv.cvResize(image, scaled, cv.CV_INTER_AREA)
    return scaled

class Callib():
    def __init__(self, transFunc, seeds, deltas, names, pts, img, cam_proj_mat, cam_centers, zoom, id):
        '''
        transFunc => takes in param vector, returns homogeneous Xform
        seeds => initial param vector (1XM array)
        deltas => step sizes in param vector quantities (1XM array)
        pts => 3D points to be mapped into img (3XN array)
        img => an OpenCV image from the camera
        '''

        self.dataset_id = id

        # Note that adaptors are deprecated...
        imgTmp = cv.cvCloneImage(img)
        imNP = cv.adaptors.Ipl2NumPy(imgTmp)
        self.height = imNP.shape[0] * zoom
        self.width = imNP.shape[1] * zoom  
        self.zoom = zoom

        pyc.size(self.width + 200, self.height)
        #pyc.size(self.height + 200, self.width + 200)
        self.pts = pts
        self.img = img
        self.vals = seeds
        self.selected = np.zeros( self.vals.shape[0] )
        self.selected[0] = 1.0
        self.dels = deltas
        self.names = names
        self.transFunc = transFunc
        self.cam_proj_mat = cam_proj_mat
        self.cam_centers = cam_centers
        self.display_laser = True


    def reDraw(self):
        pyc.background(255)
        pyc.lightSpecular(255*30/640, 255*30/640, 255*30/640)
        pyc.directionalLight(255,255,255,0,0,1)
        pyc.specular(102, 102, 102)

        # Project self.pts into image, and display it
        imgTmp = cv.cvCloneImage(self.img)
        imNP = cv.adaptors.Ipl2NumPy(scale(imgTmp, self.zoom))

        color_list = [(255,255,0),(255,0,0),(0,255,255),(0,255,0),(0,0,255),(0,100,100),(100,100,0),
                  (100,0,100),(100,200,100),(200,100,100),(100,100,200),(100,0,200),(0,200,100),
                  (0,100,200),(200,0,100),(100,0,100),(255,152,7) ]

        XformPts = tr.transform( self.transFunc(self.vals), self.pts )
        camPts = self.cam_proj_mat * tr.xyzToHomogenous(XformPts)
        camPts = camPts / camPts[2]
        camPts[0] = (camPts[0] + self.cam_centers[0]) * self.zoom
        camPts[1] = (camPts[1] + self.cam_centers[1]) * self.zoom
        camPts = np.matrix( np.round(camPts), 'int')
        
        conditions = np.concatenate([camPts[0] >= 0, 
                                     camPts[0] < imNP.shape[1],
                                     camPts[1] >= 0,
                                     camPts[1] < imNP.shape[0]], 0)
        r, c = np.where(np.all(conditions, 0))
        camPts_bound  = camPts[:, c.A[0]]
        x = np.asarray(self.pts[0])[0][c.A[0]]
        x = x - x.min()
        x = x / x.max() * 256 #512 #number of colors
        x = np.floor(x)
        x = np.asarray(np.matrix(x,'int'))[0]
        if self.display_laser:
            map2d = np.asarray(camPts_bound[0:2])
            n,m = map2d.shape
            for i in range(0,m):
                imNP[map2d[1,i],map2d[0,i], :] = [x[i],256-x[i],128+x[i]/2]#color_list[x[i]]
        imgTmp = cv.adaptors.NumPy2Ipl(imNP)
        #imgSmall = cv.cvCreateImage(cv.cvSize(imgTmp.width/3, imgTmp.height/3), cv.IPL_DEPTH_8U, 3)
        #cv.cvResize(imgTmp, imgSmall, cv.CV_INTER_AREA)
        im = cv.adaptors.Ipl2PIL(imgTmp)

        #pyc.rotate(math.radians(90))
        pyc.image(im, 0,0, self.width, self.height)
        #pyc.rotate(math.radians(-90))


        # Display the current values of the parameter vector (and highlight the selected one)
        pyc.textSize(10)
        for i, val in enumerate(self.vals):
            if np.nonzero(self.selected)[0] == i: 
                print 'x',
            print '%8.4f ' % self.vals[i], 
            pval = '%7s: %8.4f' % (self.names[i], self.vals[i])
            pyc.text(pval, self.width + 15, 20 + 20*i, 0)
            if np.nonzero(self.selected)[0] == i:
                pyc.fill(255,0,0)
                pyc.quad(self.width+4.0,  15.0 + 20.0*i - 7.0,
                         self.width+4.0,  15.0 + 20.0*i + 7.0,
                         self.width+13.0, 15.0 + 20.0*i,
                         self.width+13.0, 15.0 + 20.0*i)
        print '\n'
        
        self.move(pyc.escape_handler(pyc.draw()))
        
    def move(self, events):
        if len(events) > 0:
            for event in events:
                currselect = np.nonzero( self.selected )[0]
                if event.type == pyc.KEYDOWN:
                    if event.key == pyc.K_DOWN:
                        self.selected[currselect] = 0
                        self.selected[ ut.bound(currselect + 1, 0, self.selected.shape[0]-1) ] = 1
                    if event.key == pyc.K_UP:
                        self.selected[currselect] = 0
                        self.selected[ ut.bound(currselect - 1, 0, self.selected.shape[0]-1) ] = 1
                    if event.key == pyc.K_LEFT:
                        self.vals[currselect] -= self.dels[currselect]
                    if event.key == pyc.K_RIGHT:
                        self.vals[currselect] += self.dels[currselect]
                    if event.key == pyc.K_SPACE:
                        self.display_laser = not self.display_laser
                    if event.key == pyc.K_RETURN:
                        self.display3d()


        return events

    def display3d(self):
        import laser_camera_segmentation.processor as processor
        import laser_camera_segmentation.configuration as configuration       
        print 'display in 3d...'
        cfg = configuration.configuration('/home/martin/robot1_data/usr/martin/laser_camera_segmentation/labeling')
        cfg.cam_vec = np.array(self.vals)
        import scanr_transforms as trs
        cfg.camTlaser = trs.camTlaser(cfg.cam_vec)
        
        pc = processor.processor(cfg)
        pc.load_data(self.dataset_id)
        pc.process_raw_data()
        pc.display_3d('labels', False)        


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-c', action='store', type='string', dest='image',       
                 default='xxcalib.png', 
                 help='camera image')
    p.add_option('-p', action='store', type='string', dest='point_cloud', 
                 default='xxcalib.pkl', 
                 help='pickle file containing a point cloud matrix expressed in the laser\'s frame')
    p.add_option('-n', action='store', type='string', dest='camera_name', 
                 default='eleUTM', 
                 help='name of camera as specified in camera_config.py')

    opt, args = p.parse_args()
    image_filename       = opt.image
    point_cloud_filename = opt.point_cloud


    def cameraTlaser(vec):
        x, y, z, r1, r2, r3, r4 = vec
        disp = np.matrix([x, y, z]).T
        rot1 = tr.Rx(math.radians(r1))
        rot2 = tr.Rz(math.radians(r2))
        rot3 = tr.Rx(math.radians(r3))
        rot4 = tr.Rz(math.radians(r4))
        rt   = rot4 * rot3 * rot2 * rot1 
        laserTcam = tr.composeHomogeneousTransform(rt, disp)
        trans = tr.invertHomogeneousTransform(laserTcam)
        return trans

    #seeds = np.array([0.018, -0.057, 0.015, 91.2, 90.8, 0.0])
    #seeds = np.array([0.013, -0.027, 0.025, 91.2, 92.8, 0.0])
    #seeds = np.array([0.013, -0.032, 0.01, 91.4, 93.2, 0.0])
    #seeds = np.array([ 0.003,  0.1  , -0.02, 89.8, 89.8, 90.0, 0])
    seeds = np.array([ -0.087,  0.105  , 0.01, 89.8, 89.8, 90.0, 0])
    #deltas = np.array([0.005, 0.005, 0.005, 0.1, 0.1, 0.1, 0.1])
    #seeds = np.array([0.061, 0.032, -0.035, 0.8, 0.9, -1.7, 3.1 ])
    deltas = np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 0.1])    
    #self.cam_names = ['Ry_0', 'Rz_0', 'Rx_-90', 'Rz_-90', 'dx', 'dy', 'dz']
    names = ['x_disp', 'y_disp', 'z_disp', 'rotX', 'rotZ', 'rotX', 'rotZ']
    
    img = hg.cvLoadImage(image_filename)
    raw_laser_scans = ut.load_pickle(point_cloud_filename)
    #if raw_laser_scans.__class__ == ().__class__:
    poses, scans = raw_laser_scans['laserscans'][0]
    points_cloud_laser = p3d.generate_pointcloud(poses, scans, math.radians(-180), math.radians(180), 
                            0, .035, max_dist=5, min_dist=.2)
    #else:
    #    points_cloud_laser = raw_laser_scans


    import webcam_config as cc
    cp = cc.webcam_parameters['DesktopWebcam']
    fx = cp['focal_length_x_in_pixels']
    fy = cp['focal_length_y_in_pixels']
    cam_proj_mat =  np.matrix([[fx, 0, 0,   0],
                               [0, fy, 0,   0],
                               [0,  0, 1,   0]])

    cam_centers = ( cp['optical_center_x_in_pixels'], cp['optical_center_y_in_pixels'] )

    c = Callib(cameraTlaser, seeds, deltas, names, points_cloud_laser, img, cam_proj_mat, cam_centers, 1/1.0)
    
    while True:
        c.reDraw()
