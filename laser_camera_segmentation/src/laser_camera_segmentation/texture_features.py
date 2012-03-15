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

#  \author Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)


import hrl_lib.util as ut    
import numpy as np
import opencv as cv
import opencv.highgui as hg
    
##
# calculates eigen values of covariance matrix accumulating statistics of sobel filter responses in an image block
#
# @param cv_image opencv image to calculate texture over
# @param blocksize size of block to accumulate statistics (in pixels)
# @param filtersize size of sobel filter to use (in pixels)
# @return numpy matrix of size (width, height, 2) where [:,:,0] is the set of first eigen values and [:,:,1] is the second set
def eigen_texture(cv_image, blocksize=8, filtersize=3):
    gray_image = cv.cvCreateImage(cv.cvSize(cv_image.width, cv_image.height), cv.IPL_DEPTH_8U, 1)
    eig_tex = cv.cvCreateImage(cv.cvSize(cv_image.width*6, cv_image.height), cv.IPL_DEPTH_32F, 1)
    
    cv.cvCvtColor(cv_image, gray_image, cv.CV_BGR2GRAY)    
    cv.cvCornerEigenValsAndVecs(gray_image, eig_tex, blocksize, filtersize)
    eig_tex_np = ut.cv2np(eig_tex)
            
    eig_tex_np = np.reshape(eig_tex_np, [cv_image.height, cv_image.width, 6])            
    return eig_tex_np[:,:,0:2]
    
def visualize(eigens):
    l1 = eigens[:,:,0]
    l2 = eigens[:,:,1]    
    m1 = np.min(l1)
    m2 = np.min(l2)
    r1 = np.max(l1) - m1 
    r2 = np.max(l2) - m2
    if r1 == 0:
        r1 = 1
    if r2 == 0:
        r2 = 1      
    l1cv = ut.np2cv(np.array( (1 - ((l1-m1) / r1)) * 255, dtype='uint8'))
    l2cv = ut.np2cv(np.array( (1 - ((l2-m2) / r2)) * 255, dtype='uint8'))
    hg.cvNamedWindow('eigen value 1', 1)
    hg.cvNamedWindow('eigen value 2', 1)
    hg.cvShowImage('eigen value 1', l1cv)
    hg.cvShowImage('eigen value 2', l2cv)
    while True:
        k = hg.cvWaitKey(33)
        if k == ' ':
            return
        if k == 'x':
            exit()
    
        
if __name__ == '__main__':
    #import pdb
    #hg.cvNamedWindow('win', 1)
    im = hg.cvLoadImage('/home/haidai/svn/robot1/src/projects/08_03_dog_commands/dragonfly_color_calibration/untitled folder/camera_image.png')
    #hg.cvShowImage('win', im)
    for i in range(40):
        s = (i+1) * 2
        print s
        eig_tex_np = eigen_texture(im, blocksize=s, filtersize=3)
        visualize(eig_tex_np)


    
#    pdb.set_trace()        
#    def texture_features(self, block_size=5, filter_size=3):
#        """
#        Calculates the texture features associated with the image.
#        block_size gives the size of the texture neighborhood to be processed
#        filter_size gives the size of the Sobel operator used to find gradient information
#        """
#        #block_size = cv.cvSize(block_size, block_size)
#
#        #convert to grayscale float
#        channels = 1
#        self.gray_image = cv.cvCreateImage(cv.cvSize(self.im_width, self.im_height),
#                                           cv.IPL_DEPTH_8U, #cv.IPL_DEPTH_16U, #cv.IPL_DEPTH_32F,
#                                           channels)
#
#
#        #cv.CV_32FC1, #cv.IPL_DEPTH_32F, #cv.IPL_DEPTH_8U, #cv.IPL_DEPTH_16U, 
#        channels = 1
#        eig_tex = cv.cvCreateImage(cv.cvSize(self.im_width*6, self.im_height),
#                                    cv.IPL_DEPTH_32F, 
#                                    channels)
#
#
#        cv.cvCvtColor(self.image, self.gray_image, cv.CV_BGR2GRAY);
#
#        #cv.cvAdd(const CvArr* src1, const CvArr* src2, CvArr* dst, const CvArr* mask=NULL );
#        
#        #highgui.cvConvertImage(self.image, self.gray_image)
#        
#        cv.cvCornerEigenValsAndVecs(self.gray_image, eig_tex,#CvArr* eigenvv,
#                                    block_size, filter_size)
#
#        eig_tex = ut.cv2np(eig_tex)
#        eig_tex = np.reshape(eig_tex, [self.im_height, self.im_width, 6])
#        #print eig_tex.shape ## [480,640,3]
#        ## (l1, l2, x1, y1, x2, y2), where
#        ## l1, l2 - eigenvalues of M; not sorted
#        ## (x1, y1) - eigenvector corresponding to l1
#        ## (x2, y2) - eigenvector corresponding to l2
#        tex_feat = np.zeros([3, self.im_height * self.im_width], dtype=np.float32)
#        tmp = np.reshape(eig_tex, [self.im_height * self.im_width, 6]).T
#        s = tmp[0] > tmp[1]
#        tex_feat[1:3, s] = tmp[0, s] * tmp[2:4, s]
#        tex_feat[0, s] = tmp[1, s]
#        tex_feat[1:3, -s] = tmp[1, -s] * tmp[4:6, -s]
#        tex_feat[0, -s] = tmp[0, -s]
#        
#        self.tex_feat = tex_feat.T
#        self.tex_image = np.reshape(self.tex_feat, [self.im_height, self.im_width, 3])

















