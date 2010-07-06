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
#from opencv import cv
#from opencv import highgui
import numpy as np
import pickle as pk

def list_mat_to_mat(list_mat, axis=0):
	return np.concatenate(tuple(list_mat), axis=axis)

def load_pickle(filename):
    p = open(filename, 'r')
    picklelicious = pk.load(p)
    p.close()
    return picklelicious

def dump_pickle(object, filename):
    pickle_file = open(filename, 'w')
    pk.dump(object, pickle_file)
    pickle_file.close()




















#import Image as Image


#cv2np_type_dict = {cv.CV_16S	  : (np.int16, 1),	
#				   cv.CV_16SC	 : (np.int16, 1),   
#				   cv.CV_16SC1	: (np.int16, 1),   
#				   cv.CV_16SC2	: (np.int16, 2),   
#				   cv.CV_16SC3	: (np.int16, 3),   
#				   cv.CV_16SC4	: (np.int16, 4),   
#				   cv.CV_16U	  : (np.uint16, 1),   
#				   cv.CV_16UC	 : (np.uint16, 1),   
#				   cv.CV_16UC1	: (np.uint16, 1),   
#				   cv.CV_16UC2	: (np.uint16, 2),   
#				   cv.CV_16UC3	: (np.uint16, 3),   
#				   cv.CV_16UC4	: (np.uint16, 4),   
#				   cv.CV_32F	  : (np.float32, 1),   
#				   cv.CV_32FC	 : (np.float32, 1),   
#				   cv.CV_32FC1	: (np.float32, 1),   
#				   cv.CV_32FC2	: (np.float32, 2),   
#				   cv.CV_32FC3	: (np.float32, 3),   
#				   cv.CV_32FC4	: (np.float32, 4),   
#				   cv.CV_32S	  : (np.int32, 1),   
#				   cv.CV_32SC	 : (np.int32, 1),   
#				   cv.CV_32SC1	: (np.int32, 1),   
#				   cv.CV_32SC2	: (np.int32, 2),   
#				   cv.CV_32SC3	: (np.int32, 3),   
#				   cv.CV_32SC4	: (np.int32, 4),   
#				   cv.CV_64F	  : (np.float64, 1),   
#				   cv.CV_64FC	 : (np.float64, 1),   
#				   cv.CV_64FC1	: (np.float64, 1),   
#				   cv.CV_64FC2	: (np.float64, 2),   
#				   cv.CV_64FC3	: (np.float64, 3),   
#				   cv.CV_64FC4	: (np.float64, 4),   
#				   cv.CV_8S	   : (np.int8, 1),   
#				   cv.CV_8SC	  : (np.int8, 1),   
#				   cv.CV_8SC1	 : (np.int8, 1),   
#				   cv.CV_8SC2	 : (np.int8, 2),   
#				   cv.CV_8SC3	 : (np.int8, 3),   
#				   cv.CV_8SC4	 : (np.int8, 4),   
#				   cv.CV_8U	   : (np.uint8, 1),   
#				   cv.CV_8UC	  : (np.uint8, 1),   
#				   cv.CV_8UC1	 : (np.uint8, 1),   
#				   cv.CV_8UC2	 : (np.uint8, 2),   
#				   cv.CV_8UC3	 : (np.uint8, 3),   
#				   cv.CV_8UC4	 : (np.uint8, 4)}

#def numpymat2cvmat(nmat):
#    raise RuntimeError("numpymat2cvmat: use something else")
#    #cvmat = cv.cvCreateMat(nmat.shape[0],nmat.shape[1],cv.CV_32FC1)
#    #for i in range(nmat.shape[0]):
#    #    for j in range(nmat.shape[1]):
#    #        #print cvmat[i][j]
#    #        #print nmat[i,j]	  
#    #        cvmat[i,j] = nmat[i,j]	  
#    #return cvmat
#
#def cvmat2numpymat(cvmat):
#    raise RuntimeError("cvmat2numpymat: use something else")
#	#nmat = np.zeros((cvmat.width,cvmat.height))
#	#for i in range(cvmat.width):
#	#	for j in range(cvmat.height):
#	#		nmat[i][j] = cvmat[i][j]
#	#return nmat
#
#def cv2np(im, format='RGB'):
#    raise RuntimeError("cv2np: use something else")
#	#if format == 'BGR':
#	#	cv.cvCvtColor( im, im, cv.CV_BGR2RGB )
#	#numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(im)]
#	#array_size = [im.height, im.width, nchannels]
#	#np_im = np.frombuffer(im.imageData, dtype=numpy_type, 
#    #        count=im.height*im.width*nchannels*(im.depth/8))
#	#return np.reshape(np_im, array_size)
#def np2cv(im):
#    raise RuntimeError("np2cv: use something else")
#    #image = np2pil( im )
#    #image.save('test.bmp', 'BMP')
#    #cvim = highgui.cvLoadImage('test.bmp')
#    #return cvim

#def np2pil( im ):
#    """ for grayscale - all values must be between 0 and 255.
#        not sure about color yet.
#    """
#    raise RuntimeError("np2pil: moved to hrl_lib.util")
#    ##TODO: print 'util.np2cv: works for texseg.py'
#    ##TODO: print 'util.np2cv: more extensive tests would be useful'
#    #if len(im.shape) == 3:
#    #    shp = im.shape
#    #    channels = shp[2]
#    #    height, width = shp[0], shp[1]
#    #elif len(im.shape) == 2:
#    #    height, width = im.shape
#    #    channels = 1
#    #else:
#    #    raise AssertionError("unrecognized shape for the input image. should be 3 or 2, but was %d." % len(im.shape))
#    #
#    #if channels == 3:
#    #    image = Image.fromstring( "RGB", (width, height), im.tostring() )
#    #if channels == 1:
#    #    im = np.array(im, dtype=np.uint8)
#    #    image = Image.fromarray(im)
#    #    #image = Image.fromstring( "L", (width, height), im.tostring() )
#    #
#    #return image
