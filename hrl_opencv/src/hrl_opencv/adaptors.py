##
# Taken from http://opencv.willowgarage.com/wiki/PythonInterface
#
import roslib
roslib.load_manifest('hrl_opencv')
import cv
import Image #PIL
import numpy as np

def pil2cv(pi):
    cv_im = cv.CreateImageHeader(pi.size, cv.IPL_DEPTH_8U, 1)
    cv.SetData(cv_im, pi.tostring())
    return cv_im

def cv2pil(cv_im):
    return Image.fromstring(cv.GetSize(cv_im), "L", cv_im.tostring())

def cv2array(im):
  depth2dtype = {
        cv.IPL_DEPTH_8U: 'uint8',
        cv.IPL_DEPTH_8S: 'int8',
        cv.IPL_DEPTH_16U: 'uint16',
        cv.IPL_DEPTH_16S: 'int16',
        cv.IPL_DEPTH_32S: 'int32',
        cv.IPL_DEPTH_32F: 'float32',
        cv.IPL_DEPTH_64F: 'float64',
    }
  
  arrdtype=im.depth
  a = np.fromstring(
         im.tostring(),
         dtype=depth2dtype[im.depth],
         count=im.width*im.height*im.nChannels)
  a.shape = (im.height,im.width,im.nChannels)
  return a
    
def array2cv(a):
  dtype2depth = {
        'uint8':   cv.IPL_DEPTH_8U,
        'int8':    cv.IPL_DEPTH_8S,
        'uint16':  cv.IPL_DEPTH_16U,
        'int16':   cv.IPL_DEPTH_16S,
        'int32':   cv.IPL_DEPTH_32S,
        'float32': cv.IPL_DEPTH_32F,
        'float64': cv.IPL_DEPTH_64F,
    }
  try:
    nChannels = a.shape[2]
  except:
    nChannels = 1
  cv_im = cv.CreateImageHeader((a.shape[1],a.shape[0]), 
          dtype2depth[str(a.dtype)],
          nChannels)
  cv.SetData(cv_im, a.tostring(), 
             a.dtype.itemsize*nChannels*a.shape[1])
  return cv_im

def array2cvmat(a):
    dtype2type = {
          'uint8':   cv.CV_8UC1,
          'int8':    cv.CV_8SC1, 
          'uint16':  cv.CV_16UC1, 
          'int16':   cv.CV_16SC1, 
          'int32':   cv.CV_32SC1, 
          'float32': cv.CV_32FC1, 
          'float64': cv.CV_64FC1 
      }
    #create matrix headers
    rows = a.shape[0]
    cols = a.shape[1]
    type = dtype2type[str(a.dtype)]
    cvmat = cv.CreateMatHeader(rows, cols, type)

    #set data
    cv.SetData(cvmat, a.tostring(), a.dtype.itemsize * a.shape[1])
    return cvmat

