
#Used for first set of util functions
import os
import numpy as np
import pickle as pk
import time
# ** Do we need this? **
###from hrl_lib.msg import NumpyArray
# ** removed dependancy for laser_camera_segmentation use **

#Used for second set of util functions 
from opencv.cv import *
from opencv.highgui import *
import numpy as np
import Image as Image


## Returns a string that can be used as a timestamp (hours and minutes) in logfiles
# @return timestamp-string
def getTime():
    return '['+time.strftime("%H:%M:%S", time.localtime())+']'

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi

##
# Converts a list of numpy matrices to one large matrix
# @param list_mat the list of little matrices
# @param axis axis to concatenate little matrices
# @return one large numpy matrix
def list_mat_to_mat(list_mat, axis=0):
    return np.concatenate(tuple(list_mat), axis=axis)


## returns current time as a string: year|month|date_hours|min|sec.
## @return current time as a string: year|month|date_hours|min|sec.
def formatted_time():
    date_name = time.strftime('%Y%h%d_%H%M%S', time.localtime())
#    curtime = time.localtime()
#    date_name = time.strftime('%Y%m%d_%I%M%S', curtime)
    return date_name

## read a pickle and return the object.
# @param filename - name of the pkl
# @return - object that had been pickled.
def load_pickle(filename):
    p = open(filename, 'r')
    picklelicious = pk.load(p)
    p.close()
    return picklelicious

## Pickle an object.
# @param object - object to be pickled
# @param filename - name of the pkl file
def save_pickle(object, filename):
    pickle_file = open(filename, 'w')
    pk.dump(object, pickle_file)
    pickle_file.close()

## Calculate L2 norm for column vectors in a matrix 
# @param mat - numpy matrix
def norm(mat):
    return np.power(np.sum(np.power(mat,2), axis=0), 0.5)


def approx_equal(a, b, epsilon=.001):
    return (b < (a+epsilon)) and ((a-epsilon) < b)


def unipolar_limit( x, upper ):
    """ limit the value of x such that
        0 <= x <= upper
    """

    if x > upper:
        x=upper
    if x < 0:
        x=0

    return x

def cart_of_pol(p):
   """ Finds cartesian coordinates of polar points [r, t]' """
   r = p[0,:]
   t = p[1,:]
   x = numpy.multiply(numpy.cos(t), r)
   y = numpy.multiply(numpy.sin(t), r)
   return numpy.vstack((x,y))

def pol_of_cart(p):
    """ Find polar coordinates of cartesian points [x, y]' """
    norm = numpy.linalg.norm(p)
    ang = math.atan2(p[1,0], p[0,0])
    return numpy.matrix([norm, ang]).T

##
# Bound the value of a number to be above lower, and lower than upper
# @return a number
def bound(value, lower, upper):
    if lower >= upper:
        t = lower
        lower = upper
        upper = t
#    print 'bound', value, 'lower', lower, 'upper', upper
    #return min(max(value, lower), upper)
    ret_val = min(max(value, lower), upper)
#    if ret_val != value:
#        print 'ut.boud bounded something.'
    return ret_val



## wraps a numpy array into hrl's datatype for sending np arrays
# over ros.
# @param np array
# @return NumpyArray object (hrl_lib/msg/NumpyArray.msg)
def wrap_np_array(nparr):
    shp = nparr.shape
    npstr = nparr.tostring()
    npdtype = str(nparr.dtype)
    nparr_ros = NumpyArray(None,npstr,shp,npdtype)
    return nparr_ros

## convert hrl's ros wrapped numpy array to a numpy array
# @param NumpyArray object (hrl_lib/msg/NumpyArray.msg)
# @return np array
def unwrap_np_array(nparr_ros):
    npstr,shp,npdtype = nparr_ros.data,nparr_ros.shape,nparr_ros.dtype
    nparr = np.fromstring(npstr,dtype=npdtype)
    nparr = nparr.reshape(shp)
    return nparr


## cartesian product of list of lists.
# code copied from: http://automatthias.wordpress.com/2007/04/28/cartesian-product-of-multiple-sets/
# @return generator. can loop over it, or list(generator) will give
# the entire list.
# NOTE - itertools in python 2.6 provides this functionality. We
# should switch over to it soon.
def cartesian_product(lists, previous_elements = []):
    if len(lists) == 1:
        for elem in lists[0]:
            yield previous_elements + [elem, ]
    else:
        for elem in lists[0]:
            for x in cartesian_product(lists[1:], previous_elements + [elem, ]):
                yield x

## choose n elements from list without replacement.
# adapted code from cartesian_product
# @return generator.
def choose_without_replacement(list, n):
    lists = [list for i in range(n)]
    return _choose_without_replacement(lists)

def _choose_without_replacement(lists,previous_elements=[],ignore_count=0):
    if len(lists) == 1:
        for elem in lists[0][ignore_count:]:
            yield previous_elements + [elem, ]
    else:
        for i,elem in enumerate(lists[0][ignore_count:]):
            for x in _choose_without_replacement(lists[1:],previous_elements + [elem, ],ignore_count+i+1):
                yield x

##
# use festival text to speech to make a soud.
# @param text - string to be said.
def say(text):
    os.system( 'echo "' + text + '" | festival --tts' )


## compute rank of a matrix.
# code copied from:
# http://mail.scipy.org/pipermail/numpy-discussion/2008-February/031218.html
def matrixrank(A,tol=1e-8):
    s = np.linalg.svd(A,compute_uv=0)
    return sum( np.where( s>tol, 1, 0 ) )

##################################################

#!usr/bin/python
#
#util_additional.py
#
#The following definitions are utility and conversion definitions that used
# to be part of the gt-ros-pkg scripts in hrl_lib/util.py
#Since they have been taken out sometime during summer 2010, they are added
# as explicit dependancies to the laser_camera_segmentation project.

cv2np_type_dict = {CV_16S      : (np.int16, 1),  
                   CV_16SC1  : (np.int16, 1),   
                   CV_16SC2  : (np.int16, 2),   
                   CV_16SC3  : (np.int16, 3),   
                   CV_16SC4  : (np.int16, 4),   
                   CV_16U      : (np.uint16, 1),   
                   CV_16UC1  : (np.uint16, 1),   
                   CV_16UC2  : (np.uint16, 2),   
                   CV_16UC3  : (np.uint16, 3),   
                   CV_16UC4  : (np.uint16, 4),   
                   CV_32F      : (np.float32, 1),   
                   CV_32FC1  : (np.float32, 1),   
                   CV_32FC2  : (np.float32, 2),   
                   CV_32FC3  : (np.float32, 3),   
                   CV_32FC4  : (np.float32, 4),   
                   CV_32S      : (np.int32, 1),   
                   CV_32SC1  : (np.int32, 1),   
                   CV_32SC2  : (np.int32, 2),   
                   CV_32SC3  : (np.int32, 3),   
                   CV_32SC4  : (np.int32, 4),   
                   CV_64F      : (np.float64, 1),   
                   CV_64FC1  : (np.float64, 1),   
                   CV_64FC2  : (np.float64, 2),   
                   CV_64FC3  : (np.float64, 3),   
                   CV_64FC4  : (np.float64, 4),   
                   CV_8S    : (np.int8, 1),   
                   CV_8SC1    : (np.int8, 1),   
                   CV_8SC2    : (np.int8, 2),   
                   CV_8SC3    : (np.int8, 3),   
                   CV_8SC4    : (np.int8, 4),   
                   CV_8U    : (np.uint8, 1),   
                   CV_8UC1    : (np.uint8, 1),   
                   CV_8UC2    : (np.uint8, 2),   
                   CV_8UC3    : (np.uint8, 3),   
                   CV_8UC4    : (np.uint8, 4)}


cv2np_type_dict_invertible = {CV_16SC1   : (np.int16, 1),   
                              CV_16SC2   : (np.int16, 2),   
                              CV_16SC3   : (np.int16, 3),   
                              CV_16SC4   : (np.int16, 4),   
                              CV_16UC1   : (np.uint16, 1),   
                              CV_16UC2   : (np.uint16, 2),   
                              CV_16UC3   : (np.uint16, 3),   
                              CV_16UC4   : (np.uint16, 4),   
                              CV_32FC1   : (np.float32, 1),   
                              CV_32FC2   : (np.float32, 2),   
                              CV_32FC3   : (np.float32, 3),   
                              CV_32FC4   : (np.float32, 4),   
                              CV_32SC1   : (np.int32, 1),   
                              CV_32SC2   : (np.int32, 2),   
                              CV_32SC3   : (np.int32, 3),   
                              CV_32SC4   : (np.int32, 4),   
                              CV_64FC1   : (np.float64, 1),   
                              CV_64FC2   : (np.float64, 2),   
                              CV_64FC3   : (np.float64, 3),   
                              CV_64FC4   : (np.float64, 4),   
                              CV_8SC1     : (np.int8, 1),   
                              CV_8SC2     : (np.int8, 2),   
                              CV_8SC3     : (np.int8, 3),   
                              CV_8SC4     : (np.int8, 4),   
                              CV_8UC1     : (np.uint8, 1),   
                              CV_8UC2     : (np.uint8, 2),   
                              CV_8UC3     : (np.uint8, 3),   
                              CV_8UC4     : (np.uint8, 4)}


#def cv2np(im):
#   numpy_type, nchannels = cv2np_type_dict[cv.cvGetElemType(im)]
#   array_size = [im.height, im.width, nchannels]
#   np_im = np.frombuffer(im.imageData, dtype=numpy_type)
#   return np.reshape(np_im, array_size)

def cv2np(im, format='RGB'):
    """This function converts an image from openCV format to a numpy array.
       This utility needs both NUMPY and OPENCV to accomplish the conversion.
       cv2np(im, format='RGB')
    """
    if format == 'BGR':
        cvCvtColor( im, im, CV_BGR2RGB )
    numpy_type, nchannels = cv2np_type_dict[cvGetElemType(im)]
    array_size = [im.height, im.width, nchannels]
    #Removed multiplication of size by (im.depth/8) as numpy takes
    #into account of that in numpy_type

    if im.__doc__ == None:
        # ctypes-opencv
        return im.as_numpy_array()
    else:
        np_im = np.array( np.frombuffer(im.imageData, dtype=numpy_type, 
                            count=im.height*im.width*nchannels))

    return np.reshape(np_im, array_size)
    
    
    
def np2pil( im ):
    """ for grayscale - all values must be between 0 and 255.
            not sure about color yet.
        np2pil(im)
    """
    #TODO: print 'util.np2cv: works for texseg.py'
    #TODO: print 'util.np2cv: more extensive tests would be useful'
    if len(im.shape) == 3:
        shp = im.shape
        channels = shp[2]
        height, width = shp[0], shp[1]
    elif len(im.shape) == 2:
        height, width = im.shape
        channels = 1
    else:
        raise AssertionError("unrecognized shape for the input image. should be 3 or 2, but was %d." % len(im.shape))

    if channels == 3:
        image = Image.fromstring( "RGB", (width, height), im.tostring() )
    if channels == 1:
        im = np.array(im, dtype=np.uint8)
        #image = Image.fromarray(im)
        image = Image.fromstring( "L", (width, height), im.tostring() )

    return image


if True:
    np2cv_type_dict = dict([(str(np.dtype(v[0])), v[1]), k] for
                           k,v in cv2np_type_dict_invertible.items())


    def np2cv(im, force_color=False):
        ''' Note: force_color -- force grayscale np image into a color cv image
            np2cv(im, force_color=False)
        '''
        image = np2pil( im )
        image.save('test.bmp', 'BMP')
        if len(im.shape) == 3:
            cvim = cvLoadImage('test.bmp')
        elif len(im.shape) == 2:
            if force_color == False:
                cvim = cvLoadImage('test.bmp', CV_LOAD_IMAGE_GRAYSCALE)
            else:
                cvim = cvLoadImage('test.bmp')
        else:
            raise AssertionError("unrecognized shape for the input image. should be 3 or 2, but was %d." % len(im.shape))

        return cvim

