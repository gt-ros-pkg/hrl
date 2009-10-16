import roslib
roslib.load_manifest('hrl_opencv')
#from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension

import opencv as cv
import opencv.highgui as hg
import opencv.adaptors as ad
import Image as pil
import numpy as np

#################################################################################################################
# ROS Utility Functions
#################################################################################################################

##
# Convert from ROS to OpenCV image datatype
# @param image to convert
def ros2cv(image):
    #ros to pil then pil to opencv
    #if image.encoding != 'bgr' and image.encoding != 'rgb' and image.encoding != 'bgr8':
    channels = 1
    if image.encoding != 'bgr8':
        raise RuntimeError('Unsupported format "%s"' % image.encoding)
    else:
        channels = 3
    if image.is_bigendian != 0:
        raise RuntimeError('Unsupported endianess')

    #print image.encoding
    #print image.step
    #print image.is_bigendian
    #if image.depth != 'uint8':
    #    raise RuntimeError('Unsupported depth "%s"' % image.depth)

    #height    = image.data.layout.dim[0].size
    #width     = image.data.layout.dim[1].size
    #channels  = image.data.layout.dim[2].size
    #print 'expected', image.width * channels
    #print 'step', image.step
    #print 'image.width', image.width, image.height
    assert(image.width * channels == image.step)
    height = image.height
    width  = image.width
    np_image = np.reshape(np.fromstring(image.data, dtype='uint8', count=height*width*channels), [height, width, 3])
    #np
    #np_image[:,:,2].A1
    #np_image[:,:,1].A1
    #np_image[:,:,0].A1
    np_image2 = np.empty(np_image.shape, dtype='uint8')
    np_image2[:, :, 0] = np_image[:, :, 2]
    np_image2[:, :, 1] = np_image[:, :, 1]
    np_image2[:, :, 2] = np_image[:, :, 0]

    #concatenated = np.concatenate([np_image[:,:,2].flatten(), np_image[:,:,1].flatten(), np_image[:,:,0].flatten()], axis=0)
    #print 2050*2448*3
    #print concatenated.shape
    #np_image2 = np.reshape(np.concatenate([np_image[:,:,0].flatten(), np_image[:,:,1].flatten(), np_image[:,:,2].flatten()], axis=0), [3, width, height])
    #np_image2 = np.swapaxes(np_image2, 0, 2)
    #print np_image[:,:,2].shape
    #print 'datatype:', np_image2.dtype
    #print np_image2.shape
    return ad.NumPy2Ipl(np_image2)
    #return None

#################################################################################################################
# Highgui 
#################################################################################################################

##
# Simple drawing of text on an image with a drop shadow
def text(image, x, y, a_string):
    font = cv.cvInitFont(CV_FONT_HERSHEY_SIMPLEX, .3, .3)
    cv.cvPutText(image, a_string, cv.cvPoint(x, y),     font, cv.cvScalar(0,0,0))
    cv.cvPutText(image, a_string, cv.cvPoint(x+1, y+1), font, cv.cvScalar(255,255,255))

def clone_image(image):
    return cv.cvCloneImage(image)

def save_image(name, image):
    hg.cvSaveImage(name, image)

def show_image(name, image):
    hg.cvShowImage(name, image)

def named_window(name):
    hg.cvNamedWindow(name, hg.CV_WINDOW_AUTOSIZE)

def wait_key(msecs=33):
    return hg.cvWaitKey(msecs)

#################################################################################################################
# CXCore 
#################################################################################################################

#################################################################################################################
# CVAux 
#################################################################################################################

#################################################################################################################
# Machine Learning 
#################################################################################################################

#################################################################################################################
# CVReference 
#################################################################################################################
##
# Morphological closing
def morpho_close(cv_image, n_times=1):
    dst  = cv.cvCloneImage(cv_image)
    dst2 = cv.cvCloneImage(cv_image)
    cv.cvDilate(cv_image, dst, None, n_times)
    cv.cvErode(dst, dst2, None, n_times)
    return dst2

##
# Morphological opening
def morpho_open(cv_image, n_times=1):
    dst  = cv.cvCloneImage(cv_image)
    dst2 = cv.cvCloneImage(cv_image)
    cv.cvErode(cv_image, dst, None, n_times)
    cv.cvDilate(dst, dst2, None, n_times)
    return dst2

##
# Morphological opening
def dilate(cv_image, n_times=1):
    dst  = cv.cvCloneImage(cv_image)
    cv.cvDilate(cv_img, dst, None, n_times)
    return dst

##
# Morphological opening
def erode(cv_image, n_times=1):
    dst  = cv.cvCloneImage(cv_image)
    cv.cvErode(cv_img, dst, None, n_times)
    return dst


##
# Mask a color image with a given black and white mask
# @param img
# @param img_mask one channeled image
# @return color image with masked part filled with black
def mask(img, img_mask):
    dim      = img.width, img.height
    depth    = img.depth
    channels = img.nChannels

    r_chan = cv.cvCreateImage(cv.cvSize(*dim), depth, 1)
    g_chan = cv.cvCreateImage(cv.cvSize(*dim), depth, 1)
    b_chan = cv.cvCreateImage(cv.cvSize(*dim), depth, 1)
    combined = cv.cvCreateImage(cv.cvSize(*dim), depth, 3)
    cv.cvSplit(img, r_chan, g_chan, b_chan, None)

    cv.cvAnd(r_chan, img_mask, r_chan)
    cv.cvAnd(g_chan, img_mask, g_chan)
    cv.cvAnd(b_chan, img_mask, b_chan)
    cv.cvMerge(r_chan, g_chan, b_chan, None, combined)
    return combined

##
# Split a color image into its component parts
def split(image):
    img1 = cv.cvCreateImage(cv.cvSize(image.width, image.height), 8, 1)
    img2 = cv.cvCreateImage(cv.cvSize(image.width, image.height), 8, 1)
    img3 = cv.cvCreateImage(cv.cvSize(image.width, image.height), 8, 1)
    cv.cvSplit(image, img1, img2, img3, None)
    return (img1, img2, img3)

##
# Easy scaling of an image by factor s
# @param image
# @param s
def scale(image, s):
    scaled = cv.cvCreateImage(cv.cvSize(int(image.width * s), int(image.height * s)), image.depth, image.nChannels)
    cv.cvResize(image, scaled, cv.CV_INTER_AREA)
    return scaled

if __name__ == '__main__':
    import opencv.highgui as hg
    import rospy
    from photo.srv import *
    #a = create_ros_image()
    #print a

    rospy.wait_for_service('/photo/capture')
    say_cheese = rospy.ServiceProxy('/photo/capture', Capture)
    ros_img = say_cheese().image
    print dir(ros_img)
    cv_img = ros2cv(ros_img)
    hg.cvSaveImage('test.png', cv_img)

#def create_ros_image(width=1, height=1, channels=2, data='12'):
#    d1 = MultiArrayDimension(label='height',   size=height,   stride=width*height*channels)
#    d2 = MultiArrayDimension(label='width',    size=width,    stride=width*channels)
#    d3 = MultiArrayDimension(label='channels', size=channels, stride=channels)
#
#    layout     = MultiArrayLayout(dim = [d1,d2,d3])
#    multiarray = UInt8MultiArray(layout=layout, data=data)
#    return RosImage(label='image', encoding='bgr', depth='uint8', uint8_data=multiarray)

###
## Fill holes in a binary image using scipy
##
#def fill_holes(cv_img):
#    img_np     = ut.cv2np(cv_img)
#    img_binary = img_np[:,:,0]
#    results    = ni.binary_fill_holes(img_binary)
#    img_cv     = ut.np2cv(np_mask2np_image(results))
#    return img_cv
