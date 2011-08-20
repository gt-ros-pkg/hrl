
import os, sys, tty
import numpy as np
import cPickle as pk
import time
import threading
import math

from hrl_lib.msg import NumpyArray

## Returns a string that can be used as a timestamp (hours and minutes) in logfiles
# @return timestamp-string
def getTime():
    return '['+time.strftime("%H:%M:%S", time.localtime())+']'

#copied from manipulation stack
#angle between two quaternions (as lists)                                                                         
def quat_angle(quat1, quat2):                                                                               
    dot = sum([x*y for (x,y) in zip(quat1, quat2)])
    if dot > 1.:                                                                                                  
        dot = 1.                                                                                                  
    if dot < -1.:                                                                                                 
        dot = -1.                                                                                                 
    angle = 2*math.acos(math.fabs(dot))                                                                           
    return angle     


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
    try:
        p = open(filename, 'r')
    except IOError:
        print "hrl_lib.util: Pickle file cannot be opened."
        return None
    try:
        picklelicious = pk.load(p)
    except ValueError:
        print 'util.load_pickle failed once, trying again'
        p.close()
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
    import rospy
    rospy.loginfo('hrl_lib.util.bound is DEPRECATED. Please use numpy.clip instead')
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


## raw_input + matplotlib + ROS == strangeness.
# use this function instead.
def get_keystroke(msg):
    print msg
    # clear out anythin in the buffer.
    tty.setraw(sys.stdin, tty.TCSAFLUSH)
    # next 4 lines copied from m3.toolbox from Meka Robotics.
    os.system('stty raw')
    r = sys.stdin.read(1)
    os.system('stty sane')
    return r

## execute a bash command and get its output as a list of strings.
# e.g get_bash_command_output('ls -t')
def get_bash_command_output(cmd):
    p = os.popen(cmd)
    output_l = [s.rstrip('\n') for s in p.readlines()]
    p.close()
    return output_l






