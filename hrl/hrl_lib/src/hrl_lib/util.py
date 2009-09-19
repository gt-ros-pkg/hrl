import numpy as np
import pickle as pk
import time

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


from hrl_lib.msg import NumpyArray

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






