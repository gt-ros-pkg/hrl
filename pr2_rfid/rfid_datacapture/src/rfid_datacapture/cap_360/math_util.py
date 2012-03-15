import numpy as np, math
import numpy
import math

import sys

##
# Bound the value of a number to be above lower, and lower than upper
# @return a number
def bound(value, lower, upper):
    raise RuntimeError('math_util.bound moved to hrl_lib.util')
    #sys.exit()
#    if lower >= upper:
#        t = lower
#        lower = upper
#        upper = t
#    #print 'bound', value, 'lower', lower, 'upper', upper
#    #return min(max(value, lower), upper)
#    return min(max(value, lower), upper)

def bound_mat(m, lower, upper):
    if lower >= upper:
        t = lower
        lower = upper
        upper = t

    m = m.copy()
    m[np.where(m > upper)] = upper
    m[np.where(m < lower)] = lower
    return m

def approx_equal(a, b, epsilon=.001):
    return (b < (a+epsilon)) and ((a-epsilon) < b)

def approx_equalv(a, b, epsilon=.001):
    return np.all(np.abs(a - b) < epsilon)

def radians(mat):
    return mat * (np.pi/180.0)

def degrees(mat):
    return mat * (180.0 / np.pi)

##################################################################
#                   Angles related functions
##################################################################
def vec_of_ang(a):
    return numpy.matrix([numpy.cos(a), numpy.sin(a)]).T

def ang_of_vec(a):
    a = a / numpy.linalg.norm(a)
    return math.atan2(a[1,0], a[0,0])

def avg_ang(wa, a, wb, b):
    """
         Calculates the average between two angles
         wa  weight of first angle
         a   first angle
         wb  weight of second angle
         b   second angle
    """
    return ang_of_vec(wa * vec_of_ang(a) + wb * vec_of_ang(b))

def blend_ang(alpha, a, b):
    return avg_ang(alpha, a, 1-alpha,b)

def standard_rad(t):
    if t > 0:
        return ((t + numpy.pi) % (numpy.pi * 2))  - numpy.pi
    else:
        return ((t - numpy.pi) % (numpy.pi * -2)) + numpy.pi

def best_turn_dir(reference, new_angle):
    """ positive is left, negative is right! """
    return standard_rad(reference - new_angle)

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

##################################################################
# NUMPY HELPER FUNCTIONS
##################################################################

##
# Find the maximal position in a 2D array
# @return (r,c)
def argmax2d(mat):
    max_1d     = np.argmax(mat)
    row_length = float(mat.shape[1])
    row_idx    = np.floor(max_1d / row_length)
    col_idx    = max_1d % row_length
    return row_idx, col_idx

##
# Changes the range of an numpy array to betwen [0, 255] from it's [min_value, max_value]
def renormalize(npimage_gray):
    min_img = np.min(npimage_gray)
    max_img = np.max(npimage_gray)
    ret =  np.matrix(np.round(((npimage_gray - min_img) / (max_img - min_img) * 255.0)), 'uint8')
    return ret

def list_mat_to_mat(list_mat, axis=0):
    return np.concatenate(tuple(list_mat), axis=axis)

def list_of_mat(mat):
    for i in range(mat.shape[1]):
        yield mat[:,i]

def nearest(mat, target):
    '''
        Return a sorted list of the nearest (euclidean dist) element
        of a matrix to a target value and their indeices.
        @param mat    mxn
        @param target mx1
    '''
    #mat = mat.T
    #target = target.T
    #import util as ut
    #import pdb
    #ut.save_pickle(mat, 'mat.pkl')
    #ut.save_pickle(target, 'target.pkl')

    diff_vec = mat - target
    pwr = np.ones_like(mat[0])*2
    dist = np.power(np.sum(np.power(diff_vec, pwr),axis=0),0.5)
    indices = dist.argsort(axis=1)

    #want indices from sort order
    #indices = np.concatenate((np.matrix(range(sort_order.shape[1])), sort_order), 0)[:, sort_order.A1]
    #print sort_order
    #print indices
    #pdb.set_trace()

    return mat[:, indices.A1], indices

if __name__ == '__main__':
    import hrl_lib.util as ut
    import pdb
    mat = ut.load_pickle('mat.pkl')
    target = ut.load_pickle('target.pkl')

    diff_vec = mat - target
    pwr = np.ones_like(mat[0])*2
    dist = np.power(np.sum(np.power(diff_vec, pwr),axis=0),0.5)
    sort_order = dist.argsort(axis=1)

    #want indices from sort order
    indices = np.concatenate((np.matrix(range(sort_order.shape[1])), sort_order), 0)[:, sort_order.A1]

    print sort_order
    print indices
    pdb.set_trace()

#def nearest(mat, target):
#    '''
#    Return a sorted list of the nearest (euclidean dist) element
#    of a matrix to a target value and their indeices.
#    '''
#    mat = mat.T
#    target = target.T
#    diff_vec = mat - target
#    pwr = np.ones_like(mat[0])*2
#    dist = np.power(np.sum(np.power(diff_vec ,pwr),axis=1),0.5)
#    indices = dist.argsort(axis=0)

#    return mat[indices.A1], indices

##################################################################
# MISCELLANEOUS MATH HELPER FUNCTIONS
##################################################################
def blend(alpha, a, b):
    return (alpha * a) + ((1-alpha) * b)

def approx_equal(a, b, epsilon=.001):
    return (b < (a+epsilon)) and ((a-epsilon) < b)

def norm(mat):
    """
        Calculate L2 norm for column vectors in a matrix
    """
    return np.power(np.sum(np.power(mat,2), axis=0), 0.5)

def rnd(v):
    return int(round(v))

def point_to_homo(p):
    """ Convert points into homogeneous coordinates """
    o = numpy.matrix(numpy.ones((1, p.shape[1])))
    return numpy.vstack((p,o))

    """ Convert points back from homogeneous coordinates """
def homo_to_point(p):
    return p[0:p.shape[0]-1,:]
