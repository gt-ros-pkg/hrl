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
    #print 'bound', value, 'lower', lower, 'upper', upper
    #return min(max(value, lower), upper)
    return min(max(value, lower), upper)



