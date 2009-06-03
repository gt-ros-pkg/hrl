import numpy as np
import pickle as pk
import time

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
# @param return - object that had been pickled.
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




