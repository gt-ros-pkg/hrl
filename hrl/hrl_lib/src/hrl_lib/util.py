import numpy as np

def list_mat_to_mat(list_mat, axis=0):
    return np.concatenate(tuple(list_mat), axis=axis)

