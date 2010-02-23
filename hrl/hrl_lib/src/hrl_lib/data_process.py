import numpy as np
from scipy import interpolate

def filter(alist, indices):    
    rlist = []
    for i in indices:
        rlist.append(alist[i])
    return rlist

def gradient(t, x):
    #pdb.set_trace()
    dx = x[:, 2:] - x[:, 0:-2]
    dt = t[0, 2:] - t[0, 0:-2]
    dx_dt = np.multiply(dx, 1/dt)
    #pdb.set_trace()
    dx_dt = np.column_stack((dx_dt[:,0], dx_dt))
    dx_dt = np.column_stack((dx_dt, dx_dt[:,-1]))
    return dx_dt

def interpolate_1d(x, y, xquery):
    try:
        minx = np.min(x)
        minx_query = np.min(xquery)

        maxx = np.max(x)
        maxx_querry = np.max(xquery)

        if minx_query <= minx:
            x = np.concatenate((np.array([minx_query-.01]), x))
            y = np.concatenate((np.array([y[0]]), y))

        if maxx <= maxx_querry:
            x = np.concatenate((x, np.array([maxx_querry+.01])))
            y = np.concatenate((y, np.array([y[-1]])))

        f = interpolate.interp1d(x, y)
        return f(xquery)
    except ValueError, e:
        pdb.set_trace()
        print e

##
# Given a histogram with params, calculate 
def histogram_get_bin_numb(n, min_index, bin_size, nbins):
    bin_numb = int(np.floor((n - min_index) / bin_size))
    if bin_numb == nbins:
        bin_numb = bin_numb - 1
    return bin_numb

##
#
#
# @param index_list_list a list of list of indices to histogram by
# @param elements_list_list a list of list of elements to place in histogram bins
# @param bin_size size of bins in index_list_list units
# @param min_index optional argument for mininum index to create histogram over
# @param max_index optional argument for maximum index to create histogram over
def histogram(index_list_list, elements_list_list, bin_size, min_index=None, max_index=None):
    if min_index is None:
        min_index = np.min(np.concatenate(index_list_list))
    if max_index is None:
        max_index = np.max(np.concatenate(index_list_list))

    index_range = (max_index - min_index) 
    nbins = int(np.ceil(index_range / bin_size))
    bins = []
    for i in range(nbins):
        bins.append([])

    #pdb.set_trace()

    #Each slice contains the data for one trial, idx is the trial number
    for trial_number, element_list_slice in enumerate(zip(*elements_list_list)):
        #Iterate by using the length of the first set of data in the given trial
        for i in range(len(element_list_slice[0])):
            bin_numb = histogram_get_bin_numb(index_list_list[trial_number][i], min_index, bin_size, nbins)
            elements = [el_list[i] for el_list in element_list_slice]
            if bin_numb < 0 or bin_numb > nbins:
                continue
            bins[bin_numb].append(elements)

    return bins, np.arange(min_index, max_index, bin_size)        


##
# Given a list of 1d time arrays, find the sequence that started first and
# subtract all sequences from its first time recording.
#
# @param list_of_time_arrays a list of 1d arrays 
# @return list_of_time_arrays adjusted so that time arrays would start at 0
def equalize_times(list_of_time_arrays):
    start_times = []
    end_times = []
    for tarray in list_of_time_arrays:
        start_times.append(tarray[0])
        end_times.append(tarray[-1])

    min_start = np.min(start_times)
    max_end = np.max(end_times)

    adjusted_list_of_time_arrays = []
    for tarray in list_of_time_arrays:
        adjusted_list_of_time_arrays.append(tarray - min_start)

    return adjusted_list_of_time_arrays, min_start, max_end
