import numpy as np
from scipy import interpolate

##
# filter a list given indices
# @param alist a list
# @param indices indices in that list to select
def filter(alist, indices):    
    rlist = []
    for i in indices:
        rlist.append(alist[i])
    return rlist

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
        start_times.append(tarray[0,0])
        end_times.append(tarray[0,-1])

    #print start_times
    #print end_times
    #import pdb
    #pdb.set_trace()
    min_start = np.min(start_times)
    max_end = np.max(end_times)

    adjusted_list_of_time_arrays = []
    for tarray in list_of_time_arrays:
        adjusted_list_of_time_arrays.append(tarray - min_start)

    return adjusted_list_of_time_arrays, min_start, max_end

##
# calc dx/dt
# @param t matrix 1xn
# @param x matrix mxn
def gradient(t, x):
    #pdb.set_trace()
    dx = x[:, 2:] - x[:, 0:-2]
    dt = t[0, 2:] - t[0, 0:-2]
    dx_dt = np.multiply(dx, 1/dt)
    #pdb.set_trace()
    dx_dt = np.column_stack((dx_dt[:,0], dx_dt))
    dx_dt = np.column_stack((dx_dt, dx_dt[:,-1]))
    return dx_dt

##
# 1D interpolation
#
# @param x 1xn mat x to interpolate from
# @param y 1xn mat y to interpolate from
# @param xquery 1xn mat of query x's 
def interpolate_1d(x, y, xquery):
    try:
        x = x.A1
        y = y.A1
        xquery = xquery.A1
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
# smooth the data using a window with requested size.
# 
# This method is based on the convolution of a scaled window with the signal.
# The signal is prepared by introducing reflected copies of the signal 
# (with the window size) in both ends so that transient parts are minimized
# in the begining and end part of the output signal.
# 
# output:
#     the smoothed signal
#     
# example:
# 
# t=linspace(-2,2,0.1)
# x=sin(t)+randn(len(t))*0.1
# y=smooth(x)
# 
# see also: 
# 
# numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
# scipy.signal.lfilter
# 
# Copied from http://www.scipy.org/Cookbook/SignalSmooth
# 
# @param    x the input signal 
# @param    window_len the dimension of the smoothing window; should be an odd integer
# @param    window the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
#           flat window will produce a moving average smoothing.
# @return   the smoothed signal function
def signal_smooth(x,window_len=11,window='hamming'):

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"

    s=np.r_[x[window_len:1:-1],x,x[-1:-window_len:-1]]

    # s=numpy.r_[2*x[0]-x[window_len:1:-1],x,2*x[-1]-x[-1:-window_len:-1]]
    #print(len(s))
    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='same')
    return y[window_len-1:-window_len+1]

##
# Returns the variance of the series x given mean function y
# over a window of size window_len.
# @param x the original signal
# @param y the smoothed signal function
# @param window_len size of the window to calculate variances over
# @return the variance function
def signal_variance(x, y, window_len=10):
    if len(x) != len(y):
        raise ValueError, "Must have same length"

    vars = []
    for i in range(len(x)):
        cursum = 0. 
        cura = i - window_len/2
        curb = i + window_len/2
        if cura < 0:
            cura = 0
        if curb > len(x):
            curb = len(x)
        for xval in x[cura:curb]:
            cursum += (xval - y[i])**2
        vars += [cursum / (curb-cura)]
    vars += [vars[len(vars)-1]]
    return vars

##
# TODO docs
# Returns the variance of the series x given mean function y
# over a window of size window_len.
# @param x the original signal
# @param y the smoothed signal function
# @param window_len size of the window to calculate variances over
# @return the variance function
def signal_list_variance(x_list, means, window_len=10, num_samples=30, resample=1):
    # if len(x_list[0]) != len(means):
    #     raise ValueError, "Must have same length"

    vars = []
    num_samples_in_mean = num_samples / len(x_list)
    for i in range(0, len(means), resample):
        cursum = 0.
        cura = i - window_len/2
        curb = i + window_len/2
        if cura < 0:
            cura = 0
        if curb > len(means):
            curb = len(means)
        step = (curb - cura) / num_samples_in_mean
        n = 0
        for x in x_list:
            if cura >= len(x):
                continue

            ccurb = curb
            cstep = step
            if ccurb >= len(x):
                ccurb = len(x)
                cstep = (ccurb - cura) / num_samples_in_mean
            if cstep > 0:
                for xval in x[cura:ccurb:cstep]:
                    cursum += (xval - means[i])**2
                    n += 1
        vars += [np.sqrt(cursum)/(n)]
    return np.array(vars)

class FastGaussConvolve():
    def __init__(self, delta = 0.001, max_sigma = 5.0):
        self.kernels = []
        self.inds = np.linspace(-max_sigma, max_sigma, int(2 * max_sigma / delta) + 1)
        for deg in range(3):
            kernel = []
            for v in self.inds:
                if deg == 0:
                    kernel.append(self.gauss(v))
                elif deg == 1:
                    kernel.append(self.gauss_d1(v))
                elif deg == 2:
                    kernel.append(self.gauss_d2(v))
            self.kernels.append(kernel)
        self.delta = delta
        self.max_sigma = max_sigma

    def _gauss_lookup(self, t, sigma_t, deg):
        xs = t / sigma_t
        if xs < -self.max_sigma or xs > self.max_sigma:
            return None
        gind = int((xs + self.max_sigma) / self.delta)
        return self.kernels[deg][gind]
    
    def convolve_pt(self, signal, time_series, ind, deg, sigma_t):
        ret = 0.
        ind_dwn = ind
        gauss_sum = 0.
        while ind_dwn > 0:
            time = time_series[ind_dwn] - time_series[ind]
            gauss = self._gauss_lookup(time, sigma_t, deg)
            if gauss is None:
                break
            ret += gauss * signal[ind_dwn]
            gauss_sum += np.fabs(gauss)
            ind_dwn -= 1
        ind_up = ind + 1
        while ind_up < len(signal):
            time = time_series[ind_up] - time_series[ind]
            gauss = self._gauss_lookup(time, sigma_t, deg)
            if gauss is None:
                break
            ret += gauss * signal[ind_up]
            gauss_sum += np.fabs(gauss)
            ind_up += 1
        return ret / gauss_sum

    def convolve_signal(self, signal, time_series, deg, sigma_t):
        window_len = int(self.max_sigma * sigma_t / self.delta)

        avg_sig = sum(signal[0:20]) / 20.
        s=np.r_[[avg_sig]*window_len,signal,[avg_sig]*window_len]
        avg_diff = sum([time_series[i] - time_series[i-1] for i in range(1,20)]) / 19.
        ts=np.r_[[avg_diff*i + time_series[0] for i in range(-window_len, 0)],time_series,[avg_diff*i + time_series[-1] for i in range(1, window_len+1)]]

        ret_sig = []
        i = window_len
        while len(ret_sig) < len(signal):
            ret_sig.append(self.convolve_pt(s, ts, i, deg, sigma_t))
            i += 1

        return ret_sig

    def gauss(self, x, sig=1.):
        return 1./np.sqrt(2. * np.pi * sig ** 2) * np.exp(-x**2/(2*sig**2))

    def gauss_d1(self, x, sig=1.):
        return (-2. * x * 1./(np.sqrt(2. * np.pi * sig ** 2) * 2 * sig**2) * 
                np.exp(-x**2/(2*sig**2)))

    def gauss_d2(self, x, sig=1.):
        return (-2. * 1./(np.sqrt(2. * np.pi * sig ** 2) * 2 * sig**2) * 
                np.exp(-x**2/(2*sig**2)) +
                4. * x**2 * 1./(np.sqrt(2. * np.pi * sig ** 2) * 4 * sig**4) * 
                np.exp(-x**2/(2*sig**2)))
