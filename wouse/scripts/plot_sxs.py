#!/usr/bin/python

import numpy as np
import pickle
import matplotlib.pyplot as plt

def main():
    with open('pickled_mn_std.pkl','rb') as f:
        (means, stds) = pickle.load(f)
    width = 1./means.shape[1]
    ind = np.arange(means.shape[0])*2
    
#
 #   b = ['']*means.shape[0]
    colors=[(1,0,0,1),
            (1,0.3,0.3,1),
            (0.6,1,0.6,1),
            (1,1,1,1),
            (0,0.3,0.6),
            (0.3,0,0.6),
            (0.6,0.6,1),
            (0.6,0,0.3)]
    for i in range(means.shape[1]):
        for j in range(means.shape[0]):
            plt.bar(ind[i]+j*width, means[j,i], width, color=colors[j], yerr=stds[j,i])

    plt.show()
if __name__=='__main__':
    main()
