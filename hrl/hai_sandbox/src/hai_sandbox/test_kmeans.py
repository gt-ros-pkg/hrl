#from scipy.cluster.vq import vq
#from scipy.cluster.vq import whiten

import scipy.cluster.vq as vq
import numpy as np
#Testing kmeans cluster in scipy

features  = np.array([[ 1.9,2.3],
                      [ 1.5,2.5],
                      [ 0.8,0.6],
                      [ 0.4,1.8],
                      [ 0.1,0.1],
                      [ 0.2,1.8],
                      [ 2.0,0.5],
                      [ 0.3,1.5],
                      [ 1.0,1.0]])
whitened = vq.whiten(features)
book = np.array((whitened[0],whitened[2]))
vq.kmeans(whitened, book)

