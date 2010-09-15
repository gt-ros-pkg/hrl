import math
import numpy as np
import pylab as pb
import sys


#number of times until success
coin_success_prob = float(sys.argv[1])
counts = []
for i in range(1000):
    success = False
    count = 0
    while not success:
        #flip coin, increment
        a = np.random.rand()
        if a > coin_success_prob:
            success = True
        else:
            count = count + 1
    counts.append(count)

print np.mean(counts)
print np.std(counts)
#print counts
pb.hist(counts)
pb.show()

