
#
#
# Running mean and variance with supposedly superior numerical
# stability.
#
# Code based on the explanation and C++ code from this website:
# http://www.johndcook.com/standard_deviation.html
#
# This better way of computing variance goes back to a 1962 paper by
# B. P. Welford and is presented in Donald Knuth's Art of Computer
# Programming, Vol 2, page 232, 3rd edition. 
#
#
# Author: Advait Jain


import numpy as np
import copy


class Running_Mean_And_Variance():
    def __init__(self):
        self.mn = None
        self.s = 0
        self.k = 0

    def update(self, x):
        self.k += 1
        if self.k == 1:
            self.mn = x
            self.s = 0.
        else:
            mn_old = self.mn
            self.mn = mn_old + (x - mn_old) / self.k
            self.s = self.s + (x - mn_old) * (x - self.mn)

    def mean(self):
        return copy.copy(self.mn)

    def std(self):
        return np.sqrt(self.variance())

    def variance(self):
        return copy.copy(self.s/(self.k-1))

    def biased_std(self):
        return np.sqrt(self.biased_variance())

    def biased_variance(self):
        return copy.copy(self.s/(self.k))



if __name__ == '__main__':
    a = np.random.randn(4, 100)

    print 'Mean and Std computed using numpy:'
    print 'Mean:', np.mean(a, 1)
    print 'Std:', np.std(a, 1)

    print ''
    print 'Running Mean and Std:'

    rmav = Running_Mean_And_Variance()
    for b in a.T:
        rmav.update(b)

    print 'Mean:', rmav.mean()
    print 'Unbiased Std:', rmav.std()
    print 'Biased Std:', rmav.biased_std()





