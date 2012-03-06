#!/usr/bin/env python

import random
import time
import sys

_X_MIN = 0.35
_X_MAX = 0.85
_Y_MIN = -0.35
_Y_MAX = 0.35
_THETA_MIN = 1
_THETA_MAX = 180
def main(num_objs):
    random.seed(time.time())
    for i in xrange(num_objs):
        x = random.random()*(_X_MAX-_X_MIN) + _X_MIN
        y = random.random()*(_Y_MAX-_Y_MIN) + _Y_MIN
        theta = random.randint(_THETA_MIN, _THETA_MAX)

        print 'Start pose for obj ' + str(i) + ': (' + \
            str(x) + ', ' + str(y) + ', ' + str(theta) + ')'

if __name__ == '__main__':
    main(int(sys.argv[1]))
