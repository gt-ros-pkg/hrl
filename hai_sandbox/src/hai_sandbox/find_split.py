import numpy as np
import pdb

def max_wrt_constrain(a, b, constraint_f, toler):
    return find_split(a,  None, b, None, constraint_f, toler)

def min_wrt_constrain(a, b, constraint_f, toler):
    return find_split(b,  None, a, None, constraint_f, toler)

def find_split(a, qa, b, qb, constraint_f, toler):
    #pdb.set_trace()
    print 'a', a, qa,  'b', b, qb
    #assume we're maximizing (or going towards b)
    if abs(b - a) < toler:
        if qb == True:
            return b
        elif qa == True:
            return a
        else:
            raise RuntimeError('min interval reached without finding a point that returns success')
    else:
        nqb = constraint_f(b)

        if nqb:
            return b

        else:
            mid = (a + b) / 2.0
            nmid = constraint_f(mid)
            if not nmid:
                return find_split(a,   qa,   mid, nmid, constraint_f, toler)
            else:
                return find_split(mid, True, b,   nqb, constraint_f, toler)

def my_func(input):
    if input > 5.5:
        return True
    else:
        return False

#print 'returned', find_split(-10.0, None, 20, None, my_func, .2)
print 'returned', find_split(20.0, None, -10., None, my_func, .1)
