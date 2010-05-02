import numpy as np
import hrl_lib.util as ut

#---- X axis -----
#p = ut.load_pickle('eq_pos_2010May01_213254.pkl')
#d = ut.load_pickle('stiffness_2010May01_213323.pkl')
#n = 0


#--- Y axis ----
#p = ut.load_pickle('eq_pos_2010May01_213832.pkl')
#d = ut.load_pickle('stiffness_2010May01_213907.pkl')
#n = 1

#--- Z axis
p = ut.load_pickle('eq_pos_2010May01_214434.pkl')
d = ut.load_pickle('stiffness_2010May01_214512.pkl')
n = 2

pos_list = d['pos_list']
force_list = d['force_list']


stiff_list = []
for pos,force in zip(pos_list,force_list):
    dx = pos[n,0]-p[n,0]
    f = force[n,0]
    print 'force:',f
    print 'dx:',dx
    stiff_list.append(f/dx)

print stiff_list


