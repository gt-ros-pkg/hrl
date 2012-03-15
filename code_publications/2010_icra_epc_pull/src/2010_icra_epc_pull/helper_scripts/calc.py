#
#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)

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


