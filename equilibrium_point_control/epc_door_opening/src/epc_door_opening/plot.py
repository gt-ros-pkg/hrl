#
# Copyright (c) 2009, Georgia Tech Research Corporation
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

# Author: Advait Jain

import matplotlib.pyplot as pp
import numpy as np

import roslib; roslib.load_manifest('epc_door_opening')

import hrl_lib.util as ut
import doors_forces_kinematics.arm_trajectories_ram as atr

d = ut.load_pickle('pkls/ikea_cabinet_log.pkl')
#d = ut.load_pickle('pkls/ikea_cabinet_2.pkl')
#d = ut.load_pickle('pkls/lab_cabinet_log.pkl')

typ = 'rotary'
pr2_log = True

d['f_list'] = d['f_list_estimate']
h_config, h_ftan_estimate = atr.force_trajectory_in_hindsight(d, typ, pr2_log)
pp.plot(np.degrees(h_config), h_ftan_estimate, 'ro-', mew=0, ms=0,
        label='estimate')

if 'f_list_torques' in d:
    d['f_list'] = d['f_list_torques']
    h_config, h_ftan_torques = atr.force_trajectory_in_hindsight(d, typ,
                                                                 pr2_log)
    pp.plot(np.degrees(h_config), h_ftan_torques, 'go-', mew=0, ms=0,
            label='torques')

d['f_list'] = d['f_list_ati']
h_config, h_ftan_ati = atr.force_trajectory_in_hindsight(d, typ, pr2_log)
pp.plot(np.degrees(h_config), h_ftan_ati, 'bo-', mew=0, ms=0,
        label='ATI')

pp.legend()
pp.show()



