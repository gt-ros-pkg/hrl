
import matplotlib.pyplot as pp
import numpy as np

import roslib; roslib.load_manifest('hrl_pr2_door_opening')
roslib.load_manifest('equilibrium_point_control')

import hrl_lib.util as ut
import equilibrium_point_control.arm_trajectories_ram as atr

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



