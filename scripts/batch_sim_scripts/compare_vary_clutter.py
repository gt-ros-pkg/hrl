
import os
import matplotlib.pyplot as pp
import numpy as np, math
import scipy.stats as ss
import copy

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

base_path = '/home/mkillpack'

def return_lists_50_percent_each():
    ft_sensor_nm_list = [
                         '75_obstacle_configurations_movable010_fixed010/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_ft_sensor',

                         # '../marc_test_force_switch_direction_error/75_obstacle_configurations_movable010_fixed010/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_force_switch_direction_error/75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_force_switch_direction_error/75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_force_switch_direction_error/75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_force_switch_direction_error/75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_ft_sensor',

                         # '../marc_test_new_ft_sensor/75_obstacle_configurations_movable010_fixed010/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_new_ft_sensor/75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_new_ft_sensor/75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_new_ft_sensor/75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_ft_sensor',
                         # '../marc_test_new_ft_sensor/75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_ft_sensor',
                        ]

    skin_nm_list = [
                    '75_obstacle_configurations_movable010_fixed010/mpc_qs_1_using_skin',
                    '75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_skin',
                    '75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_skin',
                    '75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_skin',
                    '75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_skin',
                   ]

    baseline_nm_list = [
                        '75_obstacle_configurations_movable010_fixed010/mpc_qs_1_ignore_skin',
                        '75_obstacle_configurations_movable020_fixed020/mpc_qs_1_ignore_skin',
                        '75_obstacle_configurations_movable040_fixed040/mpc_qs_1_ignore_skin',
                        '75_obstacle_configurations_movable060_fixed060/mpc_qs_1_ignore_skin',
                        '75_obstacle_configurations_movable080_fixed080/mpc_qs_1_ignore_skin',
                       ]

    openrave_nm_list = [
                        '75_obstacle_configurations_movable010_fixed010/openrave_128_samples',
                        '75_obstacle_configurations_movable020_fixed020/openrave_128_samples',
                        '75_obstacle_configurations_movable040_fixed040/openrave_128_samples',
                        '75_obstacle_configurations_movable060_fixed060/openrave_128_samples_v2',
                        '75_obstacle_configurations_movable080_fixed080/openrave_128_samples',
                       ]

    multiple_reach_nm_list = [
                              '75_obstacle_configurations_movable010_fixed010/mpc_qs_1_multiple_reaches_using_skin',
                              '75_obstacle_configurations_movable020_fixed020/mpc_qs_1_multiple_reaches_using_skin',
                              '75_obstacle_configurations_movable040_fixed040/mpc_qs_1_multiple_reaches_using_skin',
                              '75_obstacle_configurations_movable060_fixed060/mpc_qs_1_multiple_reaches_using_skin',
                              '75_obstacle_configurations_movable080_fixed080/mpc_qs_1_multiple_reaches_using_skin',




    # ft_sensor_nm_list = [
    #                      '75_obstacle_configurations_aug_movable10_fixed10/qs_mpc1_with_ft_sensor',
    #                      '75_obstacle_configurations_apr_29_movable20_fixed20/mpc_qs_1_20120429_using_ft_sensor',
    #                      '75_obstacle_configurations_apr_28_movable40_fixed40/mpc_qs_1_20120428_using_ft_sensor',
    #                      '75_obstacle_configurations_apr_29_movable60_fixed60/mpc_qs_1_20120429_using_ft_sensor',
    #                      '75_obstacle_configurations_apr_29_movable80_fixed80/mpc_qs_1_20120429_using_ft_sensor',
    #                     ]

    # skin_nm_list = [
    #                 '75_obstacle_configurations_aug_movable10_fixed10/qs_mpc1_with_skin',
    #                 '75_obstacle_configurations_apr_29_movable20_fixed20/mpc_qs_1_20120429_using_skin',
    #                 '75_obstacle_configurations_apr_28_movable40_fixed40/mpc_qs_1_20120428_using_skin',
    #                 '75_obstacle_configurations_apr_29_movable60_fixed60/mpc_qs_1_20120429_using_skin',
    #                 '75_obstacle_configurations_apr_29_movable80_fixed80/mpc_qs_1_20120429_using_skin',
    #                ]

    # baseline_nm_list = [
    #                     '75_obstacle_configurations_aug_movable10_fixed10/qs_mpc1_ignore_skin',
    #                     '75_obstacle_configurations_apr_29_movable20_fixed20/mpc_qs_1_20120429_ignore_skin',
    #                     '75_obstacle_configurations_apr_28_movable40_fixed40/mpc_qs_1_20120428_ignore_skin',
    #                     '75_obstacle_configurations_apr_29_movable60_fixed60/mpc_qs_1_20120429_ignore_skin',
    #                     '75_obstacle_configurations_apr_29_movable80_fixed80/mpc_qs_1_20120429_ignore_skin',
    #                    ]

    # openrave_nm_list = [
    #                     '75_obstacle_configurations_aug_movable10_fixed10/openrave_16_samples',
    #                     '75_obstacle_configurations_apr_29_movable20_fixed20/openrave_128_samples',
    #                     '75_obstacle_configurations_apr_28_movable40_fixed40/openrave_128_samples',
    #                     '75_obstacle_configurations_apr_29_movable60_fixed60/openrave_128_samples',
    #                     '75_obstacle_configurations_apr_29_movable80_fixed80/openrave_128_samples',
    #                    ]

    # multiple_reach_nm_list = [
    #         '75_obstacle_configurations_aug_movable10_fixed10/qs_mpc1_with_skin_multiple_reaches',
    #         '75_obstacle_configurations_apr_29_movable20_fixed20/mpc_qs_1_20120429_multiple_reaches_using_skin',
    #         '75_obstacle_configurations_apr_28_movable40_fixed40/mpc_qs_1_20120428_multiple_reaches_using_skin',
    #         '75_obstacle_configurations_apr_29_movable60_fixed60/mpc_qs_1_20120429_multiple_reaches_using_skin',
    #         '75_obstacle_configurations_apr_29_movable80_fixed80/mpc_qs_1_20120429_multiple_reaches_using_skin',
                       ]
    
    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list, \
           openrave_nm_list, multiple_reach_nm_list

def return_lists_0_percent_fixed():
    ft_sensor_nm_list = [
                         '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_ft_sensor',
                        ]

    skin_nm_list = [
                     '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_skin',
                   ]

    baseline_nm_list = [
                         '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_ignore_skin',
                       ]

    openrave_nm_list = [
                       ]

    multiple_reach_nm_list = [
                   ]



    # ft_sensor_nm_list = [
    #                      '75_obstacle_configurations_aug_movable20_fixed00/qs_mpc1_with_ft_sensor',
    #                      '75_obstacle_configurations_july_09_movable40_fixed00/mpc_qs_1_using_ft_sensor',
    #                      '75_obstacle_configurations_july_09_movable80_fixed00/mpc_qs_1_using_ft_sensor',
    #                      '75_obstacle_configurations_july_09_movable120_fixed00/mpc_qs_1_using_ft_sensors',
    #                      '75_obstacle_configurations_july_09_movable160_fixed00/mpc_qs_1_using_ft_sensor',
    #                     ]

    # skin_nm_list = [
    #                  '75_obstacle_configurations_aug_movable20_fixed00/qs_mpc1_with_skin',
    #                  '75_obstacle_configurations_july_09_movable40_fixed00/mpc_qs_1_using_skin',
    #                  '75_obstacle_configurations_july_09_movable80_fixed00/mpc_qs_1_using_skin',
    #                  '75_obstacle_configurations_july_09_movable120_fixed00/mpc_qs_1_using_skin',
    #                  '75_obstacle_configurations_july_09_movable160_fixed00/mpc_qs_1_using_skin',
    #                ]

    # baseline_nm_list = [
    #                      '75_obstacle_configurations_aug_movable20_fixed00/qs_mpc1_ignore_skin',
    #                      '75_obstacle_configurations_july_09_movable40_fixed00/mpc_qs_1_ignore_skin',
    #                      '75_obstacle_configurations_july_09_movable80_fixed00/mpc_qs_1_ignore_skin',
    #                      '75_obstacle_configurations_july_09_movable120_fixed00/mpc_qs_1_ignore_skin',
    #                      '75_obstacle_configurations_july_09_movable160_fixed00/mpc_qs_1_ignore_skin',
    #                    ]

    # openrave_nm_list = [
    #                    ]

    # multiple_reach_nm_list = [
    #                ]

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list

def return_lists_25_percent_fixed():
    ft_sensor_nm_list = [
                         '75_obstacle_configurations_aug_movable15_fixed05/qs_mpc1_with_ft_sensor',
                         '75_obstacle_configurations_may_4_movable30_fixed10/mpc_qs_1_20120504_using_ft_sensor',
                         '75_obstacle_configurations_may_4_movable60_fixed20/mpc_qs_1_20120504_using_ft_sensor',
                         '75_obstacle_configurations_may_4_movable90_fixed30/mpc_qs_1_20120504_using_ft_sensor',
                         '75_obstacle_configurations_apr_29_movable120_fixed40/mpc_qs_1_20120429_using_ft_sensor',
                        ]

    skin_nm_list = [
                    '75_obstacle_configurations_aug_movable15_fixed05/qs_mpc1_with_skin',
                    '75_obstacle_configurations_may_4_movable30_fixed10/mpc_qs_1_20120504_using_skin',
                    '75_obstacle_configurations_may_4_movable60_fixed20/mpc_qs_1_20120504_using_skin',
                    '75_obstacle_configurations_may_4_movable90_fixed30/mpc_qs_1_20120504_using_skin',
                    '75_obstacle_configurations_apr_29_movable120_fixed40/mpc_qs_1_20120429_using_skin',
                   ]

    baseline_nm_list = [
                        '75_obstacle_configurations_aug_movable15_fixed05/qs_mpc1_ignore_skin',
                        '75_obstacle_configurations_may_4_movable30_fixed10/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_may_4_movable60_fixed20/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_may_4_movable90_fixed30/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_apr_29_movable120_fixed40/mpc_qs_1_20120429_ignore_skin',
                       ]

    openrave_nm_list = [
                        '75_obstacle_configurations_aug_movable15_fixed05/openrave_16_samples',
                        '75_obstacle_configurations_may_4_movable30_fixed10/openrave_128_samples',
                        '75_obstacle_configurations_may_4_movable60_fixed20/openrave_128_samples',
                        '75_obstacle_configurations_may_4_movable90_fixed30/openrave_128_samples',
                        '75_obstacle_configurations_apr_29_movable120_fixed40/openrave_128_samples',
                       ]

    multiple_reach_nm_list = [
                    '75_obstacle_configurations_aug_movable15_fixed05/qs_mpc1_with_skin_multiple_reaches',
                    '75_obstacle_configurations_may_4_movable30_fixed10/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_may_4_movable60_fixed20/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_may_4_movable90_fixed30/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_apr_29_movable120_fixed40/mpc_qs_1_20120429_multiple_reaches_using_skin',
                   ]

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list

def return_lists_75_percent_fixed(extra_datapoints = False):
    if not extra_datapoints:
        ft_sensor_nm_list = [
                             '75_obstacle_configurations_aug_movable05_fixed15/qs_mpc1_with_ft_sensor', 
                             '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_using_ft_sensor',
                            ]
        skin_nm_list = [
                        '75_obstacle_configurations_aug_movable05_fixed15/qs_mpc1_with_skin',
                        '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_using_skin',
                       ]
    else:
        ft_sensor_nm_list = [
                             '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable25_fixed75/mpc_qs_1_20120504_using_ft_sensor',
                             #'75_obstacle_configurations_may_10_movable28_fixed84/mpc_qs_1_20120510_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_using_ft_sensor',
                             #'75_obstacle_configurations_may_9_movable30_fixed90/mpc_qs_1_20120509_using_ft_sensor',
                             #'75_obstacle_configurations_may_10_movable30_fixed90/mpc_qs_1_20120510_using_ft_sensor',
                             #'75_obstacle_configurations_may_7_movable33_fixed99/mpc_qs_1_20120507_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable35_fixed105/mpc_qs_1_20120504_using_ft_sensor',
                             '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_using_ft_sensor',
                            ]
        skin_nm_list = [
                        '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable25_fixed75/mpc_qs_1_20120504_using_skin',
                        #'75_obstacle_configurations_may_10_movable28_fixed84/mpc_qs_1_20120510_using_skin',
                        '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_using_skin',
                        #'75_obstacle_configurations_may_9_movable30_fixed90/mpc_qs_1_20120509_using_skin',
                        #'75_obstacle_configurations_may_10_movable30_fixed90/mpc_qs_1_20120510_using_skin',
                        #'75_obstacle_configurations_may_7_movable33_fixed99/mpc_qs_1_20120507_using_skin',
                        '75_obstacle_configurations_may_4_movable35_fixed105/mpc_qs_1_20120504_using_skin',
                        '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_using_skin',
                       ]

    baseline_nm_list = [
                        '75_obstacle_configurations_aug_movable05_fixed15/qs_mpc1_ignore_skin',
                        '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_ignore_skin',
                        '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_ignore_skin',
                       ]

    openrave_nm_list = [
                        '75_obstacle_configurations_aug_movable05_fixed15/openrave_16_samples',
                        '75_obstacle_configurations_may_4_movable10_fixed30/openrave_128_samples',
                        '75_obstacle_configurations_may_4_movable20_fixed60/openrave_128_samples',
                        '75_obstacle_configurations_may_4_movable30_fixed90/openrave_128_samples',
                        '75_obstacle_configurations_may_4_movable40_fixed120/openrave_128_samples',
                       ]

    multiple_reach_nm_list = [
                    '75_obstacle_configurations_aug_movable05_fixed15/qs_mpc1_with_skin_multiple_reaches',
                    '75_obstacle_configurations_may_4_movable10_fixed30/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_may_4_movable20_fixed60/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_may_4_movable30_fixed90/mpc_qs_1_20120504_multiple_reaches_using_skin',
                    '75_obstacle_configurations_may_4_movable40_fixed120/mpc_qs_1_20120504_multiple_reaches_using_skin',
                   ]

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list

def return_lists_100_percent_fixed():
    ft_sensor_nm_list = ['75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_using_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_ft_sensor',
                        ]

    skin_nm_list = [        
                     '75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_skin',
                   ]

    baseline_nm_list = [
                         '75_obstacle_configurations_movable000_fixed020/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_ignore_skin',
                       ]

    openrave_nm_list = [
                         '75_obstacle_configurations_movable000_fixed020/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed040/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed080/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed120/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed160/openrave_128_samples',
                       ]

    multiple_reach_nm_list = [
                   ]

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list

def compute_success_rate(path, pkl_nm, nm_list):
    l = []
    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        l.append(d['success_count']*100. / (d['success_count']+d['fail_count']))
    return l

def compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                         ft_sensor_nm_list,
                                         skin_nm_list,
                                         baseline_nm_list,
                                         openrave_nm_list):
    ft_success_percent = []
    skin_success_percent = []
    baseline_success_percent = []
    openrave_success_percent = []
    perc_improv_skin_ft = []
    for ft_nm, skin_nm in zip(ft_sensor_nm_list, skin_nm_list):
        ft_dict = ut.load_pickle(os.path.join(path, ft_nm, pkl_nm))
        skin_dict = ut.load_pickle(os.path.join(path, skin_nm, pkl_nm))
        print 'ft_nm:', ft_nm
        ft_success_percent.append(ft_dict['success_count']*100./(ft_dict['success_count']+ft_dict['fail_count']))
        skin_success_percent.append(skin_dict['success_count']*100./(skin_dict['success_count']+skin_dict['fail_count']))
        perc_improv_skin_ft.append((skin_success_percent[-1] * 1. - ft_success_percent[-1]) / ft_success_percent[-1] *100) 
    
    for bs_nm, or_nm in zip(baseline_nm_list, openrave_nm_list):
        base_dict = ut.load_pickle(os.path.join(path, bs_nm, pkl_nm))
        or_dict = ut.load_pickle(os.path.join(path, or_nm, pkl_nm))
        baseline_success_percent.append(base_dict['success_count']*100./(base_dict['success_count']+base_dict['fail_count']))
        openrave_success_percent.append(or_dict['success_count']*100./(or_dict['success_count']+or_dict['fail_count']))
    
    return ft_success_percent, skin_success_percent, baseline_success_percent, openrave_success_percent, perc_improv_skin_ft

def fixed_0_percent():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'combined_results.pkl'

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = return_lists_0_percent_fixed()

    obj_count_list = [20, 40, 80, 120, 160]
    #obj_count_list = [40, 80, 120]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, skin_nm_list)

    openrave_success_percent = [100 for o in openrave_success_percent]
    pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, skin_success_percent, 'b-o', mew=0,
            label='MPC with tactile sensing')
    pp.plot(obj_count_list, ft_success_percent, 'g-o', mew=0, label='MPC with FT sensors')
    pp.plot(obj_count_list, baseline_success_percent, 'y-o', mew=0,
            label='MPC without tactile\n or FT sensors')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylim((-5,105))
    pp.xlim((10,170))
    pp.title('100\% movable, 0\% fixed')
    mpu.legend(draw_frame=False) #display_mode='less_space', 
    pp.ylabel('Success percent', labelpad=-2)

    return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def fixed_25_percent():
    path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    #path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'combined_results.pkl'

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = return_lists_25_percent_fixed()

    obj_count_list = [20, 40, 80, 120, 160]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, openrave_nm_list)

    pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, skin_success_percent, 'b-o', mew=0,
            label='MPC with tactile sensing')
    pp.plot(obj_count_list, ft_success_percent, 'g-o', mew=0, label='MPC with FT sensors')
    pp.plot(obj_count_list, baseline_success_percent, 'y-o', mew=0,
            label='MPC without tactile\n or FT sensors')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylim((0,100))
    pp.xlim((15,165))
    pp.title('75\% movable, 25\% fixed')

    return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def fixed_50_percent():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    #path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'combined_results.pkl'
    
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = return_lists_50_percent_each()


    #obj_count_list = [20, 40, 60, 80]
    obj_count_list = [20, 40, 80, 120, 160]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, openrave_nm_list)

    #mpu.set_figure_size(8, 8.0)
    #pp.figure()

    pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, skin_success_percent, 'b-o', mew=0, label='MPC with tactile sensing')
    pp.plot(obj_count_list, ft_success_percent, 'g-o', mew=0, label='MPC with FT sensors')
    pp.plot(obj_count_list, baseline_success_percent, 'y-o', mew=0,
            label='MPC without tactile\n\t\t or FT sensors')

    #pp.ylabel('success percent')
    #pp.xlabel('\# of fixed and movable cylinders')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylim((-5,105))
    pp.xlim((10,170))
    #mpu.legend(display_mode='less_space', draw_frame=False)
    #mpu.reduce_figure_margins(left=0.15, bottom=0.18, right=0.97, top=0.85)
    pp.title('50\% movable, 50\% fixed')
    #pp.savefig('vary_both_success_rate.pdf')

    return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def fixed_75_percent():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    pkl_nm = 'combined_results.pkl'

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                    openrave_nm_list, _ = return_lists_75_percent_fixed()
    print ft_sensor_nm_list, '\n\n'
    print skin_nm_list, '\n\n'
    print baseline_nm_list, '\n\n'


    obj_count_list = [20, 40, 80, 120, 160]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, openrave_nm_list)

    pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, skin_success_percent, 'b-o', mew=0, label='MPC with tactile sensing')
    pp.plot(obj_count_list, ft_success_percent, 'g-o', mew=0, label='MPC with FT sensors')
    pp.plot(obj_count_list, baseline_success_percent, 'y-o', mew=0,
            label='MPC without tactile\n or FT sensors')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylabel('Success percent', labelpad=-2)
    pp.ylim((0,100))
    pp.xlim((15,165))
    pp.title('25\% movable, 75\% fixed')
    mpu.legend(display_mode='less_space', draw_frame=False)

    #ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_75_percent_fixed(True)
    #obj_count_list = [40, 80, 100, 120, 140, 160]
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_75_percent_fixed(False)
    obj_count_list = [20, 40, 80, 120, 160]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, openrave_nm_list)

    return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def fixed_100_percent():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    pkl_nm = 'combined_results.pkl'

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = return_lists_100_percent_fixed()

    obj_count_list = [20, 40, 80, 120, 160]
    #obj_count_list = [40, 120, 160]
    ft_success_percent, skin_success_percent, baseline_success_percent,\
    openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
                                                                     ft_sensor_nm_list, skin_nm_list,
                                                                     baseline_nm_list, openrave_nm_list)

    pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, skin_success_percent, 'b-o', mew=0,
            label='MPC with tactile sensing')
    pp.plot(obj_count_list, ft_success_percent, 'g-o', mew=0, label='MPC with FT sensors')
    pp.plot(obj_count_list, baseline_success_percent, 'y-o', mew=0,
            label='MPC without tactile\n or FT sensors')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylim((-5,105))
    pp.xlim((10,170))
    pp.title('0\% movable, 100\% fixed')
    #pp.ylabel('Success percent', labelpad=-2)

    return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def percentile_force_statistics(pth, pkl_nm, l):
    avg_l = []
    percentile_95_l = []
    for nm in l:
        d = ut.load_pickle(os.path.join(pth,nm,pkl_nm))
        avg_l.append(np.mean(d['percentile_95_force_list']))
        percentile_95_l.append(ss.scoreatpercentile(d['percentile_95_force_list'], 95))
    return avg_l, percentile_95_l

def max_force_statistics(pth, pkl_nm, l):
    avg_l = []
    max_l = []
    percentile_l = []
    for nm in l:
        d = ut.load_pickle(os.path.join(pth,nm,pkl_nm))
        avg_l.append(d['avg_contact_force'])
        max_l.append(d['avg_max_force'])
        max_force_list = d['max_force_list']
        percentile_l.append(ss.scoreatpercentile(max_force_list, 95))
    return avg_l, max_l, percentile_l

def print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list):
    avg_ft_l, max_ft_l, tq_ft_l = max_force_statistics(path, pkl_nm, ft_sensor_nm_list)
    avg_skin_l, max_skin_l, tq_skin_l = max_force_statistics(path, pkl_nm, skin_nm_list)
    avg_baseline_l, max_baseline_l, tq_baseline_l = max_force_statistics(path, pkl_nm, baseline_nm_list)
    print 'Avg. Max Force:'
    print 'Skin -', max_skin_l
    print 'FT -', max_ft_l
    print 'Baseline -', max_baseline_l
    print
    print '95 percentile max force:'
    print 'Skin -', tq_skin_l
    print 'FT -', tq_ft_l
    print 'Baseline -', tq_baseline_l
    print

    avg_ft_l, perc_ft_l = percentile_force_statistics(path, pkl_nm, ft_sensor_nm_list)
    avg_skin_l, perc_skin_l = percentile_force_statistics(path, pkl_nm, skin_nm_list)
    avg_baseline_l, perc_baseline_l = percentile_force_statistics(path, pkl_nm, baseline_nm_list)
    print 'Avg. 95 percentile force:'
    print 'Skin -', avg_skin_l
    print 'FT -', avg_ft_l
    print 'Baseline -', avg_baseline_l
    print
    print '95 percentile of 95 percentile force:'
    print 'Skin -', perc_skin_l
    print 'FT -', perc_ft_l
    print 'Baseline -', perc_baseline_l

def compute_force_statistics():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    # path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'reach_in_force_statistics.pkl'

    obj_count_list = [20, 40, 80, 120, 160]
    print 'Total Obstacles:', obj_count_list
    print

    print '========================================================='
    print '50% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_50_percent_each()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

    print '========================================================='
    print '75% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_75_percent_fixed()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

    print '========================================================='
    print '25% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_25_percent_fixed()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

def compute_n_reaches_for_success(path, pkl_nm, nm_list):
    l = []
    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        l.extend(d['n_reaches_for_success_list'])

    arr = np.array(l)
    min_val = np.min(arr)
    max_val = np.max(arr)
    res_dict = {}
    for i in range(min_val, max_val+1):
        res_dict[i] = np.where(arr == i)[0].shape[0]
    return res_dict

def compute_average_velocity(path, pkl_nm, nm_list):
    dist_l = []
    time_l = []
    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        dist_l.extend(d['ee_distance_list'])
        time_l.extend(d['execution_time_list'])

    total_dist = np.sum(dist_l)
    total_time = np.sum(time_l)
    return total_dist / total_time

def trials_above_force_value(max_force_list, starting_force, ending_force):
    l = copy.copy(max_force_list)
    l.sort()
    n = len(l)
    y = np.array(range(n))
    max_force_arr = np.array(l)
    start_idx_buf = np.where(max_force_arr > starting_force)
    start_idx = start_idx_buf[0][0]
    return max_force_arr[start_idx:], (n - y[start_idx:])/ (n*1.)

def plot_perc_contact_forces_above_force_threshold(perc_ft, perc_skin, x):
    mpu.set_figure_size(8,5)
    pp.figure()
    mpu.reduce_figure_margins(left=0.2, bottom=0.2, top=0.85)
    #mpu.reduce_figure_margins(left=0.14, bottom=0.2, top=0.85)

    pp.plot(x, perc_ft, color='g',
            label='MPC with FT sensors')
    pp.plot(x, perc_skin, color='b',
            label='MPC with tactile sensing')
    pp.xlim(5.5, 20.5)
    pp.ylim(0, max(np.max(perc_ft), np.max(perc_skin))+1)
    #pp.axhline(0.05, c='k', ls=':')
    #pp.xlabel('95 percentile force magnitude (N)')
    pp.xlabel('force magnitude (N)')
    pp.ylabel('contact forces above \n force magnitude (\%)')
    pp.ylim((0,8.0))

    mpu.legend(display_mode='less_space', draw_frame=False)



def plot_number_above_force_threshold(max_force_ft, max_force_skin):
    mpu.set_figure_size(8,5)
    pp.figure()
    mpu.reduce_figure_margins(left=0.17, bottom=0.2, top=0.85)

    #starting_force = 5.
    starting_force = 5.
    ending_force = 35.
    ft_force_arr, ft_count_arr = trials_above_force_value(max_force_ft,
                                                          starting_force,
                                                          ending_force)
    skin_force_arr, skin_count_arr = trials_above_force_value(max_force_skin,
                                                              starting_force,
                                                              ending_force)

    pp.plot(ft_force_arr, ft_count_arr, color='g',
            label='MPC with FT sensors')
    pp.plot(skin_force_arr, skin_count_arr, color='b',
            label='MPC with tactile sensing')
    pp.xlim(starting_force, ending_force)
    pp.ylim(0, max(ft_count_arr.max(), skin_count_arr.max()))
    #pp.axhline(0.05, c='k', ls=':')
    #pp.xlabel('95 percentile force magnitude (N)')
    pp.xlabel('95th percentile force magnitude (N)')
    pp.ylabel('Fraction of trials')
    pp.ylim((0,0.2))

    mpu.legend(display_mode='less_space', draw_frame=False)

def plot_force_histogram(f_mag_l):
    max_force = max(f_mag_l)
    bin_width = 0.5
    bins = np.arange(0.-bin_width/2., max_force+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(np.array(f_mag_l), bins)
    mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=bin_width*0.8)
    pp.xlabel('max contact force (N)')
    #pp.xlabel('95 percentile contact force (N)')
    pp.ylabel('bin count')


def statistical_analysis_success_percent(skin_succ_perc, ft_succ_perc, n1, n2=0):
    success_perc_diff_l = []
    conf_95_interval_l = []
    p_value_l = []
    for p1hat_perc, p2hat_perc in zip(skin_succ_perc, ft_succ_perc):
        p1hat, p2hat = p1hat_perc/100., p2hat_perc/100.
        if n2 == 0:
            phat = (p1hat + p2hat) / 2 # each case has the same number of samples/trials
            se = math.sqrt(p1hat*(1-p1hat)/n1 + p2hat*(1-p2hat)/n1)
        else:
            phat = (p1hat*n1 + p2hat*n2) / (n1+n2) # if each case has a different # of samples
            se = math.sqrt(p1hat*(1-p1hat)/n1 + p2hat*(1-p2hat)/n2)            

            

        success_perc_diff_l.append(p2hat_perc - p1hat_perc)
        conf_95_interval_l.append(1.96 * se * 100.)

        se_h0 = math.sqrt(phat*(1-phat)*2./(n1+n2))
        z = (p1hat - p2hat) / se_h0
        # print "phat is :\t", phat
        # print "se_h0 is :\t", se_h0

        print "z is :\t", z
        if z < 0.:
            z = -z

        p_value_l.append(2 * ss.norm.sf(z))

    return success_perc_diff_l, conf_95_interval_l, p_value_l


def get_percentages_above_thresh(direc, nm, x_values):
    skin_total = 0
    skin_above = [0]*len(x_values)

    ft_total = 0
    ft_above = [0]*len(x_values)

    for name in nm:
        data_ft = ut.load_pickle(direc+name+'/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        data_skin = ut.load_pickle(direc+name+'/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')

        if data_ft == None or data_skin == None:
            print "NO DATA AT :", direc+name
            sys.exit()
  
        skin_total = skin_total + len(data_skin['all_forces_mag_list'])
        ft_total = ft_total + len(data_ft['all_forces_mag_list'])

        for i in xrange(len(x_values)):
            skin_above[i] = skin_above[i] + len(np.where(np.array(data_skin['all_forces_mag_list']) > x_values[i])[0])
            ft_above[i] = ft_above[i] + len(np.where(np.array(data_ft['all_forces_mag_list']) > x_values[i])[0])
            
    ft_result = []
    skin_result = []
    for i in xrange(len(x_values)):
        skin_result.append(float(skin_above[i])/float(skin_total)*100)
        ft_result.append(float(ft_above[i])/float(ft_total)*100)

    return skin_result, ft_result, skin_total, ft_total


if __name__ == '__main__':
    success_rate_plots = False #True
    force_statistics = False 
    average_velocity = False
    n_reaches_for_success = False #True #True
    plot_percent_above_threshold = True
    high_force_count_plot_for_50_perc_fixed = False 
    high_force_count_plot_for_100_perc_fixed = False 
    high_force_count_plot_for_0_perc_fixed = False
    multiple_reach_results = False #True
    regulate_forces_plot = False

    if regulate_forces_plot:
        #path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        path = '/home/advait/nfs/darpa_m3_logs/advait/25_obstacle_configurations_may_26_movable20_fixed20'
        pkl_nm = 'reach_in_force_statistics.pkl'
        trial_nm_l = ['mpc_qs_1_20120526_using_skin_5N',
                      'mpc_qs_1_20120526_using_skin_10N',
                      'mpc_qs_1_20120526_using_skin_15N',
                      'mpc_qs_1_20120526_using_skin_20N',
                      'mpc_qs_1_20120526_using_skin_25N']
        
        #for trial_nm in trial_nm_l:
        #    d = ut.load_pickle(os.path.join(path, trial_nm, pkl_nm))
        #    all_forces = d['all_forces_mag_list']
        #    print '-----------------------'
        #    print 'trial_nm:', trial_nm
        #    print '-----------------------'
        #    #print 'Median force:', d['median_contact_force']
        #    #print 'First quartile contact force:', d['first_quartile_contact_force']
        #    #print 'Third quartile contact force:', d['third_quartile_contact_force']
        #    print '75 percentile force:', ss.scoreatpercentile(all_forces, 75)
        #    print '90 percentile force:', ss.scoreatpercentile(all_forces, 90)
        #    print '95 percentile force:', ss.scoreatpercentile(all_forces, 95)
        #    print '99 percentile force:', ss.scoreatpercentile(all_forces, 99)

        perc_75_a = np.array([4.65661773347, 7.46382457071, 8.36974924057, 8.81591794446, 9.31266569258])
        perc_90_a = np.array([4.92841734977, 9.85172087923, 14.2601314264, 14.8729333516, 16.5725414726])
        perc_95_a = np.array([4.97980083, 9.94200627435, 14.8729220772, 19.5636655556, 23.9172521823])
        perc_99_a = np.array([5.54570756397, 10.0423941129, 15.0507787333, 20.0175772605, 25.0097461856])

        f_thresh_l = [5, 10, 15, 20, 25]
        
        print 'correlation:', ss.pearsonr(perc_95_a, f_thresh_l)

        mpu.set_figure_size(8,8)
        pp.figure()
        yerr = np.row_stack([perc_95_a-perc_75_a, perc_99_a-perc_95_a])
        pp.errorbar(f_thresh_l, perc_95_a, yerr, linewidth=0,
                    marker='o', ms=5, elinewidth=1, capsize=10)
        pp.plot([0,30], [0,30], 'r-')
        pp.axis('equal')
        #pp.xlim(0,30)
        pp.ylim(0,30)
        pp.ylabel('Contact force magnitude (N)')
        pp.xlabel('Don\'t care force threshold (N)')
        mpu.reduce_figure_margins(left=0.15, bottom=0.15, right=0.98, top=0.98)
        pp.show()

    if plot_percent_above_threshold:
        print "got in"

        nm_0 = ['75_obstacle_configurations_movable020_fixed000',
                '75_obstacle_configurations_movable040_fixed000',
                '75_obstacle_configurations_movable080_fixed000',
                '75_obstacle_configurations_movable120_fixed000',
                '75_obstacle_configurations_movable160_fixed000']


        nm_50 = ['75_obstacle_configurations_movable010_fixed010',
                 '75_obstacle_configurations_movable020_fixed020',
                 '75_obstacle_configurations_movable040_fixed040',
                 '75_obstacle_configurations_movable060_fixed060',
                 '75_obstacle_configurations_movable080_fixed080']
                 

        nm_100 = ['75_obstacle_configurations_movable000_fixed020',
                  '75_obstacle_configurations_movable000_fixed040',
                  '75_obstacle_configurations_movable000_fixed080',
                  '75_obstacle_configurations_movable000_fixed120',
                  '75_obstacle_configurations_movable000_fixed160']
        
        x_values = [6+i*0.5 for i in xrange(29)]

        direc = '/home/mkillpack/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'

        print "getting values from pkl files, will take a while ..."


        perc_skin_0, perc_ft_0, skin_total_0, ft_total_0 = get_percentages_above_thresh(direc, nm_0, x_values)

        print "perc_skin_0 :\n", perc_skin_0
        print "perc_ft_0 :\n", perc_ft_0

        print "p-values for 0:\n", statistical_analysis_success_percent(perc_skin_0, perc_ft_0, skin_total_0, ft_total_0)[2], "\n"

        print "95 perc conf intervals for 0: \n", statistical_analysis_success_percent(perc_skin_0, perc_ft_0, skin_total_0, ft_total_0)[1], "\n"

        plot_perc_contact_forces_above_force_threshold(perc_ft_0, perc_skin_0, x_values)
        pp.title('0\% fixed')
        pp.ylim((0, 20.))
        pp.savefig('contact_force_percent_above_threshold_0_perc_fixed.pdf')

        perc_skin_50, perc_ft_50, skin_total_50, ft_total_50 = get_percentages_above_thresh(direc, nm_50, x_values)

        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%  skin results %%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # percent above 5 N : 8.64199864301
        # percent above 6 N : 1.44248696781
        # percent above 10 N : 0.438476688036
        # percent above 15 N : 0.157913092349


        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%  ft  results %%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # percent above 5 N : 9.73502144213
        # percent above 6 N : 3.74509528207
        # percent above 10 N : 1.45356346702
        # percent above 15 N : 0.662257671716

        print "perc_skin_50 :\n", perc_skin_50
        print "perc_ft_50 :\n", perc_ft_50

        plot_perc_contact_forces_above_force_threshold(perc_ft_50, perc_skin_50, x_values)
        pp.title('50\% fixed')
        pp.ylim((0, 20.))
        pp.savefig('contact_force_percent_above_threshold_50_perc_fixed.pdf')


        perc_skin_100, perc_ft_100, skin_total_100, ft_total_100 = get_percentages_above_thresh(direc, nm_100, x_values)

        print "perc_skin_100 :\n", perc_skin_100
        print "perc_ft_100 :\n", perc_ft_100

        print "perc_100 skin :", 1./(np.array(perc_skin_100)/100.), "\n"
        print "perc_100 ft :", 1./(np.array(perc_ft_100)/100.)

        # 100 fixed stuff
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%  skin results %%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # percent above 5 N : 20.2294871406
        # percent above 6 N : 3.17798267048
        # percent above 10 N : 0.968019198327
        # percent above 15 N : 0.32822637565


        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%  ft  results %%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # percent above 5 N : 20.0479481783
        # percent above 6 N : 7.37324111795
        # percent above 10 N : 3.303457195
        # percent above 15 N : 1.68182416629


        plot_perc_contact_forces_above_force_threshold(perc_ft_100, perc_skin_100, x_values)
        pp.title('100\% fixed')
        pp.ylim((0, 20.))
        pp.savefig('contact_force_percent_above_threshold_100_perc_fixed.pdf')

        pp.show()

        print "p-values for 50:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        print "p-values for 100:\n", statistical_analysis_success_percent(perc_skin_100, perc_ft_100, skin_total_100, ft_total_100)[2], "\n"        

        print "95 perc conf intervals for 50: \n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[1], "\n"
        print "95 perc conf intervals for 100:\n", statistical_analysis_success_percent(perc_skin_100, perc_ft_100, skin_total_100, ft_total_100)[1], "\n"        

    if high_force_count_plot_for_50_perc_fixed:
        print "FOR 50 PERCENT FIXED"
        path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        plot_number_above_force_threshold(max_force_ft, max_force_skin)
        pp.title('160 cylinders. 50\% fixed')
        #pp.ylim((0, 0.6))
        pp.savefig('trials_above_force_160_for_50_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_ft)
        pp.savefig('ft_histogram_160_50_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_skin)
        pp.savefig('skin_histogram_160_50_perc_fixed.pdf')

############################
#         d = ut.load_pickle('120_ft.pkl')
#         max_force_ft = d['max_force_list']
#         #max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle('120_skin.pkl')
#         max_force_skin = d['max_force_list']
#         #max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         print '120 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '120 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_120.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_120.pdf')

# ############################

        d = ut.load_pickle(path+'75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        # d = ut.load_pickle('80_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('80_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        plot_number_above_force_threshold(max_force_ft, max_force_skin)
        pp.title('80 cylinders. 50\% fixed')
        #pp.ylim((0, 0.2))
        pp.savefig('trials_above_force_80_for_50_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_ft)
        pp.savefig('ft_histogram_80_50_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_skin)
        pp.savefig('skin_histogram_80_50_perc_fixed.pdf')

# ###########################
#         d = ut.load_pickle('40_ft.pkl')
#         max_force_ft = d['max_force_list']
#         #max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle('40_skin.pkl')
#         max_force_skin = d['max_force_list']
#         #max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         print '40 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '40 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
# ############################

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_40.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_40.pdf')

        pp.show()

    if high_force_count_plot_for_0_perc_fixed:
        print "FOR 0 PERCENT FIXED"
        path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        d = ut.load_pickle(path+'75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        try:
            plot_number_above_force_threshold(max_force_ft, max_force_skin)
            pp.title('160 cylinders. 0\% fixed')
            #pp.ylim((0, 0.6))
            pp.savefig('trials_above_force_160_for_0_perc_fixed.pdf')

            pp.figure()
            plot_force_histogram(max_force_ft)
            pp.savefig('ft_histogram_160_0_perc_fixed.pdf')

            pp.figure()
            plot_force_histogram(max_force_skin)
            pp.savefig('skin_histogram_160_0_perc_fixed.pdf')
        except:
            print "No trials with forces above threshold"

        d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        # d = ut.load_pickle('80_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('80_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        try:
            plot_number_above_force_threshold(max_force_ft, max_force_skin)
            pp.title('80 cylinders. 0\% fixed')
            #pp.ylim((0, 0.2))
            pp.savefig('trials_above_force_80_for_0_perc_fixed.pdf')

            pp.figure()
            plot_force_histogram(max_force_ft)
            pp.savefig('ft_histogram_80_0_perc_fixed.pdf')

            pp.figure()
            plot_force_histogram(max_force_skin)
            pp.savefig('skin_histogram_80_0_perc_fixed.pdf')
        except:
            print "No trials with forces above threshold"



        pp.show()

    if high_force_count_plot_for_100_perc_fixed:
        print "FOR 100 PERCENT FIXED"
        path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('160_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        plot_number_above_force_threshold(max_force_ft, max_force_skin)
        pp.title('160 cylinders. 100\% fixed')
        #pp.ylim((0, 0.6))
        pp.savefig('trials_above_force_160_for_100_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_ft)
        pp.savefig('ft_histogram_160_100_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_skin)
        pp.savefig('skin_histogram_160_100_perc_fixed.pdf')

        d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        # d = ut.load_pickle('80_ft.pkl')
        #max_force_ft = d['max_force_list']
        max_force_ft = d['percentile_95_force_list']

        d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #d = ut.load_pickle('80_skin.pkl')
        #max_force_skin = d['max_force_list']
        max_force_skin = d['percentile_95_force_list']

        print '----------------------------------------------'
        #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
        #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
        print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
        print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
        #print 'Max U:', len(max_force_skin) * len(max_force_ft)

        #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
        #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

        plot_number_above_force_threshold(max_force_ft, max_force_skin)
        pp.title('80 cylinders. 100\% fixed')
        #pp.ylim((0, 0.2))
        pp.savefig('trials_above_force_80_for_100_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_ft)
        pp.savefig('ft_histogram_80_100_perc_fixed.pdf')

        pp.figure()
        plot_force_histogram(max_force_skin)
        pp.savefig('skin_histogram_80_100_perc_fixed.pdf')

        pp.show()


    if force_statistics:
        compute_force_statistics()

    if average_velocity:
        path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        #path = '/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
        pkl_nm = 'reach_in_kinematics_statistics.pkl'
        ft1_l, skin1_l, baseline1_l, _, _ = return_lists_50_percent_each()
        ft2_l, skin2_l, baseline2_l, _, _ = return_lists_100_percent_fixed()
        ft3_l, skin3_l, baseline3_l, _, _ = return_lists_0_percent_fixed()
        obj_count_list = [20, 40, 80, 120, 160]
        print 'Average velocity with tactile sensing:', compute_average_velocity(path, pkl_nm, skin1_l+skin2_l+skin3_l)
        print 'Average velocity with FT sensor:', compute_average_velocity(path, pkl_nm, ft1_l+ft2_l+ft3_l)
        print 'Average velocity with neither:', compute_average_velocity(path, pkl_nm, baseline1_l+baseline2_l+baseline3_l)

    if n_reaches_for_success:
        #estimated_optimal_success = (0.881 + 0.623 + 0.392 +0.212 + 0.927 + 0.800 +0.612 + 0.420 +0.978 +0.935 +0.885 +0.797) * 600 #before low clutter trials
        #estimated_optimal_success = (0.881 + 0.623 + 0.392 +0.212 + 0.927 + 0.800 +0.612 + 0.420 +0.978 +0.935 +0.885 +0.797+ 0.940 + 0.977 + 0.972) * 600  #this is with 16 samples
        #d = {1: 3350, 2: 815, 3: 315, 4: 96, 5: 48, -1: 2576}
        #d = {0: 0, 1: 3350, 2: 815, 3: 315, 4: 96, 5: 48, -1: 2576}  #before the low clutter trials
        #d = {0: 0, 1: 4904, 2: 1107, 3: 378, 4: 116, 5: 53, -1: 2649}

        # for 50/50 now only:
        # all_d = []
        # all_d.append({0: 0, 1: 525, 2: 46, 3: 5, 4: 1, 5: 0, -1: 23})
        # all_d.append({0: 0, 1: 439, 2: 73, 3: 14, 4: 4, 5: 4, -1: 66})        
        # all_d.append({0: 0, 1: 296, 2: 81, 3: 40, 4: 10, 5: 5, -1: 168})
        # all_d.append({0: 0, 1: 210, 2: 78, 3: 27, 4: 9, 5: 5, -1: 271})
        # all_d.append({0: 0, 1: 119, 2: 59, 3: 26, 4: 11, 5: 5, -1: 380})

        # est_opt = [591, 578, 503, 338, 252]

        # mpu.set_figure_size(8, 5)
        # fig = pp.figure()

        # ind = 0
        # num_ls = [20, 40, 80, 120, 160]
        # for d in all_d:
        #     n_reach_list = []
        #     count_l = []
        #     for k in d.keys():
        #         if k == -1 or k==0:
        #             continue
        #         n_reach_list.append(k)
        #         count_l.append(d[k])

        #     total_success = np.sum(count_l)
        #     frac_optimal = np.cumsum(count_l)*1./est_opt[ind]  #estimated_optimal_success
        #     #frac_optimal = np.cumsum(count_l)  #estimated_optimal_success
        #     #pp.plot(n_reach_list, frac_optimal,mew=0, label=str(num_ls[ind]))
        #     pp.plot(n_reach_list, frac_optimal, '-o', mew=0, label=str(num_ls[ind]))
        #     pp.hold(True)
        #     ind = ind + 1

        # estimated_optimal_success = (591+578+503+338+252)
        # d={0: 0, 1: 525+439+296+210+119, 2: 46+73+81+78+59, 3: 5+14+40+27+26, 4: 1+4+10+9+11, 5: 0+4+5+5+5, -1: 23+66+168+271+380}

        estimated_optimal_success = (1183+1133+983+775+555)
        d={0: 0, 1: 1054+904+624+442+260, 2: 86+131+160+175+118, 3: 16+35+76+50+48, 4: 3+13+28+27+21, 5: 1+10+10+6+6, -1: 40+104+302+500+747}

        mpu.set_figure_size(8, 5)
        fig = pp.figure()



        n_reach_list = []
        count_l = []
        for k in d.keys():
            if k == -1 or k==0:
                continue
            n_reach_list.append(k)
            count_l.append(d[k])

        total_success = np.sum(count_l)
        frac_optimal = np.cumsum(count_l)*1./estimated_optimal_success  #estimated_optimal_success
        
        print "frac_optimal is :", frac_optimal

        pp.plot(n_reach_list, frac_optimal, '-o', mew=0)
        # #pp.ylim((0,100.))
        # pp.legend()
        pp.ylim((0.65, 1.))
        pp.xlim((0,6))
        pp.xlabel('Number of greedy MPC reaches')
        pp.ylabel('Fraction of \nestimated optimal', labelpad=14,
                horizontalalignment='center')
        mpu.reduce_figure_margins(left=0.22, bottom=0.20, right=0.98, top=0.97)
        pp.savefig('success_with_retries_50_50_only.pdf')
        pp.show()



        # frac_optimal = np.cumsum(count_l)*100./estimated_optimal_success
        # print 'frac_optimal', frac_optimal
        # fig = pp.figure()
        # pp.plot(n_reach_list, frac_optimal, 'b-o', mew=0)
        # pp.ylim((0,100.))
        # pp.xlim((0,6))
        # pp.xlabel('Number of reaches with MPC')
        # pp.ylabel('Percent of \nestimated optimal', labelpad=14,
        #         horizontalalignment='center')
        # mpu.reduce_figure_margins(left=0.20, bottom=0.20, right=0.98, top=0.97)
        # pp.savefig('success_with_retries.pdf')
        # pp.show()

        #        path = '/home/advait/nfs/darpa_m3_logs/advait'
        #        pkl_nm = 'count_reaches_for_success.pkl'
        #
        #        print '------------------------------'
        #        print '25% movable, 75% fixed'
        #        print '------------------------------'
        #        _, _, _, _, mul1 = return_lists_75_percent_fixed()
        #        obj_count_list = [40, 80, 120, 160]
        #
        #        d = compute_n_reaches_for_success(path, pkl_nm, mul1)
        #        print d
        #
        #        print '------------------------------'
        #        print '50% movable, 50% fixed'
        #        print '------------------------------'
        #        _, _, _, _, mul2 = return_lists_50_percent_each()
        #        obj_count_list = [40, 80, 120, 160]
        #
        #        d = compute_n_reaches_for_success(path, pkl_nm, mul2)
        #        print d
        #
        #        print '------------------------------'
        #        print '75% movable, 25% fixed'
        #        print '------------------------------'
        #        _, _, _, _, mul3 = return_lists_25_percent_fixed()
        #        obj_count_list = [40, 80, 120, 160]
        #
        #        d = compute_n_reaches_for_success(path, pkl_nm, mul3)
        #        print d
        #
        #        print '-----------------------------'
        #        print 'Overall'
        #        print '-----------------------------'
        #        d = compute_n_reaches_for_success(path, pkl_nm, mul1+mul2+mul3)
        #        print d

    if multiple_reach_results:
        path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        #path = base_path+'/hrl_file_server/darpa_m3/advait_ijrr_revision_1_all_data'
        #path = '/home/advait/nfs/darpa_m3_logs/advait'

        pkl_nm = 'combined_results.pkl'

        print '------------------------------'
        print '25% movable, 75% fixed'
        print '------------------------------'
        _, skin_nm_list, _, openrave_nm_list, multiple_reach_nm_list = return_lists_75_percent_fixed()
        obj_count_list = [20, 40, 80, 120, 160]

        single_success = compute_success_rate(path, pkl_nm, skin_nm_list)
        multi_success = compute_success_rate(path, pkl_nm, multiple_reach_nm_list)
        or_success = compute_success_rate(path, pkl_nm, openrave_nm_list)

        print 'total number of objects:'
        print obj_count_list
        print
        print 'single reach success percent:'
        print single_success
        print
        print 'multiple reaches success percent'
        print multi_success
        print
        print 'estimated max success percent'
        print or_success

        print '------------------------------'
        print '50% movable, 50% fixed'
        print '------------------------------'
        _, skin_nm_list, _, openrave_nm_list, multiple_reach_nm_list = return_lists_50_percent_each()
        obj_count_list = [20, 40, 80, 120, 160]

        single_success = compute_success_rate(path, pkl_nm, skin_nm_list)
        multi_success = compute_success_rate(path, pkl_nm, multiple_reach_nm_list)
        or_success = compute_success_rate(path, pkl_nm, openrave_nm_list)

        print 'total number of objects:'
        print obj_count_list
        print
        print 'single reach success percent:'
        print single_success
        print
        print 'multiple reaches success percent'
        print multi_success
        print
        print 'estimated max success percent'
        print or_success

        print '------------------------------'
        print '75% movable, 25% fixed'
        print '------------------------------'
        _, skin_nm_list, _, openrave_nm_list, multiple_reach_nm_list = return_lists_25_percent_fixed()
        obj_count_list = [20, 40, 80, 120, 160]

        single_success = compute_success_rate(path, pkl_nm, skin_nm_list)
        multi_success = compute_success_rate(path, pkl_nm, multiple_reach_nm_list)
        or_success = compute_success_rate(path, pkl_nm, openrave_nm_list)

        print 'total number of objects:'
        print obj_count_list
        print
        print 'single reach success percent:'
        print single_success
        print
        print 'multiple reaches success percent'
        print multi_success
        print
        print 'estimated max success percent'
        print or_success

    if success_rate_plots:
        #mpu.set_figure_size(8*3, 6.5*2)
        mpu.set_figure_size(8*3, 8.0)
        fig = pp.figure()
        fig.subplots_adjust(hspace=0.350, wspace=0.0001, bottom=0.17,
                            top=0.84, left=0.045, right=0.99)

        sp1 = pp.subplot(1,3,1)
        obj_l_all_move, pi_all_move, ft_0, skin_0 = fixed_0_percent()

        print "ft_0 :\n", ft_0
        print "skin_0 :\n", skin_0
        print "p-values :\n", statistical_analysis_success_percent(skin_0, ft_0, 1200)[2]

        sp2 = pp.subplot(1,3,2)
        obj_l_both, pi_both, ft_50, skin_50 = fixed_50_percent()

        print "ft_50 :\n", ft_50
        print "skin_50 :\n", skin_50
        print "p-values :\n", statistical_analysis_success_percent(skin_50, ft_50, 1200)[2]

        sp3 = pp.subplot(1,3,3)
        obj_l_all_fixed, pi_all_fixed, ft_100, skin_100 = fixed_100_percent()

        print "ft_100 :\n", ft_100
        print "skin_100 :\n", skin_100
        print "p-values :\n", statistical_analysis_success_percent(skin_100, ft_100, 1200)[2]

        # sp1 = pp.subplot(2,3,1)
        # obj_l_fix, pi_fix, ft_75, skin_75 = fixed_75_percent()

        # sp2 = pp.subplot(2,3,2)
        # obj_l_both, pi_both, ft_50, skin_50 = fixed_50_percent()

        # sp3 = pp.subplot(2,3,3)
        # obj_l_move, pi_move, ft_25, skin_25 = fixed_25_percent()

        # ####commment these lines##############
        # sp4 = pp.subplot(2,3,4)
        # obj_l_all_fixed, pi_all_fixed, ft_100, skin_100 = fixed_100_percent()

        # sp6 = pp.subplot(2,3,6)
        # obj_l_all_move, pi_all_move, ft_0, skin_0 = fixed_0_percent()
        # ####commment these lines##############


        pp.setp(sp2.get_yticklabels(), visible=False)
        pp.setp(sp3.get_yticklabels(), visible=False)
        pp.savefig('success_rate_with_clutter.pdf')

        if True:
            mpu.set_figure_size(7.4, 7.0)
            pp.figure()

            
            ft_arr = np.sum(np.row_stack([ft_0, ft_50, ft_100]), 0)
            skin_arr = np.sum(np.row_stack([skin_0, skin_50, skin_100]), 0)

            # ft_arr = np.sum(np.row_stack([ft_25, ft_50, ft_75]), 0)
            # skin_arr = np.sum(np.row_stack([skin_25, skin_50, skin_75]), 0)


            obj_l = [20, 40, 80, 120, 160]
            pi = (skin_arr - ft_arr) / ft_arr * 100.
            pp.plot(obj_l, pi, 'c-o', mew=0)
            pp.ylabel('percent improvement \n in success rate',
                      horizontalalignment='center', labelpad=14)
            pp.xlabel('Total number of cylinders', labelpad=2)
            pp.xlim((15,165))
            pp.ylim((0,35))
            mpu.reduce_figure_margins(left=0.205, bottom=0.13, right=0.97, top=0.90)
            pp.title('Tactile sensing vs FT sensors')
            pp.savefig('percent_improvement_combined.pdf')

            pp.figure()
            #'legend.linewidth': 2}
            # pp.plot(obj_l_all_fixed, pi_all_fixed, 'g-o', mew=0, label='0\% movable, 100\% fixed')
            # pp.plot(obj_l_fix, pi_fix, 'c-o', mew=0, label='vary \# fixed')
            # pp.plot(obj_l_fix, pi_fix, 'c-o', mew=0, label='25\% movable, 75\% fixed')
            # pp.plot(obj_l_both, pi_both, 'm-o', mew=0, label='50\% movable, 50\% fixed')
            # pp.plot(obj_l_move, pi_move, 'k-o', mew=0, label='vary \# movable')
            # pp.plot(obj_l_move, pi_move, 'k-o', mew=0, label='75\% movable, 25\% fixed')
            # pp.plot(obj_l_all_move, pi_all_move, 'y-o', mew=0, label='100\% movable, 0\% fixed')
            

            pp.plot(obj_l_all_fixed, pi_all_fixed, 'c-o', mew=0)#, label='0\% movable, 100\% fixed')
            pp.plot(np.NaN, np.NaN, 'c-o', mew=0, ms=4., label='100\% fixed')
            print "percent improvement over ft sensors for 100\% fixed", pi_all_fixed

            pp.plot(obj_l_both, pi_both, 'm-o', mew=0)#, label='50\% movable, 50\% fixed')
            pp.plot(np.NaN, np.NaN, 'm-o', mew=0, ms=4., label='50\% fixed')
            print "percent improvement over ft sensors for 50\% fixed", pi_both

            pp.plot(obj_l_all_move, pi_all_move, 'k-o', mew=0)#, label='100\% movable, 0\% fixed')
            pp.plot(np.NaN, np.NaN, 'k-o', mew=0, ms=4., label='0\% fixed')
            print "percent improvement over ft sensors for 0\% fixed", pi_all_move

            pp.ylabel('percent improvement \n in success rate',
                      horizontalalignment='center', labelpad=14)
            print "all fixed improvement", pi_all_fixed
            pp.xlabel('Total number of cylinders', labelpad=2)
            pp.xlim((10,170))
            pp.ylim((-5,100))
            pp.yticks([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100])
            mpu.legend(display_mode='less_space', draw_frame=False)
            mpu.reduce_figure_margins(left=0.205, bottom=0.13, right=0.97, top=0.90)
            pp.title('Tactile sensing vs FT sensors')
            # params = {'legend.markerscale': 0.01}
            # pp.rcParams.update(params)
            pp.savefig('percent_improvement.pdf')

        pp.show()


