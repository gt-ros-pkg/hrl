
import math, numpy as np
import glob
import sys, time
sys.path.append('../handheld_hook/')

import analyse_logs as al
import matplotlib_util.util as mpu
import hrl_lib.util as ut

if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='directory with logged data')
    opt, args = p.parse_args()

    ft_pkl = glob.glob(opt.dir + '/ft_log*.pkl')[0]
    poses_pkl = glob.glob(opt.dir + '/poses_dict*.pkl')[0]

    ft_dict = ut.load_pickle(ft_pkl)
    poses_dict = ut.load_pickle(poses_pkl)
    mechanism_dict = poses_dict['mechanism']
    hand_dict = poses_dict['hand']
    
    ft_time_list = ft_dict['time_list']
    mechanism_dict['time_list'] = ft_time_list
    hand_dict['time_list'] = ft_time_list

#----------- time synchronize --------------
    print 'Begin synchronize'
    d = al.synchronize(ft_dict, mechanism_dict, hand_dict)
    print 'End synchronize'
    #ut.save_pickle(d, opt.dir+'/combined_log'+ut.formatted_time()+'.pkl')
    ut.save_pickle(d, opt.dir+'/combined_log.pkl')
    print 'Saved pickle'

#------------ compute radial and tangential forces ----------
    pkl_name = glob.glob(opt.dir + '/combined_log*.pkl')[0]
    mech_pkl_name = glob.glob(opt.dir + '/mechanism_info*.pkl')[0]

    md = ut.load_pickle(mech_pkl_name)
    cd = ut.load_pickle(pkl_name)
    cd['hook_checker_number'] = md['checkerboard_number']
    cd['radius'] = md['radius']
    rad, tan, ang, typ = al.compute_mechanism_properties(cd)
    rad, tan_b, ang, typ = al.compute_mechanism_properties(cd,
                                        bias_ft=True)

#------------ plot spring scale and FT data -------------
    spring_pkl = glob.glob(opt.dir+'/spring_scale*.pkl')[0]
    spring_list = ut.load_pickle(spring_pkl)

    mpu.plot_yx(spring_list, label='Spring Scale', color='b', axis=None)

    mpu.plot_yx(rad, label='Measured Radial force (unbiased)', color='r',
                xlabel='Reading number', ylabel='Force (N)', axis=None)

    mpu.plot_yx(tan, label='Measured Tangential force (unbiased)', color='g',
                xlabel='Reading number', ylabel='Force (N)', axis=None)
    mpu.plot_yx(tan_b, label='Measured Tangential force (biased)', color='y',
                xlabel='Reading number', ylabel='Force (N)',
                axis=None, plot_title=opt.dir)

    mpu.legend()
    mpu.savefig(opt.dir.split('/')[0]+'.png')
    mpu.show()





