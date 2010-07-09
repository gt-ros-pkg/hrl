
import math, numpy as np
import glob

import roslib; roslib.load_manifest('hrl_tilting_hokuyo')
import hrl_tilting_hokuyo.display_3d_mayavi as d3m

import matplotlib_util.util as mpu
import hrl_lib.util as ut


if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('-f', '--fname', action='store', default='',
                 type='string', dest='fname', help='pkl with logged data')
    p.add_option('-d', '--dir', action='store', default='',
                 type='string', dest='dir', help='directory with logged data')
    opt, args = p.parse_args()

    if opt.dir != '':
        poses_pkl = glob.glob(opt.dir + '/poses_dict*.pkl')[0]
        d = ut.load_pickle(poses_pkl)
    elif opt.fname != '':
        d = ut.load_pickle(opt.fname)
    else:
        raise RuntimeError('need either -d or -f')

    hand_list = d['hand']['pos_list']
    hand_rot = d['hand']['rot_list']
    if hand_list != []:
        hand_mat = np.column_stack(hand_list)
        print 'hand_mat.shape', hand_mat.shape
        d3m.plot_points(hand_mat, color = (1.,0.,0.), mode='sphere')

        directions_x = (np.row_stack(hand_rot)[:,0]).T.reshape(len(hand_rot), 3).T
        directions_z = (np.row_stack(hand_rot)[:,2]).T.reshape(len(hand_rot), 3).T
        #print 'directions.shape', directions_x.shape
        #print 'hand_mat.shape', hand_mat.shape

        #mpu.plot_yx(ut.norm(directions).A1, axis=None)
        #mpu.show()
        #mpu.plot_yx(ut.norm(hand_mat[:, 1:] - hand_mat[:, :-1]).A1, axis=None)
        #mpu.show()

        d3m.plot_normals(hand_mat, directions_x, color=(1.,0,0.))
        d3m.plot_normals(hand_mat, directions_z, color=(0.,0,1.))

    mechanism_list = d['mechanism']['pos_list']
    mechanism_rot =  d['mechanism']['rot_list']
    #mechanism_list = []
    if mechanism_list != []:
        mechanism_mat = np.column_stack(mechanism_list)
        print 'mechanism_mat.shape', mechanism_mat.shape
        d3m.plot_points(mechanism_mat, color = (0.,0.,1.), mode='sphere')
        #d3m.plot_points(mechanism_mat, color = (0.,0.,1.), mode='sphere')
        c = np.row_stack(mechanism_rot)[:,2]
        directions = c.T.reshape(len(mechanism_rot), 3).T
        print 'directions.shape', directions.shape
        print 'mechanism_mat.shape', mechanism_mat.shape
        d3m.plot_normals(mechanism_mat, directions, color=(0,0,1.))

    d3m.show()

#    mpu.plot_yx(pos_mat[0,:].A1, pos_mat[1,:].A1)
#    mpu.show()



