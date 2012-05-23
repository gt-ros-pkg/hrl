import roslib; roslib.load_manifest('trf_learn')
import numpy as np
import pylab as pb
import hrl_lib.util as ut
import sys
import pdb

def transforms_to_arrays(tlist):
    trans_list = []
    rotat_list = []

    for t in tlist:
        trans = t.transform.translation
        rotat = t.transform.rotation
        trans_arr = [trans.x, trans.y, trans.z]
        rotat_arr = [rotat.x, rotat.y, rotat.z, rotat.w]
        trans_list.append(trans_arr)
        rotat_list.append(rotat_arr)
    return np.matrix(trans_list).T, np.matrix(rotat_list).T

if __name__ == '__main__':
    fname = sys.argv[1]
    data = ut.load_pickle(fname)
    print 'locations', data['locations'], data['locations'][0].__class__
    print 'base poses', data['base_poses'][0][0], data['base_poses'][0][0].__class__
    print 'messages', len(data['messages']), data['messages'][0].__class__
    mats = [transforms_to_arrays(l) for l in data['base_poses']]
    pdb.set_trace()
    print 'done!'
