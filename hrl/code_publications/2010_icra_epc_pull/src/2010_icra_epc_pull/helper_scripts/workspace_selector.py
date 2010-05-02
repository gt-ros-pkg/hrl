
import hrl_lib.util as ut
import matplotlib_util.util as mpu
import math, numpy as np
import sys

def plot_workspace(pts,ha,z):
    if pts.shape[1] == 0:
        return
    mpu.figure()
    good_location = pts.mean(1)
    mpu.plot_yx(pts[1,:].A1,pts[0,:].A1,label='ha:%.1f'%(math.degrees(ha)),
                axis='equal',linewidth=0)
    mpu.plot_yx(good_location[1,:].A1,good_location[0,:].A1,
                axis='equal',linewidth=0,scatter_size=90,color='k')
    mpu.savefig('z%.2f_ha%.1f.png'%(z,math.degrees(ha)))

argv = sys.argv
fname = sys.argv[1]
dd = ut.load_pickle(fname)

color_list = ['b','y','g']
i = 0
mpu.figure(dpi=100)
for ha in dd.keys():
    d = dd[ha]
    l = []
    key_list = d['pts'].keys()
    for k in key_list:
        pts = d['pts'][k]
        l.append(pts.shape[1])
        #plot_workspace(pts,ha,k)

    ll = zip(key_list,l)
    ll.sort()

    key_list,l = zip(*ll)
    if ha == 0:
        label = 'Hook Left'
    elif abs(ha-math.pi/2) < 0.01:
        label = 'Hook Down'
        continue
    else:
        label = 'Hook Up'

    mpu.plot_yx(key_list,l,axis=None,label=label, color=color_list[i],
                xlabel='\# of points with IK soln',
                ylabel='Height (m)', scatter_size=8)
    i += 1
    max_idx = np.argmax(l)
    good_height = key_list[max_idx]
    print 'good_height:', good_height

    mpu.plot_yx([good_height],[l[max_idx]],axis=None,
                color='r', xlabel='\# of points with IK soln',
                ylabel='Height (m)', scatter_size=8)
    d['height'] = good_height


#ut.save_pickle(dd,fname)
#mpu.legend()
#mpu.savefig('workspace_npts.png')
#mpu.show()


