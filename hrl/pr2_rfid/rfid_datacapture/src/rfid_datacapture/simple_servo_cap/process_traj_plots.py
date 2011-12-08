#!/usr/bin/python

# Basically a giant script.

import roslib
roslib.load_manifest( 'geometry_msgs' )  # the pickle files containe Point and Pose Stamped.
roslib.load_manifest( 'sensor_msgs' )
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import PointCloud


import sys
import glob
import yaml
import time
import optparse
import cPickle as pkl
import numpy as np, math
import pylab as pl

import friis
import point_cloud_utils as pcu

PLOT = False

# glob_files: '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/datacap/*.pkl'
# filters:
#   antennas:
#     PR2_Head: '/head_rfid'
#   tags:
#     'datacap     ':

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('--yaml', action='store', type='string', dest='yaml', default='',
                 help='yaml file that describes this run.')
    p.add_option('--plot', action='store_true', dest='plot',
                 help='Pop-up the resulting plot')
    opt, args = p.parse_args()

    yaml_fname = opt.yaml
    PLOT = opt.plot
else:
    yaml_fname = ''


# SCRIPT:


if not yaml_fname:
    print 'YAML file required!'
    exit()
else:
    f = open( yaml_fname )
    yaml_config = yaml.load( f )
    f.close()

def add_files( d, arg ):
    fname, fcount = arg
    print 'Loading (%d of %d): %s' % (fcount, len(fnames), fname)
    f = open( fname, 'r' )
    d_new = pkl.load( f )
    f.close()

    for k in d_new.keys():
        if not d.has_key( k ):
            d[k] = []
        d[k] += d_new[k]

    return d

def base_map_xy( reading ):
    # Model estimate of P^inc_tag.
    # r_rdr, theta_rdr, phi_rdr, r_tag, theta_tag, phi_tag, rssi, antname, tagid = reading
    d_rdr = reading[0]
    d_tag = reading[1]
    read = reading[2]
    d_rot = reading[3]
    ps = reading[4]

    tag_map, rdr_map, rot_map, base_map = ps
    return [ base_map.pose.position.x, base_map.pose.position.y ]


fnames = reduce( lambda x,y: x+y, [ glob.glob(i) for i in yaml_config['glob_files'] ], [] )

def pos_finish( reads_list ):
    d_rdr = reads_list[-1][0]
    d_tag = reads_list[-1][1]
    read = reads_list[-1][2]
    d_rot = reads_list[-1][3]
    ps = reads_list[-1][4]

    base_map = ps[3]
    bmx = base_map.pose.position.x
    bmy = base_map.pose.position.y

    if bmx < 7+2 and bmx > 7-2 and bmy > 3-2 and bmy < 3+2:
        rv = True
    else:
        rv = False
    return rv

if len(glob.glob(yaml_config['use_combined'])) > 0:
    print 'Loading pickle: %s' % (yaml_config['use_combined'])
    f = open( yaml_config['use_combined'], 'r' )
    data = pkl.load( f )
    f.close()
    print 'Done.'
else:
    f = open( yaml_config['use_combined'], 'w' )
    #d = reduce( add_files, zip(fnames,range(len(fnames))), {} )

    data = []
    # Apply Filters:
    for i,fname in enumerate( fnames ):
        print 'Loading (%d of %d): %s' % (i, len(fnames), fname)
        f = open( fname, 'r' )
        d_new = pkl.load( f )
        f.close()

        for k in d_new.keys():
            if dict.fromkeys(yaml_config['filters']['tags']).has_key( k ):
                data += [ base_map_xy(r) + [ r[2][0], pos_finish( d_new[k] )] for r in d_new[k] ]

    data = np.array( data ).T # 4xN: x,y,rssi,positive finish

    print 'Dumping data into combined pickle file: %s ' % (yaml_config['use_combined'])
    f = open( yaml_config['use_combined'], 'w' )
    pkl.dump( data, f, -1 )
    f.close()

    print 'Done.  Re-run.'
    exit()


# data will be 4xN => x, y, RSSI, positive finish (bool)

# Calculate Useful Values

xy = data[0:2,:]
xyz = np.row_stack([ xy, np.zeros( xy.shape[1] )])
rssi = data[2]
pos_read = data[3]

pp = pcu.np_points_to_ros( np.matrix(xyz[:,np.where(pos_read > 0.5)[0]]) )
pp.header.frame_id = '/map'

pn = pcu.np_points_to_ros( np.matrix(xyz[:,np.where(pos_read < 0.5)[0]]) )
pn.header.frame_id = '/map'

rospy.init_node( 'traj_pub_node' )
time.sleep( 0.3 )
print 'PUBLISHING'

pubp = rospy.Publisher( 'traj_pub_pos', PointCloud )
pubn = rospy.Publisher( 'traj_pub_neg', PointCloud )

while not rospy.is_shutdown():
    pp.header.stamp = rospy.Time.now()
    pn.header.stamp = rospy.Time.now()

    pubp.publish( pp )
    pubn.publish( pn )

    rospy.sleep( 0.5 )
