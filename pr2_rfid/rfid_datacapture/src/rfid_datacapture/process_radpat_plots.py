#!/usr/bin/python

# Basically a giant script.

import roslib
roslib.load_manifest( 'geometry_msgs' )  # the pickle files containe Point and Pose Stamped.
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped


import sys
import glob
import yaml
import time
import optparse
import cPickle as pkl
import numpy as np, math
import pylab as pl

import friis

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
    yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/rad_plot_combined.yaml'


# SCRIPT:


if not yaml_fname:
    print 'YAML file required!'
    exit()
else:
    f = open( yaml_fname )
    yaml_config = yaml.load( f )
    f.close()

# XBINS = yaml_config['rad_histbins']['xbins']
# YBINS = yaml_config['rad_histbins']['ybins']
XBINS = 50
YBINS = 50
XMIN = yaml_config['rad_histbins']['xmin']
XMAX = yaml_config['rad_histbins']['xmax']
YMIN = yaml_config['rad_histbins']['ymin']
YMAX = yaml_config['rad_histbins']['ymax']


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

def planar_xy( reading ):
    # Model estimate of P^inc_tag.
    # r_rdr, theta_rdr, phi_rdr, r_tag, theta_tag, phi_tag, rssi, antname, tagid = reading
    d_rdr = reading[0]
    d_tag = reading[1]
    read = reading[2]
    d_rot = reading[3]
    ps = reading[4]

    r_rdr, theta_rdr, phi_rdr = d_rdr
    r_tag, theta_tag, phi_tag = d_tag

    x,y,z = friis.SphereToCart( r_rdr, theta_rdr, phi_rdr )
    return [x,y]


fnames = reduce( lambda x,y: x+y, [ glob.glob(i) for i in yaml_config['glob_files'] ], [] )

if len(glob.glob(yaml_config['use_combined'])) > 0:
    print 'Loading pickle: %s' % (yaml_config['use_combined'])
    f = open( yaml_config['use_combined'], 'r' )
    data = pkl.load( f )
    f.close()
    print 'Done.'
else:
    f = open( yaml_config['use_combined'], 'w' )
    d = reduce( add_files, zip(fnames,range(len(fnames))), {} )

    # Apply Filters:

    # Start with just the desired tagids
    all_reads = reduce( lambda x,y: x+y,
                        [ d[k] for k in yaml_config['filters']['tags'] if d.has_key(k) ],
                        [] )

    print '*** File \'%s\' had a total of %d reads ***' % ( yaml_fname, len( all_reads ))

    # Filter based on antennas
    # return [ [r_rdr, theta_rdr, phi_rdr],  # all floats
    #          [r_tag, theta_tag, phi_tag],  # all floats
    #          [rr.rssi, rr.antenna_name, rr.tagID ], # int, string, string
    #          [theta_rot_map, theta_tag_map], # floats (radians)
    #          [ tag_map, rdr_map, rot_map ] ] # geometry_msgs/PoseStamped

    ant_dict = dict.fromkeys( yaml_config['filters']['antennas'] )
    filt_reads = [ r for r in all_reads if ant_dict.has_key( r[2][1] ) ]

    reads = filt_reads

    data = np.array([ planar_xy(r) + [ r[2][0] ] for r in reads ]).T  # 3xN: x,y,rssi
    print 'Dumping data into combined pickle file: %s ' % (yaml_config['use_combined'])
    pkl.dump( data, f, -1 )
    f.close()
    print 'Done.  Re-run.'
    exit()


# data will be 3xN => x, y, RSSI


# Calculate Useful Values

xy = data[0:2,:]
rssi = data[2]
pos_mask = rssi != -1
neg_mask = rssi == -1

# *******************
# Reads per location
# *******************
H,xedges,yedges = np.histogram2d( xy[0], xy[1],
                                  bins=(XBINS,YBINS),
                                  range=[[XMIN,XMAX],
                                         [YMIN,YMAX]])
XS,YS = np.meshgrid( xedges, yedges )

bins_ind_x = np.sum( xy[0][:,np.newaxis] > xedges[:-1], axis = 1 ) - 1  # Tells the index for each of the reads
bins_ind_y = np.sum( xy[1][:,np.newaxis] > yedges[:-1], axis = 1 ) - 1

d = np.copy( H.T ) # The H matrices are actually transposed from how we display
d[ np.where( d > 100 ) ] = 100 # I just want to see which locations have few / no reads.
dma = np.ma.array( d, mask=(d<yaml_config['points_per_loc']) )

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, dma, cmap=pl.cm.jet ) # or hot?
pl.clim( 0.0, 100.0 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.title( 'Number of reads attempts at each location' )
pl.savefig( yaml_config['outimage'] + '_datapoints_masked.png' )

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, d, cmap=pl.cm.jet ) # or hot?
pl.clim( 0.0, 100.0 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.title( 'Number of reads attempts at each location' )
pl.savefig( yaml_config['outimage'] + '_datapoints_notmasked.png' )

# *******************
# Tag detection probability
# *******************

# Note... I'm still using H from above!

# Need to rebuild dma to not be capped at 100 reads.
dma = np.ma.array( np.copy(H.T), mask=(H.T<yaml_config['points_per_loc']) )
H_pos,xedges,yedges = np.histogram2d( xy[0][pos_mask], xy[1][pos_mask],
                                      bins=(XBINS,YBINS),
                                      range=[[XMIN,XMAX],
                                             [YMIN,YMAX]])

# Where was it actually detected.
dma_det = np.ma.array( np.copy(H_pos.T), mask=(H.T<yaml_config['points_per_loc']) )

# Compute the probability...
dma_det[ np.where(dma.mask==False) ] = (1.0*dma_det[ np.where(dma_det.mask==False) ]) / (1.0*dma[ np.where(dma_det.mask==False) ])

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, dma_det, cmap=pl.cm.jet ) # or hot?
pl.clim( 0.0, 1.0 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.title( 'Probability of Tag Detection' )
pl.savefig( yaml_config['outimage'] + '_tag_detect_prob.png' )


# *******************
# Mean RSSI
# *******************

# Note... I'm still using H and H_pos from above!

def get_mean( xi, yi ):
    # Which indices (into 3xN data) correspond to this x-bin and y-bin?
    data_ind = np.intersect1d( np.where( bins_ind_x == yi )[0], np.where( bins_ind_y == xi )[0] )
    rm = np.mean([ r for r in rssi[ data_ind ] if r != -1 ])
    return rm

def get_std( xi, yi ):
    # Which indices (into 3xN data) correspond to this x-bin and y-bin?
    data_ind = np.intersect1d( np.where( bins_ind_x == yi )[0], np.where( bins_ind_y == xi )[0] )
    rm = np.std([ r for r in rssi[ data_ind ] if r != -1 ])
    return rm

# To calculate the rssi mean...
dma_rm = np.ma.array( np.copy(H_pos.T), mask=(H_pos.T<yaml_config['rssi_points_per_loc']) )

means = np.ma.copy( dma_rm )
for i, (xi,yi) in enumerate( zip( *np.where( dma_rm.mask == False ))):
    # Note: the bin indices are relative to H-matrices, which are transposed.  So we switch xi/yi
    means[xi,yi] = get_mean( xi, yi )

stddev = np.ma.copy( dma_rm )
for i, (xi,yi) in enumerate( zip( *np.where( dma_rm.mask == False ))):
    # Note: the bin indices are relative to H-matrices, which are transposed.  So we switch xi/yi
    stddev[xi,yi] = get_std( xi, yi )
    

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, means, cmap=pl.cm.jet ) # or hot?
pl.clim( 72,96 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.title( 'Mean RSSI' )
pl.savefig( yaml_config['outimage'] + '_tag_rssi_mean.png' )

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, stddev, cmap=pl.cm.jet ) # or hot?
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.title( 'Standard Deviation of RSSI' )
pl.savefig( yaml_config['outimage'] + '_tag_rssi_std.png' )


if PLOT:
    pl.show()



# SAVE SENSOR MODEL

fname = yaml_config['use_combined'].replace('.pkl','_MODEL.pkl')

MODEL = { 'detect_model': np.ma.copy( dma_det ),
          'rssi_model': np.ma.copy( means ),
          'stddev_model': np.ma.copy( stddev ),
          'xedges': xedges, # These are necessary to define the boundaries of the bins.
          'yedges': yedges } 

f = open( fname, 'w' )
pkl.dump( MODEL, f, -1 )
f.close()

