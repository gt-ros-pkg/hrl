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
import string
from scipy.spatial import KDTree
import matplotlib as mpl

import friis
import math_util as mu

PLOT = False

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
    yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/dir_est_head_datacap.yaml'
    # yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/dir_est_shoulder_datacap.yaml'
    
if not yaml_fname:
    print 'YAML file required!'
    exit()
else:
    f = open( yaml_fname )
    yaml_config = yaml.load( f )
    f.close()

def desired_ant( r ):
    if dict.fromkeys(yaml_config['filters']['antennas']).has_key( r[2][1] ):
        return True
    else:
        return False

# def xy( reading ):
#     # Model estimate of P^inc_tag.
#     # r_rdr, theta_rdr, phi_rdr, r_tag, theta_tag, phi_tag, rssi, antname, tagid = reading
#     d_rdr = reading[0]
#     d_tag = reading[1]
#     read = reading[2]
#     d_rot = reading[3]
#     ps = reading[4]

#     tag_map, rdr_map, rot_map, base_map = ps
#     return [ base_map.pose.position.x, base_map.pose.position.y ]
                                    

class Scan():
    def __init__( self, d, tagid, fname ):
        self.d = d
        self.tagid = tagid
        self.fname = fname
        
        if not self.d.has_key( self.tagid ):
            print 'ERROR: %s does not have tagid \'%s\'' % (self.fname, self.tagid)
            exit()
            
        self.theta_rot_map = [ r[3][0] for r in self.d[ self.tagid ] if desired_ant( r )]
        self.theta_tag_map = [ r[3][1] for r in self.d[ self.tagid ] if desired_ant( r )]
        self.rssi = [ r[2][0] for r in self.d[ self.tagid ] if r[2][1] if desired_ant( r )]

        self.dat = np.row_stack([ np.array( self.theta_rot_map ),
                                  np.array( self.theta_tag_map ),
                                  np.array( self.rssi ) ])

        self.theta_tag_gt = np.mean( self.dat[1] )  # The actual estimates for gt will fluctuate per read due to tf

        # We want to know if the tag was inview and if there were positive reads
        self.h, self.bins = np.histogram( self.dat[0], 36, (-np.pi, np.pi )) # 10-deg width bins
        self.bin_centers = (self.bins[:-1] + self.bins[1:]) / 2.0
        self.ind_all = np.sum( self.dat[0][:,np.newaxis] > self.bins[:-1], axis = 1) - 1  # gives indices for each datapoint into corresponding bin

        self.ind_tag = np.sum( np.array([ self.theta_tag_gt ])[:,np.newaxis] > self.bins[:-1], axis = 1) - 1
        self.inview = np.sum( self.ind_all == self.ind_tag ) > 0  # was there at least one reading in the ground-truth bin?
        self.hasposreads = np.sum( self.dat != -1 ) > 0

        # Histmax estimate only uses positive readings
        self.m_rssi = [ np.mean( self.dat[2,np.intersect1d( np.where( self.ind_all == i )[0],
                                                            np.where( self.dat[2] != -1 )[0] )])
                        for i in xrange(len(self.h))]
        self.m_rssi = np.nan_to_num( np.array( self.m_rssi ))  # convert all the places with nan (mean of zero readings) to

        self.ind_max = np.argmax( self.m_rssi )
        self.max_m_rssi = self.m_rssi[ self.ind_max ]
        self.histmax_est = self.bin_centers[ self.ind_max ]
        self.histmax_err = mu.standard_rad( self.histmax_est - self.theta_tag_gt )
        
        # Static estimation: pick the midpoint angle.
        #    We assume theta_rot_map will be contiguous between [-pi,pi] --> choose a good rotation frame.
        self.static_est = np.min(self.dat[0]) + ( np.max(self.dat[0]) - np.min(self.dat[0]) ) / 2.0
        self.static_err = mu.standard_rad( self.static_est - self.theta_tag_gt )

        first_read = self.d[ self.tagid ][0]
        pose_robot = first_read[4][3]
        prx = pose_robot.pose.position.x
        pry = pose_robot.pose.position.y
        
        pose_tag   = first_read[4][0]
        ptx = pose_tag.pose.position.x
        pty = pose_tag.pose.position.y

        dist = np.sqrt( (prx-ptx)**2.0 + (pry-pty)**2.0 )
        
        self.stats = [ prx,
                       pry,
                       dist,
                       self.max_m_rssi,
                       self.histmax_est,
                       self.histmax_err,
                       self.static_est,
                       self.static_err ]
        return

        
fnames = reduce( lambda x,y: x+y, [ glob.glob(i) for i in yaml_config['glob_files'] ], [] )

# Load all pkl files into one huge dictionary.
# d = { 'tagid1': [ PROC_READ1, PROC_READ2, ... ],
#       ...
#     }

scans = []
for i,fname in enumerate( fnames ):
    print 'Loading (%d of %d): %s' % (i+1, len(fnames), fname)
    f = open( fname, 'r' )
    d_new = pkl.load( f )
    f.close()

    for k in d_new.keys():
        if dict.fromkeys(yaml_config['filters']['tags']).has_key( k ):
            scans += [ Scan( d_new, k, fname ) ]

skipped = 0
stats = []

for s in scans:
    if not s.inview or not s.hasposreads:
        skipped += 1
    else:
        if PLOT:
            f = pl.figure( figsize=(10,6) )
            pl.axes([0.1,0.1,0.65,0.8])
            pl.plot( s.dat[0] * 180.0 / np.pi, np.clip(s.dat[2],69,110), 'rx' )
            pl.hold( True )
            pl.plot( [ s.theta_tag_gt * 180.0 / np.pi ], [ np.max(s.dat[2])+2 ], 'bo' ) # tag groundtruth
            pl.plot( [ s.histmax_est  * 180.0 / np.pi ], [ np.max(s.dat[2])+1 ], 'ko' ) # histmax estiamte
            pl.ylim( (68,105) )
            pl.xlim( (-180,180) )
            pl.xlabel( 'Rotation Angle (degrees)')
            pl.ylabel( 'RSSI')
            pl.title( 'RSSI versus rotation angle' )
            pl.legend(['RFID reads', 'Groundtruth', 'ARGMAX Est'], loc=(1.03,0.2))

            f = s.fname
            sr = string.rfind( f, '/' )
            pl.savefig( yaml_config['outimage'] + f[sr+1:sr+string.find(f[sr:],'.')] + '_pincrdr.png' )
            pl.close()

        stats.append( s.stats )

npstats = np.array( stats ).T

print 'Skipped (not in view or no positive reads): ', skipped

print 'ARGMAX Stats:'
herr = npstats[5]
magherr = np.abs( herr )
print '\tmean err: ', math.degrees(np.mean( herr ))
print '\tstd err: ',  math.degrees(np.std(  herr ))
print '\tRMS err (should be same as stderr): ', math.degrees( np.sqrt(np.mean(np.power(herr, 2.0))) )
print '\tmean magerr: ',   math.degrees(np.mean( magherr ))
print '\tstddev magerr: ', math.degrees(np.std(  magherr ))


print 'STATIC Stats:'
serr = npstats[7]
magserr = np.abs( serr )
print '\tmean err: ', math.degrees(np.mean( serr ))
print '\tstd err: ',  math.degrees(np.std(  serr ))
print '\tRMS err (should be same as stderr): ', math.degrees( np.sqrt(np.mean(np.power(serr, 2.0))) )
print '\tmean magerr: ',   math.degrees(np.mean( magserr ))
print '\tstddev magerr: ', math.degrees(np.std(  magserr ))



# Setup for plots below
dist = npstats[2]
h_d,bins_d = np.histogram( dist, 8, range=(0.0,8.0)) # we want to consider distances out to 0m-8m
ind_d = np.sum( dist[:,np.newaxis] > bins_d[:-1], axis=1) - 1
num_d = np.array([ len( np.where( ind_d == i )[0] ) for i in xrange(len(h_d)) ])

rssi = npstats[3]
h_r,bins_r = np.histogram( rssi, 8, range=(71,101)) # we want to consider distances out to 0m-8m
ind_r = np.sum( rssi[:,np.newaxis] > bins_r[:-1], axis=1) - 1
num_r = np.array([ len( np.where( ind_r == i )[0] ) for i in xrange(len(h_r)) ])

# Means (md) and StandardDev (sd) at Distance (in Degrees)
magherr_md = np.array([ np.mean( magherr[ np.where( ind_d == i )[0] ]) for i in xrange(len(h_d)) ]) * 180.0 / np.pi 
magherr_sd = np.array([ np.std(  magherr[ np.where( ind_d == i )[0] ]) for i in xrange(len(h_d)) ]) * 180.0 / np.pi
magherr_sd[ np.where( num_d < 3 )[0] ] = 0 # Only put errorbars where we have sufficient data!

magserr_md = np.array([ np.mean( magserr[ np.where( ind_d == i )[0] ]) for i in xrange(len(h_d)) ]) * 180.0 / np.pi 
magserr_sd = np.array([ np.std(  magserr[ np.where( ind_d == i )[0] ]) for i in xrange(len(h_d)) ]) * 180.0 / np.pi
magserr_sd[ np.where( num_d < 3 )[0] ] = 0 # Only put errorbars where we have sufficient data!

# Means (mr) and StandardDev (sr) at MaxRSSI (in Degrees)
magherr_mr = np.array([ np.mean( magherr[ np.where( ind_r == i )[0] ]) for i in xrange(len(h_r)) ]) * 180.0 / np.pi 
magherr_sr = np.array([ np.std(  magherr[ np.where( ind_r == i )[0] ]) for i in xrange(len(h_r)) ]) * 180.0 / np.pi
magherr_sr[ np.where( num_r < 3 )[0] ] = 0 # Only put errorbars where we have sufficient data!

magserr_mr = np.array([ np.mean( magserr[ np.where( ind_r == i )[0] ]) for i in xrange(len(h_r)) ]) * 180.0 / np.pi 
magserr_sr = np.array([ np.std(  magserr[ np.where( ind_r == i )[0] ]) for i in xrange(len(h_r)) ]) * 180.0 / np.pi
magserr_sr[ np.where( num_r < 3 )[0] ] = 0 # Only put errorbars where we have sufficient data!


# | Err | vs. distance (Bar Plot)

f = pl.figure( figsize=(10,6) )
pl.axes([0.1,0.1,0.65,0.8])
pl.bar( bins_d[:-1], magherr_md, 1.0, color='g', yerr=magherr_sd )
pl.xlim( (0.0,8.0) )
pl.ylim( (-20.0,180) )
pl.xlabel( 'Distance From Tag (m)')
pl.ylabel( '|Angular Error| (degrees)')
# pl.title( 'ARGMAX' )
pl.savefig( yaml_config['outimage'] + '_magherr_vs_dist.png' )
pl.close()

f = pl.figure( figsize=(10,6) )
pl.axes([0.1,0.1,0.65,0.8])
pl.bar( bins_d[:-1], magserr_md, 1.0, color='r', yerr=magserr_sd )
pl.xlim( (0.0,8.0) )
pl.ylim( (-20.0,180) )
pl.xlabel( 'Distance From Tag (m)')
pl.ylabel( '|Angular Error| (degrees)')
# pl.title( 'STATIC' )
pl.savefig( yaml_config['outimage'] + '_magserr_vs_dist.png' )


# | Err | vs. MaxRSSI (Bar Plot)

f = pl.figure( figsize=(10,6) )
pl.axes([0.1,0.1,0.65,0.8])
pl.bar( bins_r[:-1], magherr_mr, bins_r[1] - bins_r[0], color='g', yerr=magherr_sr )
pl.xlim( (69,105) )
print 'Max RSSI (argmax):'
print '\t', bins_r[:-1]
print '\t', magherr_mr
print '\t', magherr_sr
pl.ylim( (-20.0,180) )
pl.xlabel( r'Max $\mathsf{E}[P(RSSI|\tau)]$')
pl.ylabel( '|Angular Error| (degrees)')
# pl.title( 'ARGMAX' )
pl.savefig( yaml_config['outimage'] + '_magherr_vs_mrssi.png' )
pl.close()

f = pl.figure( figsize=(10,6) )
pl.axes([0.1,0.1,0.65,0.8])
pl.bar( bins_r[:-1], magserr_mr, bins_r[1] - bins_r[0], color='r', yerr=magserr_sr )
print 'Max RSSI (static):'
print '\t', bins_r[:-1]
print '\t', magserr_mr
print '\t', magserr_sr
pl.xlim( (69,105) )
pl.ylim( (-20.0,180) )
pl.xlabel( r'Max $\mathsf{E}[P(RSSI|\tau)]$')
pl.ylabel( '|Angular Error| (degrees)')
# pl.title( 'ARGMAX' )
pl.savefig( yaml_config['outimage'] + '_magserr_vs_mrssi.png' )
pl.close()


# | Err | vs. location  (Surface Plot)

XBINS = 9
YBINS = 5
XMIN = 0.0
XMAX = 11.0
YMIN = 0.0
YMAX = 7.0

xy = np.row_stack([ npstats[0],
                    npstats[1] ])  # xy as 2xN
                     
H,xedges,yedges = np.histogram2d( xy[0], xy[1],
                                  bins=(XBINS,YBINS),
                                  range=[[XMIN,XMAX],
                                         [YMIN,YMAX]])
XS,YS = np.meshgrid( xedges, yedges )

bins_ind_x = np.sum( xy[0][:,np.newaxis] > xedges[:-1], axis = 1 ) - 1  # Tells the index for each of the reads
bins_ind_y = np.sum( xy[1][:,np.newaxis] > yedges[:-1], axis = 1 ) - 1

def get_mean( xi, yi, darr ):
    # Which indices (into 3xN data) correspond to this x-bin and y-bin?
    data_ind = np.intersect1d( np.where( bins_ind_x == yi )[0], np.where( bins_ind_y == xi )[0] )
    return np.mean( darr[ data_ind ]) * 180 / np.pi

# import pdb
# pdb.set_trace()

magherr_Z = np.ma.array( np.copy(H.T), mask=(H.T < 1) )
magserr_Z = np.ma.array( np.copy(H.T), mask=(H.T < 1) )

# pdb.set_trace()

for i, (xi,yi) in enumerate( zip( *np.where( magherr_Z.mask == False ))):
    magherr_Z[xi,yi] = get_mean( xi, yi, magherr )
    magserr_Z[xi,yi] = get_mean( xi, yi, magserr )

# pdb.set_trace()

f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, magherr_Z, cmap=pl.cm.jet ) # or hot?
pl.xlim( (-1,12) )
pl.ylim( (-1,8) )
pl.axis( 'equal' )
pl.clim( 0.0, 180.0 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.savefig( yaml_config['outimage'] + '_magherr_contour.png' )
pl.close()


f = pl.figure( figsize=(10,6) )
pl.hold(True)
pl.pcolor( XS, YS, magserr_Z, cmap=pl.cm.jet ) # or hot?
pl.xlim( (-1,12) )
pl.ylim( (-1,8) )
pl.axis( 'equal' )
pl.clim( 0.0, 180.0 )
pl.colorbar()
pl.xlabel( 'X-Coordinate (meters)' )
pl.ylabel( 'Y-Coordinate (meters)' )
pl.savefig( yaml_config['outimage'] + '_magserr_contour.png' )
pl.close()
