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
    yaml_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/cap_360/friis_plot_datacap.yaml'


# SCRIPT:


if not yaml_fname:
    print 'YAML file required!'
    exit()
else:
    f = open( yaml_fname )
    yaml_config = yaml.load( f )
    f.close()


# Load all pkl files into one huge dictionary.
# d = { 'tagid1': [ PROC_READ1, PROC_READ2, ... ],
#       ...
#     }
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

# Calculate Useful Values

def friis_pwr_tag( reading ):
    # Model estimate of P^inc_tag.
    # r_rdr, theta_rdr, phi_rdr, r_tag, theta_tag, phi_tag, rssi, antname, tagid = reading
    d_rdr = reading[0]
    d_tag = reading[1]
    read = reading[2]
    d_rot = reading[3]
    ps = reading[4]

    r_rdr, theta_rdr, phi_rdr = d_rdr
    r_tag, theta_tag, phi_tag = d_tag

    watts = friis.pwr_inc_tag( r_rdr,
                               friis.patch.G, theta_rdr, phi_rdr,
                               friis.dipole.G, theta_tag, phi_tag )
    return friis.WattsToDBm( watts )

def friis_pwr_rdr( reading ):
    # Model estimate of P^inc_rdr.
    # r_rdr, theta_rdr, phi_rdr, r_tag, theta_tag, phi_tag, rssi, antname, tagid = reading
    d_rdr = reading[0]
    d_tag = reading[1]
    read = reading[2]
    d_rot = reading[3]
    ps = reading[4]

    r_rdr, theta_rdr, phi_rdr = d_rdr
    r_tag, theta_tag, phi_tag = d_tag
    
    watts = friis.pwr_inc_rdr( r_rdr,
                               friis.patch.G, theta_rdr, phi_rdr,
                               friis.dipole.G, theta_tag, phi_tag )
    return friis.WattsToDBm( watts )



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

    p_inc_tag = np.array([ friis_pwr_tag( r ) for r in reads ]) # in dBm!
    p_inc_rdr = np.array([ friis_pwr_rdr( r ) for r in reads ]) # in dBm!
    rssi      = np.array([ r[2][0] for r in reads ])
    data = np.row_stack([ p_inc_tag,
                          p_inc_rdr,
                          rssi ])
    print 'Dumping data into combined pickle file: %s ' % (yaml_config['use_combined'])
    pkl.dump( data, f, -1 )
    f.close()
    print 'Done.  Re-run.'
    exit()

p_inc_tag = data[0]
p_inc_rdr = data[1]
rssi = data[2]

pos_mask = rssi != -1
neg_mask = rssi == -1

if len(pos_mask) == 0:
    print '### File \'%s\' had no positive reads -- exiting ###' % ( yaml_fname )
    exit()
if len(neg_mask) == 0:
    print '### File \'%s\' had no positive reads -- exiting ###' % ( yaml_fname )
    exit()

# P^inc_rdr vs. RSSI.
def plot_pincrdr( f = None ):
    if not f:
        f = pl.figure( figsize=(10,6) )
        
    pl.axes([0.1,0.1,0.65,0.8])
    pl.plot( p_inc_rdr[pos_mask], rssi[pos_mask], 'bx', alpha = 0.5 )
    pl.xlabel( '$P^{inc}_{rdr}$ (dBm)')
    pl.ylabel( 'RSSI')
    pl.title( 'Measured RSSI vs Predicted Power at Reader' )

    xval = None
    yval = None
    if yaml_config.has_key( 'rdr_calcfriis' ):
        # Old way: take max Pincrdr
        # ind = p_inc_rdr >= np.max( p_inc_rdr ) - yaml_config['rdr_calcfriis']['within_max']

        # New way: take region we know to be linear in Friis.  Note, this also elimates negative reads!
        ind = np.all( np.row_stack([
            rssi >= yaml_config['rdr_calcfriis']['rssi_min'],
            rssi <= yaml_config['rdr_calcfriis']['rssi_max'] ]), axis = 0)

        xval = np.mean( p_inc_rdr[ ind ] )  # Friis line runs throught this point
        yval = np.mean( rssi[ ind ] )

        # CURIOUS: Both methods are the same.  I could probably show that, but I'm tired!

        # We know the slope is 0.75.  y = 0.75 * x + c ==>  c = mean( y - 0.75 x )
        # c = np.mean( rssi[ind] - 0.75 * p_inc_rdr[ind] )  
        # xval = -19.19 # arbitrary, but convenient
        # yval = 0.75 * xval + c
        
        print 'Calculated Friis Line:\n\txval: %3.2f\n\tyval: %3.2f' % (xval,yval)

    if yaml_config.has_key( 'rdr_drawfriis' ):
        xval = yaml_config[ 'rdr_drawfriis' ][ 'xval' ]
        yval = yaml_config[ 'rdr_drawfriis' ][ 'yval' ]

    if xval and yval:
        pl.hold( True )

        # Slope of line (from Matt's measurements) should be 0.75
        xs = np.linspace( yaml_config['rdr_axis'][0], yaml_config['rdr_axis'][1], 100 )
        ys = 0.75 * ( xs - xval ) + yval # pt-slope form
        pl.plot( xs, ys, 'g-', linewidth = 2.0 )
        pl.legend(['Positive Reads','Friis Model Fit'], loc=(1.03,0.2))
    else:
        pl.legend(['Positive Reads'], loc=(1.03,0.2))


    pl.axis( yaml_config['rdr_axis'] )
    return f

plot_pincrdr()
pl.savefig( yaml_config['outimage'] + '_pincrdr.png' )


# P^inc_rdr vs. P(read)
def plot_pincrdr_probs( f = None ):
    hist, bins = np.histogram( p_inc_rdr,
                               bins = yaml_config['rdr_histbins']['bins'],
                               range = (yaml_config['rdr_histbins']['min'],
                                        yaml_config['rdr_histbins']['max']) )
    bin_width = bins[1] - bins[0]
    # Find out which bin each read belongs to.
    # Sanity check: print [ sum( bins_ind == i ) for i in xrange(len(hist)) ] => equals hist
    bins_ind = np.sum( p_inc_rdr[:,np.newaxis] > bins[:-1], axis = 1 ) - 1
    prob_read = [ sum(rssi[bins_ind == i] != -1)*1.0 / hist[i] # positive reads / total reads for bin i
                  for i in xrange(len(hist)) # same as len(bins)-1
                  if hist[i] != 0 ] # only where we have data!  (also prevents div by 0)

    if not f:
        f = pl.figure( figsize=(10,6) )
    pl.axes([0.1,0.1,0.65,0.8])
    pos_bars = pl.bar([ bins[i] for i in xrange(len(hist)) if hist[i] != 0 ], # Only plot the bars for places we have data!
                      prob_read,  # This is only defined for ones that have data!
                      width = bin_width,
                      color = 'b',
                      alpha = 0.7 )
    pl.hold( True )
    neg_bars = pl.bar([ bins[i] for i in xrange(len(hist)) if hist[i] != 0 ], # Only plot the bars for places we have data!
                      [ 1.0 - p for p in prob_read ],  # This is only defined for ones that have data!
                      width = bin_width,
                      bottom = prob_read,
                      color = 'r',
                      alpha = 0.7 )
    pl.axis([ yaml_config['rdr_axis'][0], yaml_config['rdr_axis'][1], 0.0, 1.0 ])
    pl.xlabel( '$P^{inc}_{rdr}$ (dBm)')
    pl.ylabel( 'Probability of Tag Read / No-Read')
    pl.title( 'Probability of Tag Read / No-Read vs Predicted Power at Reader ' )
    pl.legend((pos_bars[0], neg_bars[0]), ('P( read )', 'P( no read )'), loc=(1.03,0.2))

    return f

plot_pincrdr_probs()
pl.savefig( yaml_config['outimage'] + '_pincrdr_probs.png' )




# P^inc_tag vs. P(read)
def plot_pinctag_probs( f = None ):
    pl.figure( figsize=(10,6) )
    pl.axes([0.1,0.1,0.65,0.8])
    pl.plot( p_inc_tag[pos_mask], rssi[pos_mask], 'bx', alpha = 0.5 )
    pl.xlabel( '$P^{inc}_{tag}$ (dBm)')
    pl.ylabel( 'RSSI')
    pl.title( 'Measured RSSI vs Predicted Power at Tag' )
    pl.legend(['Positive Reads'], loc=(1.03,0.2))
    pl.axis( yaml_config['tag_axis'] )
    pl.savefig( yaml_config['outimage'] + '_pinctag.png' )
    
    hist, bins = np.histogram( p_inc_tag,
                               bins = yaml_config['tag_histbins']['bins'],
                               range = (yaml_config['tag_histbins']['min'],
                                        yaml_config['tag_histbins']['max']) )
    bin_width = bins[1] - bins[0]
    # Find out which bin each read belongs to.
    # Sanity check: print [ sum( bins_ind == i ) for i in xrange(len(hist)) ] => equals hist
    bins_ind = np.sum( p_inc_tag[:,np.newaxis] > bins[:-1], axis = 1 ) - 1
    prob_read = [ sum(rssi[bins_ind == i] != -1)*1.0 / hist[i] # positive reads / total reads for bin i
                  for i in xrange(len(hist)) # same as len(bins)-1
                  if hist[i] != 0 ] # only where we have data!  (also prevents div by 0)

    if not f:
        f = pl.figure( figsize=(10,6) )
    pl.axes([0.1,0.1,0.65,0.8])
    pos_bars = pl.bar([ bins[i] for i in xrange(len(hist)) if hist[i] != 0 ], # Only plot the bars for places we have data!
                      prob_read,  # This is only defined for ones that have data!
                      width = bin_width,
                      color = 'b',
                      alpha = 0.7 )
    pl.hold( True )
    neg_bars = pl.bar([ bins[i] for i in xrange(len(hist)) if hist[i] != 0 ], # Only plot the bars for places we have data!
                      [ 1.0 - p for p in prob_read ],  # This is only defined for ones that have data!
                      width = bin_width,
                      bottom = prob_read,
                      color = 'r',
                      alpha = 0.7)
    pl.axis([ yaml_config['tag_axis'][0], yaml_config['tag_axis'][1], 0.0, 1.0 ])
    pl.xlabel( '$P^{inc}_{tag}$ (dBm)')
    pl.ylabel( 'Probability of Tag Read / No-Read')
    pl.title( 'Probability of Tag Read / No-Read vs Predicted Power at Tag' )
    pl.legend((pos_bars[0], neg_bars[0]), ('P( read )', 'P( no read )'), loc=(1.03,0.2))
    # pl.legend(['P( read )'], loc=(1.03,0.2))
    pl.savefig( yaml_config['outimage'] + '_pinctag_probs.png' )
    # return f

plot_pinctag_probs()




if PLOT:
    pl.show()
