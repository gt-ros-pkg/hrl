#!/usr/bin/python

# BRUTE FORCE EXCESSIVENESS!

import rfid_model
import lib_pfilter

import random as rd
import cPickle as pkl
import numpy as np, math
import pylab as pl
import time
import glob



tdb = { 0: ['OrangeMedBot',[]],
        1: ['TravisTVremo',[]],
        2: ['RedBottle   ',[]],
        3: ['OnMetalKeys ',[]],
        4: ['WhiteMedsBot',[]],
        5: ['BlueMedsBox ',[]],
        6: ['TeddyBearToy',[]],
        7: ['CordlessPhon',[]],
        8: ['BlueHairBrus',[]]}

pts = { 0: ['BehindTree',[3.757, 6.017, 0.036]],
        1: ['FireplaceMantle',[5.090, 4.238, 1.514]],
        2: ['CircleEndTable',[5.399, 2.857, 0.607]],
        3: ['Couch',[3.944, 1.425, 0.527]],
        4: ['RectEndTable',[3.302, 0.932, 0.534]],
        5: ['BehindKitchenTable',[-0.339, -2.393, 0.793]],
        6: ['NearDishwaser',[-1.926, -0.835, 0.946]],
        7: ['InCupboard',[-3.257, 1.294, 1.397]],
        8: ['OnFilingCabinet',[-0.083, 2.332, 0.670]]}


pf_costmap = '/home/travis/svn/robot1/src/projects/rfid_pf/src/rfid_pf/pf_costmap.pkl'

f = open( pf_costmap )
costmap = pkl.load( f )
f.close()

# location_xy = np.array( pts[ loc ][1][0:2] )

def best_location( loc_num ):  
    tag_loc_xyz = pts[loc_num][1]
    
    location_xy = np.array( tag_loc_xyz[0:2] )
    
    cm = costmap[ np.where( costmap[:,2] )[0] ] # Locations where the robot can be located
    dxycm = cm[ :, 0:2 ] - location_xy
    dist = np.sqrt( np.power( dxycm[:,0], 2.0 ) +
                    np.power( dxycm[:,1], 2.0 ))

    sind = np.argsort( dist ) # just so I can look at several nearby.

    best_pos = cm[sind[0],0:2]
    dxy = dist[sind[0]]
    next_nearest = dist[ sind[1] ]  # NOTE: I already verified that the best ones are unique!

    print 'BEST for location %d: %s' % ( loc_num, pts[loc_num][0] )
    print '\tBest Position     ', best_pos
    print '\tErr (meters)      ', dxy
    print

    return best_pos, dxy



def uniform_predict( loc_num ):
    tag_loc_xyz = pts[loc_num][1]

    location_xy = np.array( tag_loc_xyz[0:2] )
    cm = costmap[ np.where( costmap[:,2] )[0] ] # Locations where the robot can be located

    dxycm = cm[ :, 0:2 ] - location_xy
    dist = np.sqrt( np.power( dxycm[:,0], 2.0 ) +  # error
                    np.power( dxycm[:,1], 2.0 ))

    sind = np.argsort( dist ) # just so I can look at several nearby.

    # Brute force estimate picking a random location from the set of possible locations
    dxy_mean = np.mean( dist )  # Mean error for a random point
    dxy_std = np.std( dist )

    # Uniform angle distribution (for given position) between [-pi,pi]
    #   ==> Uniform error distribution over [0,pi]
    
    # mean ==> 1/2 (a + b)         ==> (1/2)*(180-deg + 0)          = 90.0-deg
    # std  ==> var = 1/12 (b-a)**2 ==> sqrt( 1/12 ) * (180-deg - 0) = 51.5-deg

    dtheta_mean = 0.5 * ( np.pi + 0.0 )
    dtheta_std  = np.sqrt( 1.0 / 12.0 ) * ( np.pi - 0 )

    # # Brute force verification
    # # Any value is equally probable for actual angle... pick (pi / 4)

    # roslib.load_manifest( 'rfid_datacapture' )
    # import rfid_datacapture.math_util as mu

    # th = np.pi / 4
    # thetas = np.linspace( -1.0 * np.pi, np.pi, 360, endpoint=False )
    # err = np.abs([ mu.standard_rad( th - i ) for i in thetas ])

    # dtheta_mean = np.mean( err )  # ==> CONFIRMED: 90.0-deg
    # dtheta_std = np.std( err )    # ==> CONFIRMED: 51.9-deg

    print 'UNIFORM for location %d: %s' % ( loc_num, pts[loc_num][0] )
    print '\tPosition Err (meters):  %2.2f (%2.2f)' % (dxy_mean, dxy_std)
    print '\tAngular  Err (deg):     %3.2f (%3.2f)' % (math.degrees( dtheta_mean ),
                                                       math.degrees( dtheta_std ))
    print 

    return dxy_mean, dxy_std, dtheta_mean, dtheta_std
    


if __name__ == '__main__':
    # Compute
    for nn in xrange( 9 ):
        best_location( nn )
        uniform_predict( nn )



