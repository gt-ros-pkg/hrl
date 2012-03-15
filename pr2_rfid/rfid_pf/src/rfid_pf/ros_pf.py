#!/usr/bin/python

import rfid_model
import lib_pfilter

import roslib
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('pfilter')
roslib.load_manifest('tf')
roslib.load_manifest('rfid_behaviors')
import rospy

import visualization_msgs.msg as vm
import hrl_lib.transforms as tr
import hrl_lib.viz as viz
vsm = viz.single_marker
import pfilter.pfilter as pfilter
import tf.transformations as tft
from rfid_behaviors.msg import RecorderReads
from display_particles import DisplayParticles
from geometry_msgs.msg import Quaternion, Point

import random as rd
import cPickle as pkl
import numpy as np, math
import pylab as pl
import time
import glob

rospy.init_node( 'ros_pf' )

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


def process_read( pf, p_start, m ):
    # Note: p_start is numpy array (Nx3) and it's weights ([:,2]) will be modified in-place!
    # m is form: RFIDread, PoseStamped (ant in map), PoseStamped (robot base in map)
    #   We assume it is a postive reading for the desired tagID!

    xy_map = np.copy( p_start[:,0:2].T )  # 2xN
    
    # Antenna is at position given by m.ps_ant_map.
    #   IMPORTANT NOTE: I'm assuming that the antenna XY plane is parallel to the map XY plane!
    o = m.ps_ant_map.pose.orientation
    d = m.ps_ant_map.pose.position
    
    roll, pitch, yaw = tft.euler_from_quaternion([ o.x, o.y, o.z, o.w ])
    dx, dy = d.x, d.y

    # Transform xy_map points into antenna frame
    t_new = tr.composeHomogeneousTransform( tr.rotZ( yaw ),
                                            np.matrix([dx,dy,0.0]).T )
    trans = tr.invertHomogeneousTransform( t_new )

    xy_new = (trans * tr.xyToHomogenous( xy_map ))[0:2]  # This is a matrix!  (2xN)
    xy_new = np.array( xy_new ) # Convert it back to array!

    # Build new particles using old weights (Nx3)
    p_new = np.column_stack([ xy_new.T, p_start[:,2] ])  # take weights from last iteration

    rv = pf.step( 0.0, m.read.rssi, p_new, should_resample_func = pfilter.retFalse )

    # Put back the new weights:
    p_start[:,2] = rv[:,2]
    return



def resample( pf, p_set ):
    print 'RESAMPLING'
    rt0 = time.time()

    # Perform low-variance resampling.  Note: p_set is already normalized
    p_uss = np.matrix(np.column_stack([ p_set[:,0:2],  # x,y
                                        np.zeros( p_set.shape[0] ),
                                        np.ones( p_set.shape[0] ),
                                        p_set[:,2] ]).T)  # w  ---> now 5xN

    p_uss_new = pfilter.set_resample_uss( p_set.shape[0], p_uss )

    p_set[:,0:2] = np.array(p_uss_new[0:2].T)
    p_set[:,2] = np.ones( p_set.shape[0] )

    # The USS resampler will give repeated particles.  Re-distribute them according to Joho.
    xy_set = p_set[:,0:2]

    discount = 0.95
    a = ( 3.0 * discount - 1 ) / ( 2.0 * discount )
    hsquare = 1.0 - np.power(a,2.0)

    p_mean = np.mean( xy_set )
    p_cov = np.cov( xy_set.T )

    for i, xyi in enumerate( xy_set ):
        n_xyi = np.random.multivariate_normal( a * xyi + (1-a) * p_mean,
                                               hsquare * p_cov, 1 ).flatten()
        xy_set[i] = n_xyi

    p_set[:,0:2] = xy_set
    print '\tResampling took %2.1f' % (time.time() - rt0)

    # update_display()
    return
    



def process_run( trial_num, obj_num ):
    # Setup
    obj_name = tdb[obj_num][0]
    tname = obj_name.replace( ' ', '' )

    loc = (trial_num + obj_num) % 9
    loc_name = pts[loc][0]
    loc_pos  = np.array(pts[loc][1])  # Tag ground-truth location

    print 'Trial %d with Object %d (%s) at Position %d (%s)' % (trial_num,
                                                                obj_num,
                                                                obj_name,
                                                                loc,
                                                                loc_name)

    # build particles
    X,Y = np.meshgrid( np.arange(-5,8,0.05),
                       np.arange(-5,8,0.05) )
    xyw = np.row_stack([ X.flatten(),  # Build Nx3
                         Y.flatten(),
                         np.ones( X.shape ).flatten() ]).T # weights (multiplicative)

    p_set = np.copy( xyw )


    # Build pfilter obj.
    pf = pfilter.PFilter( rfid_model.NoMotion(), rfid_model.RfidModel( rfid_model.yaml_fname ))


    # Load search reads
    fname_prefix = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/'

    search_fname = fname_prefix
    search_fname += 'search_aware_home/woot_150_'+str(trial_num)+'_reads.pkl'

    f = open( search_fname, 'r' )
    dat_search_all = pkl.load( f )
    f.close()

    print '\tLEN_ALL (pre): ', len(dat_search_all)
    dat_search = [ d for d in dat_search_all if d.read.rssi != -1 and d.read.tagID == obj_name ] # Only positive reads!
    print '\tLEN_ALL (post): ', len(dat_search)


    # Load servo reads (if applicable): woot_150_7_tag_TeddyBearToy_servo.pkl
    servo_fname = fname_prefix
    servo_fname += 'search_aware_home/'
    servo_fname += 'woot_150_'+str(trial_num)+'_tag_'+obj_name.replace(' ','')+'_servo.pkl'

    glob_r = glob.glob( servo_fname )
    if glob_r == []:
        print '\t No results for this instance.\n\n'
        return
    if len(glob_r) > 1:
        print '\t Multiple results...?! Weirdness.  Skipping.'
        return


    f = open( servo_fname, 'r' )
    dat_servo_all = pkl.load( f )
    f.close()

    print '\tLEN_SERVO (pre): ', len(dat_servo_all)
    dat_servo = [ d for d in dat_servo_all if d.read.rssi != -1 and d.read.tagID == obj_name ] # Only positive reads!
    print '\tLEN_SERVO (post): ', len(dat_servo)
    print ''


    # Apply the particle filter for search (only)
    print '\tApplying Search-Only Measurements (%d)' % (len(dat_search))
    t0 = time.time()
    for m in dat_search:
        process_read( pf, p_set, m )

        # print 'Min: ', np.min( p_set[:,2] )
        # print 'Max: ', np.max( p_set[:,2] )

        neff = 1.0 / np.sum( np.power( p_set[:,2], 2.0 ))
        npart = len( p_set[:,2] )
        neff_rat = 100.0 * neff / npart  # ratio of "effective" particles (as a percentage)
        # print 'Neff: %d of %d (%2.1f %%)' % ( neff, npart, neff_rat )

        # if opt.resamp and neff_rat < 15.0:
        #     resample( pf, p_set )

    # Save the Search (only) results
    pf_fname = servo_fname.replace('_servo.pkl', '_pf_search.pkl')

    f = open( pf_fname, 'w' )
    pkl.dump( np.copy( p_set ), f )
    f.close()


    # Apply the particle filter for rest of servoing reads
    print '\n\tApplying Subsequent Servo Measurements (+%d => %d)' % (len(dat_servo), len(dat_servo)+len(dat_search))
    for m in dat_servo:
        process_read( pf, p_set, m )

        # print 'Min: ', np.min( p_set[:,2] )
        # print 'Max: ', np.max( p_set[:,2] )

        neff = 1.0 / np.sum( np.power( p_set[:,2], 2.0 ))
        npart = len( p_set[:,2] )
        neff_rat = 100.0 * neff / npart  # ratio of "effective" particles (as a percentage)
        # print 'Neff: %d of %d (%2.1f %%)' % ( neff, npart, neff_rat )

        # if opt.resamp and neff_rat < 15.0:
        #     resample( pf, p_set )

    dt = time.time() - t0
    print '\n\tExecution time: %3.2f (sec). Average rate: %3.2f (Hz)\n' % ( dt, (len(dat_search) + len(dat_servo))*1.0 / dt )

    # Save the Search + Servo results
    pf_fname = servo_fname.replace('_servo.pkl', '_pf_servo.pkl')

    f = open( pf_fname, 'w' )
    pkl.dump( np.copy( p_set ), f )
    f.close()

    print 'DONE.\n\n\n'
    return

    



if __name__ == '__main__':
    # for trial_num in [4]:  # trial
    #     for obj_num in [5]:  # object
    #         process_run( trial_num, obj_num )


    for trial_num in xrange( 9 ):  # trial
        for obj_num in xrange( 9 ):  # object
            process_run( trial_num, obj_num )




# Test:

# def create_fake_read( rssi, dx, dy, dt ):
#     rr = RecorderReads()
#     rr.read.rssi = rssi

#     q = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, dt ))
#     p = Point( dx, dy, 0 )

#     rr.ps_ant_map.pose.orientation = q
#     rr.ps_ant_map.pose.position = p
#     return rr

# dat = [ create_fake_read( 84, 0, 0, math.radians( 0 )) ]
# dat += [ create_fake_read( 84, 2, -2, math.radians( 90 )),
#          create_fake_read( 84, 2, 2, math.radians( -90 )),
#          create_fake_read( 84, 4, 0, math.radians( 180 )) ]


# import optparse
# p = optparse.OptionParser()
# p.add_option('--trial', action='store', type='int', dest='trial',
#              help='trial number (0-8)')
# p.add_option('--obj', action='store', type='int', dest='obj',
#              help='object number (0-8)')
# p.add_option('--resamp', action='store_true', dest='resamp',
#              help='resample?', default = False)
# opt, args = p.parse_args()



