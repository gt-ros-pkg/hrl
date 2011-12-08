#!/usr/bin/python

import rfid_model
import lib_pfilter

import roslib
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('pfilter')
import rospy

import hrl_lib.transforms as tr
import tf.transformations as tft
import display_particles
from display_particles import DisplayParticles
from geometry_msgs.msg import Quaternion, Point

import rfid_datacapture.math_util as mu
import visualization_msgs.msg as vm
import hrl_lib.transforms as tr
import hrl_lib.viz as viz
vsm = viz.single_marker


import random as rd
import cPickle as pkl
import numpy as np, math
import pylab as pl
import time
import glob

# trial_num = 4
# obj_num = 3
# servo_yn = False


def pprint(r):
    print '\nLocation %d, Object %d, Trial %d' % (r['loc'], r['obj_num'], r['trial_num'])

    if r['servo_yn']:
        print '\tSEARCH plus SERVO'
    else:
        print '\tSEARCH ONLY'
        
    print '\tPos Reads:      %d' % (r['pos_readings'])
    print '\tTot Reads:      %d' % (r['tot_readings'])
    print '\tPercent Reads:  %2.1f' % (r['pos_readings']*100.0/r['tot_readings'])

    if r['best_pos'].__class__ == ''.__class__:
        print '\tEstimate Loc:   --'
    else:
        print '\tEstimate Loc:   <%2.3f, %2.3f>' % ( r['best_pos'][0], r['best_pos'][1] )

    if r['orient_est'].__class__ == ''.__class__:
        print '\tEstimate Theta: --'
    else:
        print '\tEstimate Theta: %2.1f (deg)' % ( math.degrees( r['orient_est'] ))

    if r['dxy'].__class__ == ''.__class__:
        print '\tDist Err (m):   --'
    else:
        print '\tDist Err (m):   %2.3f' % ( r['dxy'] )

    if r['dtheta'].__class__ == ''.__class__:
        print '\tAng Err (deg):  --'
    else:
        print '\tAng Err (deg):  %2.1f' % ( math.degrees( r['dtheta'] ))

    # if r.has_key('other'):
    #     print '\tOther params: ', r['other']

    print '\n\n\n'        



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

def process_trialobj( trial_num, obj_num, servo_yn ):
    obj_name = tdb[obj_num][0]
    tname = obj_name.replace( ' ', '' )

    loc = (trial_num + obj_num) % 9
    loc_name = pts[loc][0]
    loc_pos  = np.array(pts[loc][1])  # Tag ground-truth location


    fname_prefix = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/'

    servo_fname = fname_prefix
    servo_fname += 'search_aware_home/'
    servo_fname += 'woot_150_'+str(trial_num)+'_tag_'+obj_name.replace(' ','')+'_servo.pkl'

    pf_search = servo_fname.replace('_servo.pkl', '_pf_search.pkl')
    pf_servo = servo_fname.replace('_servo.pkl', '_pf_servo.pkl')



    # Search only
    search_reads_fname = fname_prefix
    search_reads_fname += 'search_aware_home/woot_150_'+str(trial_num)+'_reads.pkl'
    f = open( search_reads_fname, 'r' )
    summary_search = pkl.load( f )
    f.close()

    pos_readings_search = sum([ True for p in summary_search if p.read.rssi != -1 and p.read.tagID == obj_name ])
    tot_readings_search = len( summary_search )

    if pos_readings_search == 0:  # No results!
        print '\t No results for this instance.'

        res = { 'loc': loc,
                'obj_num': obj_num,
                'trial_num': trial_num,
                'pos_readings': pos_readings_search,
                'tot_readings':tot_readings_search,
                'best_pos': '--',
                'orient_est': '--',
                'dxy': '--',
                'dtheta': '--',
                'servo_yn': servo_yn,
                'other': { 'w_mass': 0.0,  # specific to pf
                           'orient_fit': 0.0 }}
        return False, res, None

    f = open( pf_search )  # load the particle set from Search data.  Will overwrite if using search plus servo
    p_set_loaded = pkl.load( f )
    f.close()

    pos_readings = pos_readings_search
    tot_readings = tot_readings_search

    # If this is Search PLUS servo...
    if servo_yn:

        # update the stats for pos reads
        f = open( servo_fname, 'r' )
        summary_servo = pkl.load( f )
        f.close()

        pos_readings_servo = sum([ True for p in summary_servo if p.read.rssi != -1 and p.read.tagID == obj_name ])
        tot_readings_servo = len( summary_servo )

        pos_readings = pos_readings_search + pos_readings_servo
        tot_readings = tot_readings_search + tot_readings_servo


        # use the particle set from the SERVO data
        f = open( pf_servo )
        p_set_loaded = pkl.load( f )
        f.close()



    # print '\t Positive Reads: %d of %d (%2.1f)' % ( pos_readings,
    #                                                 tot_readings,
    #                                                 100.0 * pos_readings / tot_readings )

    # maxw = np.max( p_set_loaded[:,2] )
    # minw = np.min( p_set_loaded[:,2] )
    # w_normed = 1.0 * ( p_set_loaded[:,2] - minw ) / ( maxw - minw )

    # # Only keep particles in the top 98% of likelihoods (250 / 255).  This
    # # makes the selection only consider locations where the probability
    # # mass is actually located. (These are the ones displayed in the
    # # screen captures!)

    # p_set = p_set_loaded[ np.where( w_normed > 0.02 )[0] ]
    # print 'Shape: ', p_set.shape

    # # print 'p_set size (pre-cap): ', p_set.shape
    # # p_set_precap = np.copy( p_set )
    # # p_set = p_set[np.argsort(p_set[:,2])][:1000] # Cap the total number: keep at most the top 1000
    # # print 'p_set size (pre-cap): ', p_set.shape

    p_set = np.copy( p_set_loaded )
    p_set = p_set[ np.argsort( p_set[:,2] )[::-1] ] # sort by decreasing weight
    w_norm = p_set[:,2] / np.sum( p_set[:,2] )
    w_cum = np.cumsum( w_norm )


    # ONLY KEEP top 8000 particles (computation)
    w_mass =  w_cum[:8000][-1] * 100.0 # Ratio of mass in p_set to total:
    p_set = p_set[:8000] # only keep top 8000 for computational reasons!

    # # ONLY KEEP particles that are in top 98% of normalized values. (these are the ones displayed)
    # maxw = np.max( p_set[:,2] )
    # minw = np.min( p_set[:,2] )
    # w_scaled = 1.0 * ( p_set[:,2] - minw ) / ( maxw - minw )
    # p_set = p_set[ np.where( w_scaled > 0.02 )[0] ]
    # p_set = p_set[:8000] # Only keep top 8000 max
    # w_mass = w_cum[ p_set.shape[0] ]
    # print 'p_set size (pre-cap): ', p_set.shape
    # print w_mass
    

    # print '\tShape: ', p_set.shape

    pf_costmap = '/home/travis/svn/robot1/src/projects/rfid_pf/src/rfid_pf/pf_costmap.pkl'

    f = open( pf_costmap )
    costmap = pkl.load( f )
    f.close()

    cm = costmap[ np.where( costmap[:,2] )[0] ]  # Locations where the robot can be located

    # Determine the score for each possible xy robot location

    def score_loc( xy ):
        # The xy location under consideration for the robot
        mag = np.sqrt( np.sum( np.power( p_set[:,0:2] - xy, 2.0 ), axis = 1))  # || dist ||
        score = mag * p_set[:,2] #  || dist || * w's
        return np.sum( score )


    # Compute all the scores for possible robot locations
    t0 = time.time()
    pos_scores = [ score_loc( i[0:2] ) for i in cm ]
    dt = time.time() - t0

    # print '\tScore computations per second: ', len( pos_scores ) * 1.0 / dt

    best_ind = np.argmin( pos_scores )
    best_pos = cm[ best_ind ][0:2]


    # # Calculate the angle that is the mean
    # # Now that we have best_pos, we need to find the best orientation.

    # def score_orient( xyw, best_pos ): # returns 1x2
    #     # xyw is 1x3
    #     dxy = xyw[0:2] - best_pos  # move into best_pose frame (1x2)

    #     dxy_unit = dxy / np.linalg.norm( dxy ) # normalize to unit circle
    #     return dxy_unit * xyw[2] # 1x2;  [x_circ => x / |dxy| * w, y_circ...]

    # so = np.array([ score_orient( i, best_pos ) for i in p_set ])
    # so[ np.where(np.isnan( so )) ] = 0.0  # for positions where the particle is one and the same, the norm is 0.0 so score is nan.

    # x_so = np.sum( so[:,0] )
    # y_so = np.sum( so[:,1] )
    # orient_est = np.arctan2( y_so, x_so )

    # orient_fit = np.sqrt( x_so**2.0 + y_so**2.0 ) / ( np.sum( p_set[:,2] ))

    # Brute force calculate the angle that yields the minimum |dtheta|
    theta_hats = np.linspace( -1.0 * np.pi, np.pi, 360, endpoint = False )
    theta_hats = np.array([ mu.standard_rad( i ) for i in theta_hats ])

    dxy = p_set[:,0:2] - best_pos # put the p_set into the best_pos frame!
    pset_thetas = np.arctan2( dxy[:,1], dxy[:,0] )
    pset_thetas = np.array([ mu.standard_rad( i ) for i in pset_thetas ])

    pset_w_normed = p_set[:,2] / np.sum( p_set[:,2] )

    def exp_err( th ):
        errs = np.abs([ mu.standard_rad( i ) for i in th - pset_thetas ])
        weighted_errs = pset_w_normed * errs
        mean_we = np.mean( weighted_errs )
        return mean_we
        
    theta_hats_res = np.array([ exp_err( i ) for i in theta_hats ])
    # rrr = theta_hats
    # res = theta_hats_res

    orient_est_ind = np.argmin( theta_hats_res )
    orient_est = theta_hats[ orient_est_ind ]

    # Compute errors:
    dxy = np.linalg.norm( best_pos - loc_pos[0:2] )
    true_theta = np.arctan2( loc_pos[1] - best_pos[1],  # y / x
                             loc_pos[0] - best_pos[0] )
    dtheta = mu.standard_rad( orient_est - true_theta )

    res = { 'loc': loc,
            'obj_num': obj_num,
            'trial_num': trial_num,
            'pos_readings': pos_readings,
            'tot_readings':tot_readings,
            'best_pos': best_pos,
            'orient_est': orient_est,
            'dxy': dxy,
            'dtheta': dtheta,
            'servo_yn': servo_yn,
            'other': { 'w_mass': w_mass,  # specific to pf
                       'theta_hats': theta_hats,
                       'theta_hats_res': theta_hats_res,
                       'p_set': p_set }}
    
    return True, res, np.copy( p_set )  # Was_reads?, results dict, particles (for display)



def MAIN_PROCESS( trial_num, obj_num, servo_yn, screen_cap = False ):
    print 'In MP: ', trial_num, obj_num, servo_yn, screen_cap
    # best_pos, orient_est, p_set = process_trialobj( 4, 3, True )
    was_reads, res, p_set = process_trialobj( trial_num, obj_num, servo_yn )

    print 'RESULTS'
    pprint( res )

    # save results:

    if screen_cap:
        f = open( 'Obj%d_Trial%d_Servo%d_pf_results.pkl' % (obj_num, trial_num, int( servo_yn )), 'w')
        pkl.dump( res, f )
        f.close()

    # Make screen capture.

    if not was_reads:  # Skip step if no reads.
        return

    best_pos = res[ 'best_pos' ]
    orient_est = res[ 'orient_est' ]

    pub_mark = rospy.Publisher( '/tag_poses', vm.Marker )

    if servo_yn:
        c_tm = [0./255, 205./255, 255./255, 1.0] # rgba
    else:
        c_tm = [255./255, 123./255, 1./255, 1.0] # rgba

    tm = vsm( np.matrix([ best_pos[0],best_pos[1], 0.0 ]).T,
              np.matrix(tft.quaternion_from_euler( 0.0, 0.0, orient_est )).T,
              'arrow', '/map',
              scale = [0.5, 1.0, 1.0],
              color = c_tm, 
              duration = 50.0,
              m_id = 2*p_set.shape[0] + 1 )

    def pub_tm( ):
        tm.header.stamp = rospy.Time.now()
        for i in xrange( 10 ):
            pub_mark.publish( tm )
            rospy.sleep( 0.3 )

    print 'Click on RVIZ!'
    time.sleep( 3 )

    pub_tm()
    pub_tm()
    display_particles.display_trialobj( trial_num, obj_num, servo_yn, screen_cap = screen_cap )
    print 'Done.\n\n\n'

    return






if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('--trial', action='store', type='int', dest='trial',
                 help='trial number (0-8)')
    p.add_option('--obj', action='store', type='int', dest='obj',
                 help='object number (0-8)')
    p.add_option('--servo', action='store_true', dest='servo',
                 help='Use combined search and servo?', default = False)
    opt, args = p.parse_args()

    obj_num = opt.obj
    trial_num = opt.trial
    servo_yn = opt.servo
    
    rospy.init_node( 'goober' )


    if trial_num < 9:
        while not rospy.is_shutdown():
            print 'Publishing.'
            MAIN_PROCESS( trial_num, obj_num, servo_yn )
    else:
        # for i in range( 0, 1 ): # trial
        #     for j in range( 4, 5 ): # obj
        for i in range( 0, 9 ): # trial
            for j in range( 0, 9 ): # obj
                for s in [False, True]: # servo_yn
                    MAIN_PROCESS( i, j, s, screen_cap = True )
