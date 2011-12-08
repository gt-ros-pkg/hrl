#! /usr/bin/python

# The goal is to combine all the results of Obj/Trial/Servo results into
# one big datastructure for easy generation of tables and figures for the dissertation!

import roslib
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_pf')
import rospy

import cPickle as pkl
import numpy as np, math

import rfid_pf.pf_stats as pfs
import rfid_pf.stats_best_uniform as sbu
import scipy.stats as ss


def process_summary( cr_filt='pf', servo_yn=False, print_str='PARTICLE FILTER:', combined_results=None ):
    # PF overall stats (SEARCH):
    # servo_yn = False

    # Only take results for this setting (eg servo state)
    
    # filt = [ res for res in combined_results['pf'] if res['servo_yn'] == servo_yn ]
    filt = [ res for res in combined_results[cr_filt] if res['servo_yn'] == servo_yn ]


    # Refine to eliminate "no read" scenarios
    filt = [ f for f in filt if f['dxy'].__class__ != ''.__class__ ]
    

    dist_abs = []
    dist_from_opt = []
    ang_error = []

    for res in filt:
        # The distance from the tag
        dist_abs.append( res['dxy'] )

        # The difference in distance from tag for this result versus the best possible achievable result
        opt_for_loc = [b for b in combined_results['best'] if b['loc'] == res['loc']][0] # it's unique
        dist_from_opt.append( np.abs( res['dxy'] - opt_for_loc['best_dxy'] ))

        # | Angular Error | (deg)
        ang_error.append( math.degrees( np.abs(res['dtheta']) ))

    if servo_yn:
        servo_state = 'search PLUS servo'
    else:
        servo_state = 'search ONLY'
        
    print print_str + ' ' + servo_state
    print 'Average Absolute Distance From Tag (meters):\n\t%2.3f (%2.3f)\n' % ( np.mean( dist_abs ),
                                                                                np.std(  dist_abs ))

    print 'Difference between Best Possible Distance and ',
    print 'PF-estimated Distance From Tag (meters):\n\t%2.3f (%2.3f)\n' % ( np.mean( dist_from_opt ),
                                                                            np.std(  dist_from_opt ))

    print 'Average Angular Error Magnitude (degrees):\n\t%3.1f (%3.1f)\n' % ( np.mean( ang_error ),
                                                                              np.std(  ang_error ))

    print '\n\n'
    return dist_abs, dist_from_opt, ang_error


def process_uniform_summary( combined_results = None):

    dxy_means =    [ i['dxy_mean'] for i in combined_results['uniform'] ]
    dtheta_means = [ i['dtheta_mean'] for i in combined_results['uniform'] ]
        
    print 'Average Absolute Distance From Tag (meters):\n\t%2.3f (%2.3f)\n' % ( np.mean( dist_abs ),
                                                                                np.std(  dist_abs ))

    print 'Difference between Best Possible Distance and ',
    print 'PF-estimated Distance From Tag (meters):\n\t%2.3f (%2.3f)\n' % ( np.mean( dist_from_opt ),
                                                                            np.std(  dist_from_opt ))

    print 'Average Angular Error Magnitude (degrees):\n\t%3.1f (%3.1f)\n' % ( np.mean( ang_error ),
                                                                              np.std(  ang_error ))

    print '\n\n'


def global_stats():
    # Get the stupid depracation warning out of the way.
    ss.ttest_ind( range(30), range(30))

    print '\n\n\n'

    f = open( 'summarize_results.pkl', 'r' )
    combined_results = pkl.load( f )
    f.close()


    pf_dist_abs, pf_dist_from_opt, pf_ang_error = process_summary( 'pf', True, 'PARTICLE FILTER:', combined_results )
    rel_dist_abs, rel_dist_from_opt, rel_ang_error = process_summary( 'rel', True, 'RELATIVE METHODS:', combined_results )

    def ttest_significance( p_value ):
        if p_value < 0.05:
            return 'YES'
        else:
            return 'NO'

    print 'T-TEST RESUTLS: '
    print 'Absolute Distance:'
    tvalue, pvalue = ss.ttest_ind( pf_dist_abs, rel_dist_abs )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))

    print 'Difference between Best Dist:'
    tvalue, pvalue = ss.ttest_ind( pf_dist_from_opt, rel_dist_from_opt )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))

    print 'Angular Error:'
    tvalue, pvalue = ss.ttest_ind( pf_ang_error, rel_ang_error )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))

    print '...\n\n'
    pf_dist_abs, pf_dist_from_opt, pf_ang_error = process_summary( 'pf', False, 'PARTICLE FILTER:', combined_results )
    rel_dist_abs, rel_dist_from_opt, rel_ang_error = process_summary( 'rel', False, 'RELATIVE METHODS:', combined_results )

    print 'T-TEST RESUTLS: '
    print 'Absolute Distance:'
    tvalue, pvalue = ss.ttest_ind( pf_dist_abs, rel_dist_abs )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))

    print 'Difference between Best Dist:'
    tvalue, pvalue = ss.ttest_ind( pf_dist_from_opt, rel_dist_from_opt )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))

    print 'Angular Error:'
    tvalue, pvalue = ss.ttest_ind( pf_ang_error, rel_ang_error )
    print '\tp-value: %1.7f ==> Significance: %s' % ( pvalue, ttest_significance( pvalue ))



if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('--recalc', action='store_true', dest='recalc',
                 help='Recalculate the summary pickle files', default = False)
    opt, args = p.parse_args()


    if not opt.recalc:
        global_stats()


    if opt.recalc:
        combined_results = { 'rel':     [],
                             'pf' :     [],
                             'best':    [],
                             'uniform': [] }

        combined_results_full = { 'rel':     [],
                                  'pf' :     [],
                                  'best':    [],
                                  'uniform': [] }

        # Add the results (trial and obj) of the actual estimators (relative and pf)
        for trial_num in range( 0, 9 ): # trial
            for obj_num in range( 0, 9 ): # obj
                for servo_yn in [False, True]: # servo_yn

                    print 'Adding Trial %d, Obj %d, Servo %d' % (trial_num, obj_num, servo_yn)

                    # Add the relative sensing results
                    rel_result_fname = '/home/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/'
                    rel_result_fname += 'Obj%d_Trial%d_Servo%d_rel_results.pkl' % ( obj_num, trial_num, int( servo_yn ))

                    try:
                        f = open( rel_result_fname, 'r' )
                        rel_result = pkl.load( f )
                        f.close()

                        combined_results_full['rel'].append( dict(rel_result) )  # Deep copy
                        rel_result['other'] = None  # Save space!
                        combined_results['rel'].append( rel_result )
                    except:
                        print 'File not found (skipping): ', rel_result_fname
                        print 'FIX THIS LATER!\n'
                        pass


                    # Add the pf results
                    pf_result_fname = '/home/travis/svn/robot1/src/projects/rfid_pf/src/rfid_pf/'
                    pf_result_fname += 'Obj%d_Trial%d_Servo%d_pf_results.pkl' % ( obj_num, trial_num, int( servo_yn ))

                    try:
                        f = open( pf_result_fname, 'r' )
                        pf_result = pkl.load( f )
                        f.close()

                        combined_results_full['pf'].append( dict(pf_result) )  # Deep copy
                        pf_result['other'] = None  # Save space
                        combined_results['pf'].append( pf_result )
                    except:
                        print 'File not found (skipping): ', pf_result_fname
                        print 'FIX THIS LATER!\n'
                        pass


        # Add the results (location by location)
        for loc_num in range( 0, 9 ):

            # Stats for the "best"
            best_pos, best_dxy = sbu.best_location( loc_num )
            res = { 'loc':loc_num,
                    'best_pos':best_pos,
                    'best_dxy':best_dxy }
            combined_results['best'].append( res )
            combined_results_full['best'].append( dict(res) )  # Deep copy


            # Stats for Uniform
            dxy_mean, dxy_std, dtheta_mean, dtheta_std = sbu.uniform_predict( loc_num )
            res = { 'loc':loc_num,
                    'dxy_mean':dxy_mean,
                    'dxy_std':dxy_std,
                    'dtheta_mean':dtheta_mean,
                    'dtheta_std':dtheta_std }
            combined_results['uniform'].append( res )
            combined_results_full['uniform'].append( dict(res) )  # Deep copy

        f = open( 'summarize_results.pkl', 'w' )
        pkl.dump( combined_results, f )
        f.close()

        f = open( 'summarize_results_full.pkl', 'w' )
        pkl.dump( combined_results_full, f )
        f.close()

        exit()

        # DONE WITH RECALC

    


