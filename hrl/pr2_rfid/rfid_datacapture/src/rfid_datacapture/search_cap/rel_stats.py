#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('tf')
roslib.load_manifest('rfid_pf')
import rospy

import rfid_datacapture.math_util as mu
import sm_aware_home_explore as ahe
import glob
import yaml
import tf
import tf.transformations as tft
import json
import numpy as np, math
import cPickle as pkl
import template
import rfid_pf.pf_stats as pfs

res = []

def process_trialobj( trial_num, obj_num, servo_yn ):
    obj_name = ahe.tdb[ obj_num ][0]
    tname = obj_name.replace( ' ', '' )

    loc = (trial_num + obj_num) % 9
    loc_name = ahe.pts[loc][0]
    loc_pos  = np.array(ahe.pts[loc][1])  # Tag ground-truth location

    print 'Trial %d with Object %d (%s) at Position %d (%s)' % (trial_num, obj_num, obj_name, loc, loc_name)

    fname = 'search_aware_home/woot_150_'+str(trial_num)+'_reads.pkl'
    f = open( fname, 'r' )
    summary_search = pkl.load( f )
    f.close()

    pos_readings_search = sum([ True for p in summary_search if p.read.rssi != -1 and p.read.tagID == obj_name ])
    tot_readings_search = len( summary_search )

    search_positions_fname = 'search_aware_home/woot_150_' + str(trial_num) + '_tag_' + tname + '.yaml'
    servo_positions_fname = 'search_aware_home/woot_150_' + str(trial_num) + '_tag_' + tname + '_end.txt'
    servo_fname = 'search_aware_home/woot_150_' + str(trial_num) + '_tag_' + tname + '_servo.pkl'


    if pos_readings_search == 0:
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

    # All fnames should be defined if we got here!

    f = open( search_positions_fname )
    y = yaml.load( f )
    f.close()

    # "Best" Location determined by search
    efq = tft.euler_from_quaternion
    search_theta = efq( [ y['pose']['orientation']['x'],
                          y['pose']['orientation']['y'],
                          y['pose']['orientation']['z'],
                          y['pose']['orientation']['w'] ])[-1]

    search_pos = np.array([ y['pose']['position']['x'],
                            y['pose']['position']['y'],
                            y['pose']['position']['z'] ])

    search_true_theta = np.arctan2( loc_pos[1] - search_pos[1], # y / x
                                    loc_pos[0] - search_pos[0] )
    search_theta_diff = mu.standard_rad( search_theta - search_true_theta )
    # search_pos_diff = np.linalg.norm( search_pos - loc_pos )
    search_pos_diff = np.linalg.norm( search_pos[0:2] - loc_pos[0:2] )

    # Should we return the search only results?
    if not servo_yn:
        res = { 'loc': loc,
                'obj_num': obj_num,
                'trial_num': trial_num,
                'pos_readings': pos_readings_search,
                'tot_readings':tot_readings_search,
                'best_pos': search_pos,
                'orient_est': search_theta,
                'dxy': search_pos_diff,
                'dtheta': search_theta_diff,
                'servo_yn': servo_yn }
        return True, res, None

    # Else... compute the SEARCH PLUS SERVO results

    # update the stats for pos reads
    f = open( servo_fname, 'r' )
    summary_servo = pkl.load( f )
    f.close()

    pos_readings_servo = sum([ True for p in summary_servo if p.read.rssi != -1 and p.read.tagID == obj_name ])
    tot_readings_servo = len( summary_servo )

    # Location after Servoing
    f = open( servo_positions_fname )
    r = f.readlines()
    f.close()
    # ['At time 1313069718.853\n',
    #   '- Translation: [2.811, 1.711, 0.051]\n',
    #   '- Rotation: in Quaternion [0.003, 0.001, -0.114, 0.993]\n',
    #   '            in RPY [0.005, 0.003, -0.229]\n',
    #   'At time 1313069719.853\n',
    #   '- Translation: [2.811, 1.711, 0.051]\n',
    #   '- Rotation: in Quaternion [0.003, 0.001, -0.114, 0.993]\n',
    #   '            in RPY [0.005, 0.002, -0.229]\n']

    rpy = r[-1].find('RPY')+3
    servo_theta = json.loads( r[-1][rpy:] )[-1]

    tion = r[-3].find('tion:')+5
    servo_pos = np.array(json.loads( r[-3][tion:] ))

    servo_true_theta = np.arctan2( loc_pos[1] - servo_pos[1], # y / x
                                   loc_pos[0] - servo_pos[0] )
    servo_theta_diff = mu.standard_rad( servo_theta - servo_true_theta )
    # servo_pos_diff = np.linalg.norm( servo_pos - loc_pos )
    servo_pos_diff = np.linalg.norm( servo_pos[0:2] - loc_pos[0:2] )

    # print '\t Post-Servo Stats:'
    # print '\t\t Tag-Robot distance err (m): %2.3f' % (servo_pos_diff)
    # print '\t\t Tag-Robot orient err (deg): %2.3f' % (math.degrees(servo_theta_diff))
    # print '\t\t Debug Stats', math.degrees(servo_theta), math.degrees(servo_true_theta), servo_pos, loc_pos

    res = { 'loc': loc,
            'obj_num': obj_num,
            'trial_num': trial_num,
            'pos_readings': pos_readings_search + pos_readings_servo,
            'tot_readings':tot_readings_search + tot_readings_servo,
            'best_pos': servo_pos,
            'orient_est': servo_theta,
            'dxy': servo_pos_diff,
            'dtheta': servo_theta_diff,
            'servo_yn': servo_yn }

    return True, res, None

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
        print 'Publishing.'
        was_reads, res, p_set = process_trialobj( trial_num, obj_num, servo_yn )

        print 'RESULTS'
        pfs.pprint( res )

        f = open( 'Obj%d_Trial%d_Servo%d_rel_results.pkl' % (obj_num, trial_num, int( servo_yn )), 'w')
        pkl.dump( res, f )
        f.close()
            
    else:
        # for i in range( 0, 1 ): # trial
        #     for j in range( 4, 5 ): # obj
        for i in range( 0, 9 ): # trial
            for j in range( 0, 9 ): # obj
                for s in [False, True]: # servo_yn

                    was_reads, res, p_set = process_trialobj( trial_num = i, obj_num = j, servo_yn = s )

                    print 'RESULTS'
                    pfs.pprint( res )

                    f = open( 'Obj%d_Trial%d_Servo%d_rel_results.pkl' % ( j, i, int( s )), 'w')
                    pkl.dump( res, f )
                    f.close()

