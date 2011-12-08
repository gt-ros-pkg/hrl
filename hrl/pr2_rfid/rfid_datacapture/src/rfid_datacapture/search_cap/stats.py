#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('tf')
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


print 'THIS FILE DEPRICATED AND INACCURATE!!!  USE REL_STATS!'


res = []

for i in xrange( 9 ):  # trial
    for j in xrange( 9 ):  # object
        skip = False # (hacky)
        
        obj_num = j
        obj_name = ahe.tdb[j][0]
        tname = obj_name.replace( ' ', '' )

        trial_num = i

        loc = (i + j) % 9
        loc_name = ahe.pts[loc][0]
        loc_pos  = np.array(ahe.pts[loc][1])  # Tag ground-truth location

        print 'Trial %d with Object %d (%s) at Position %d (%s)' % (i, obj_num, obj_name, loc, loc_name)

        fname = 'search_aware_home/woot_150_'+str(i)+'_reads.pkl'
        f = open( fname, 'r' )
        summary = pkl.load( f )
        f.close()

        pos_readings = sum([ True for p in summary if p.read.rssi != -1 and p.read.tagID == obj_name ])
        tot_readings = len( summary )
        print '\t Positive Reads: %d of %d (%2.2f)' % ( pos_readings,
                                                        tot_readings,
                                                        100.0 * pos_readings / tot_readings )


        search_fname = 'search_aware_home/woot_150_' + str(i) + '_tag_' + tname + '.yaml'
        glob_r = glob.glob( search_fname )
        if glob_r == []:
            print '\t No results for this instance.'
            skip = True 
        if len(glob_r) > 1:
            print '\t Multiple results...?! Weirdness.  Skipping.'
            skip = True


        servo_fname = 'search_aware_home/woot_150_' + str(i) + '_tag_' + tname + '_end.txt'
        glob_r = glob.glob( servo_fname )
        if glob_r == []:
            print '\t No results for this instance.'
            skip = True 
        if len(glob_r) > 1:
            print '\t Multiple results...?! Weirdness.  Skipping.'
            skip = True



        if not skip:
            f = open( search_fname )
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
            search_pos_diff = np.linalg.norm( search_pos - loc_pos )
        
            print '\t Post-Search Stats:'
            print '\t\t Tag-Robot distance err (m): %2.3f' % (search_pos_diff)
            print '\t\t Tag-Robot orient err (deg): %2.3f' % (math.degrees(search_theta_diff))
            print '\t\t Debug Stats', math.degrees(search_theta), math.degrees(search_true_theta), search_pos, loc_pos

            # Location after Servoing
            f = open( servo_fname )
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
            servo_pos_diff = np.linalg.norm( servo_pos - loc_pos )

            print '\t Post-Servo Stats:'
            print '\t\t Tag-Robot distance err (m): %2.3f' % (servo_pos_diff)
            print '\t\t Tag-Robot orient err (deg): %2.3f' % (math.degrees(servo_theta_diff))
            print '\t\t Debug Stats', math.degrees(servo_theta), math.degrees(servo_true_theta), servo_pos, loc_pos

            res.append( [ loc, obj_num, i, pos_readings, tot_readings, servo_pos_diff, math.degrees(servo_theta_diff) ] )

        else:
            res.append( [ loc, obj_num, i, pos_readings, tot_readings, '--', '--' ] )
            
        print '\t Done.\n\n'

res.sort()
print '\n\nRESULTS SORTED\n\n'
def pprint(r):
    print 'Location %d, Object %d, Trial %d' % (r[0], r[1], r[2])
    if r[3] > 0:
        print '\tPos Reads:      %d' % (r[3])
        print '\tTot Reads:      %d' % (r[4])
        print '\tPercent Reads:  %2.3f' % (r[3]*100.0/r[4])
        print '\tDist Err (m):   %2.3f' % (r[5])
        print '\tAng Err (deg):  %2.3f' % (r[6])
    else:
        print '\tPos Reads:      %d' % (r[3])
        print '\tTot Reads:      %d' % (r[4])
        print '\tPercent Reads:  %2.3f' % (r[3]*100.0/r[4])
        print '\tDist Err (m):   ----'
        print '\tAng Err (deg):  ----'

[ pprint(r) for r in res ]



print '\n\n#########  OUTPUTTING TEX TABLES from template.py #########\n\n'

def delta_xy( r ):
    if r[3] > 0:
        return '%2.3f m' % (r[5])
    else:
        return '$--$'

def delta_theta( r ):
    if r[3] > 0:
        return '%2.1f$^o$' % (r[6])
    else:
        return '$--$'


for i in xrange( 9 ):
    io = [ r for r in res if r[0] == i ]


    if len(io) != 9:
        print 'BIG PROBLEM.  IO != 9'
        exit()

    args = []
    # Top Table
    args += [ '%d / %d (%2.1f\\%%)' % (r[3], r[4], r[3] * 100.0 / r[4]) for r in io ][0:4]
    args += [ delta_xy(r) for r in io ][0:4]
    args += [ delta_theta(r) for r in io ][0:4]
    # Bottom Table
    args += [ '%d / %d (%2.1f\\%%)' % (r[3], r[4], r[3] * 100.0 / r[4]) for r in io ][4:]
    args += [ delta_xy(r) for r in io ][4:]
    args += [ delta_theta(r) for r in io ][4:]
    # Overall Table
    args += [ io[0][0] ] # Title
    args += [ np.mean( [ r[3] for r in io ] ),
              np.std(  [ r[3] for r in io ] ) ] # Reads
    args += [ np.mean( [ r[5] for r in io if r[3] > 0 ] ),  
              np.std(  [ r[5] for r in io if r[3] > 0 ] )] # Distances (Only meaningful for detected tags)
    args += [ np.mean( np.abs([ r[6] for r in io if r[3] > 0 ]) ),  
              np.std(  np.abs([ r[6] for r in io if r[3] > 0 ]) )] # |angle| (Only meaningful for detected tags)

    # Caption
    args += [ io[0][0], io[0][0] ]

    io_templated = template.x % tuple( args )
    f = open('rfid_search_loc%d_table.tex' % (i),'w')
    f.write( io_templated )
    f.close()


print 'THIS FILE DEPRICATED AND INACCURATE!!!  USE REL_STATS!'
