#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('tf')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('rosbag')
roslib.load_manifest('visualization_msgs')
import rospy

import cPickle as pkl
import hrl_lib.rutils as ru
import hrl_lib.viz as viz
import sensor_msgs.msg as sm
import numpy as np, math
import sm_aware_home_explore as ahe
import rosbag
import visualization_msgs.msg as vm
from hrl_lib.cmd_process import CmdProcess
import glob
import json
import yaml
import time
import os

def publish_robotpose( trial, obj, pub_time = 30e3, screen_cap = False ):
    print 'Change screen to RVIZ. Starting %d, %d' % (trial, obj)
    loc = ( trial + obj ) % 9

    obj_name = ahe.tdb[obj][0]
    tname = obj_name.replace( ' ', '' )

    # woot_150_6_tag_BlueHairBrus_headpost.bag
    fname = 'search_aware_home/woot_150_'+str(trial)+'_tag_'+tname+'_headpost.bag'

    # Start the new bagplay
    bp = bagplay( fname )
    bp.run()

    while not bp.is_finished():
        try:
            rospy.init_node( 'markpub' )
            pub_mark = rospy.Publisher( '/robot_pose', vm.Marker )
        except:
            print 'Init Failure.'

            
        # Publish the robot marker

        vsm = viz.single_marker
        mark = [ vsm( np.matrix([ 0.0, 0.0, 0.0 ]).T,
                      np.matrix([ 0.0, 0.0, 0.0, 1.0 ]).T,
                      'cube', '/base_link',
                      scale = [0.65, 0.65, 0.001],
                      color = [158./255, 86./255, 192./255, 0.9], # rgba,
                      duration = 30.0,
                      m_id = 20000 )]

        [ pub_mark.publish( x ) for x in mark ]


        sim_safe_sleep( 1.0 )  # Cannot use rostime, since it will stall when bag stops

    # Screenshot!

    if screen_cap:
        os.system( 'scrot -d 2 -u Obj%d_Trial%d_RobotView.png' % ( obj, trial ))
        
    # Let markers expire
    print 'Waiting for markers and points to expire'
    t0 = time.time()
    t_sleep = 60.0
    while time.time() - t0 < t_sleep and not rospy.is_shutdown():
        if int(time.time() - t0) % 5 == 0:
            print 'Time left: %d' % (t_sleep - int(time.time() - t0))
        time.sleep( 1.0 )

    return


def sim_safe_sleep( dur, real_time_sleep = 0.05 ):
    t0 = rospy.Time.now().to_sec()
    ct = rospy.Time.now().to_sec()
    while True:
        if ct - t0 >= dur:
            break

        time.sleep( real_time_sleep )
        nt = rospy.Time.now().to_sec()

        if nt == ct: # rostime will stop when bag not playing -- exit immediately.
            break  
        
        ct = nt
    return

def bagplay( fname ):
    # to use:
    #   bp = bagplay( my_file_name )
    #   bp.run() # starts the execution
    #   while not bp.is_finished():
    #       rospy.sleep( 0.5 )
    #   bp.kill() # not necessary
    cmd = 'rosbag play --clock ' + fname + ' -r 2.0 -q'
    rospy.logout( 'Launching bag file: %s' % fname )
    return CmdProcess( cmd.split() )


def order_by_rostime( dat ):
    # dat is [[trial, obj], ... ]
    # I'm too lazy to figure out how to reset time and prevent "TF_OLD_DATA" errors / warnings.
    #   Instead, we're just going to order the bag playback in wall-clock order.

    def build_fname( t,o ):
        # woot_150_6_tag_BlueHairBrus_headpost.bag
        fname = 'search_aware_home/woot_150_'+str(t)+'_tag_'+ahe.tdb[o][0].replace( ' ', '' )+'_headpost.bag'
        return fname
        

    dat = [ [t,o] + [ build_fname(t,o) ] for t,o in dat ]
    dat = [ d for d in dat if glob.glob( d[-1] ) != [] ]

    rospy.logout( 'Ordering the bagfiles in increasing order of start time.' )
    def gettime( fname ):
        print fname
        # returns the timestamp of the first message
        b = rosbag.Bag( fname )
        msg = b.read_messages().next()
        tt = msg[-1].to_sec()
        b.close()
        return tt

    start_times = [ gettime( d[-1] ) for d in dat ]
    rospy.logout( 'Done ordering.' )
    
    return [ [dat[ind][0],dat[ind][1]] for ind in np.argsort( start_times ) ]


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    # p.add_option('--fname', action='store', type='string', dest='fname',
    #              help='File name. Should be woot_150_x_reads.pkl', default='')
    p.add_option('--trial', action='store', type='int', dest='trial',
                 help='trial number (0-8)')
    p.add_option('--obj', action='store', type='int', dest='obj',
                 help='object number (0-8)')
    p.add_option('--sc', action='store_true', dest='sc',
                 help='Take screenshot', default=False)
    # p.add_option('--loc', action='store', type='int', dest='loc',
    #              help='location number (0-8)')

    opt, args = p.parse_args()

    if opt.trial < 9:
        publish_robotpose( opt.trial, opt.obj, screen_cap = opt.sc )
    else:
        print 'Click on RVIZ!'
        time.sleep( 3 )

        #X,Y = np.meshgrid( range(0,9), range(0,9) )
        X,Y = np.meshgrid( range(0,9), range(0,9) )
        trial_obj = zip( Y.flatten(), X.flatten() )
        [ publish_robotpose( trial, obj, 15, screen_cap = True )
          for trial, obj in order_by_rostime(trial_obj) ]
    

