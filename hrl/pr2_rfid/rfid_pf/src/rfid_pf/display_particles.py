#!/usr/bin/python

import rfid_model
import lib_pfilter

import roslib
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('pfilter')
import rospy

import visualization_msgs.msg as vm
import hrl_lib.transforms as tr
import hrl_lib.viz as viz
vsm = viz.single_marker
import pfilter.pfilter as pfilter

import random as rd
import numpy as np, math
import pylab as pl
import time
import cPickle as pkl
import os
import glob

class DisplayParticles:
    def __init__( self, pub_topic = '/particles' ):
        self.m = None
        self.pub_mark = rospy.Publisher( pub_topic, vm.Marker )
        self.mid = 0

        try:
            rospy.init_node('display_particles')
        except:
            rospy.logout( 'DisplayParticles: Node already initialized' )
            pass

    def create_mark( self, p, c = [1.0, 0.0, 0.0, 0.8], mid = None ):
        if mid == None:
            self.mid += 1
            mid = self.mid
            
        m =  vsm( np.matrix([ p[0], p[1], 0.0 ]).T,
                  np.matrix([ 0.0, 0.0, 0.0, 1.0 ]).T,
                  'sphere', '/map',
                  scale = [0.025, 0.025, 0.025],
                  color = [1.0, 0.0, 0.0, 0.3], # rgba,
                  duration = 10.0,
                  m_id = mid )
        m.header.stamp = rospy.Time.now()
        return m
    
    
    def update( self, particles ):
        xyz = np.column_stack([ particles[:,0:2], np.zeros( particles.shape[0] )]).T
        w = particles[:,2] # becomes 1D
        # print w
        wmin = np.min( w )
        wmax = np.max( w )
        # import pdb
        # pdb.set_trace()

        if wmin == wmax:
            colors = np.row_stack([ np.ones( particles.shape[0] ),
                                    np.zeros( particles.shape[0] ),
                                    np.zeros( particles.shape[0] ),
                                    np.ones( particles.shape[0] ) ])
        else:
            colors = np.array([ pl.cm.jet( int( 1.0 * ( wi - wmin ) / (wmax - wmin) * 255.0 ))
                                for wi in w ]).T
        m = viz.list_marker( xyz, colors, [0.025, 0.025, 0.025], 'points', '/map', 300 )
        m.header.stamp = rospy.Time.now()
        for i in xrange( 10 ):
            self.pub_mark.publish( m )
            rospy.sleep( 0.2 )

        return


    def update2( self, particles ):
        xyz = np.column_stack([ particles[:,0:2], np.zeros( particles.shape[0] )]).T
        w = particles[:,2] # becomes 1D
        # print w
        wmin = np.min( w )
        wmax = np.max( w )
        # import pdb
        # pdb.set_trace()

        if wmin == wmax:
            colors = np.row_stack([ np.ones( particles.shape[0] ),
                                    np.zeros( particles.shape[0] ),
                                    np.zeros( particles.shape[0] ),
                                    np.ones( particles.shape[0] ) ])
            iv = np.ones( len( w )).tolist()
        else:
            iv = [ int( 1.0 * ( wi - wmin ) / (wmax - wmin) * 255.0 ) for wi in w ]
            colors = np.array([ pl.cm.jet( ivi ) for ivi in iv ]).T
            # colors[3] *= 0.3

        print np.array(iv)
        ind = np.where( np.array(iv) > 5 )[0]
        aind = np.argsort( w[ind] )  # sort them so that some come to top.
        
        m = viz.list_marker( xyz[:,ind][:,aind], colors[:,ind][:,aind], [0.05, 0.05, 0.025], 'points', '/map', 30 )
        m.header.stamp = rospy.Time.now()
        for i in xrange( 10 ):
            self.pub_mark.publish( m )
            rospy.sleep( 0.2 )

        return





def display_trialobj( trial_num, obj_num, servo_yn, screen_cap = False ):
    try:
        rospy.init_node( 'ros_pf' )
    except:
        print 'display_trialobj: node already initialized'
        pass
    

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

    glob_r = glob.glob( servo_fname )
    if glob_r == []:
        print '\t No results for this instance.\n\n'
        return
    if len(glob_r) > 1:
        print '\t Multiple results...?! Weirdness.  Skipping.'
        return

    if servo_yn:
        f = open( pf_servo, 'r' )
        p_set = pkl.load( f )
        f.close()
    else:
        f = open( pf_search, 'r' )
        p_set = pkl.load( f )
        f.close()

    dp = DisplayParticles()

    pub_mark = rospy.Publisher( '/tag_poses', vm.Marker )

    tm = vsm( np.matrix([ pts[loc][1][0],
                            pts[loc][1][1],
                            pts[loc][1][2] ]).T,
                np.matrix([ [0.0], [0.0], [0.0], [1.0] ]),
                'sphere', '/map',
                color = [0.0, 1.0, 0.0, 1.0], # rgba
                duration = 150.0,
                m_id = 2*p_set.shape[0] + 1 )

    def pub_tm():
        tm.header.stamp = rospy.Time.now()
        for i in xrange( 10 ):
            pub_mark.publish( tm )
            rospy.sleep( 0.3 )

    def update_display():
        print 'UPDATING DISPLAY... '
        pub_tm()
        pub_tm()
        dp.update2( p_set )
        rospy.sleep( 0.3 )
        print 'DONE.'


    print 'UPDATING'
    update_display()
    rospy.sleep( 1.0 )

    if screen_cap:
        os.system( 'scrot -d 2 -u Obj%d_Trial%d_Servo%d_pf_results.png' % ( obj_num, trial_num, int(servo_yn) ))

    rospy.sleep( 2.0 )
    print 'DERP'

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
    
    if trial_num < 9:
        while not rospy.is_shutdown():
            display_trialobj( trial_num, obj_num, servo_yn )
    else:
        print 'Click on RVIZ!'
        time.sleep( 3 )

        for trial_num in range( 0, 9 ):
            for obj_num in range( 0, 9 ):
                for servo_yn in [False, True]:
                    print 'Calling display trialobj: ', trial_num, obj_num, servo_yn
                    display_trialobj( trial_num, obj_num, servo_yn, screen_cap = True )
                    print 'Done.\n\n\n'
