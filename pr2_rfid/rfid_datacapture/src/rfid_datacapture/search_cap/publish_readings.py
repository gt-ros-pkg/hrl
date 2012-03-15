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
roslib.load_manifest('visualization_msgs')
import rospy

import cPickle as pkl
import hrl_lib.rutils as ru
import hrl_lib.viz as viz
import sensor_msgs.msg as sm
import numpy as np, math
import sm_aware_home_explore as ahe
import visualization_msgs.msg as vm
import glob
import json
import yaml
import time
import os


def publish_readings( trial, obj, pub_time = 30e3, screen_cap = False ):
    loc = ( trial + obj ) % 9

    fname = 'search_aware_home/woot_150_'+str(trial)+'_reads.pkl'
    
    f = open( fname, 'r' )
    r = pkl.load(f)
    f.close()

    rospy.init_node( 'starter_woot' )

    vsm = viz.single_marker


    # RFID readings
    pos = [ vsm( np.matrix([ p.ps_ant_map.pose.position.x,
                             p.ps_ant_map.pose.position.y,
                             p.ps_ant_map.pose.position.z ]).T,
                 np.matrix([ p.ps_ant_map.pose.orientation.x,
                             p.ps_ant_map.pose.orientation.y,
                             p.ps_ant_map.pose.orientation.z,
                             p.ps_ant_map.pose.orientation.w ]).T,
                 'arrow', '/map',
                 color = [1.0, 0.0, 0.0, 0.8], # rgba,
                 duration = 10.0,
                 m_id = i )
            for i,p in enumerate( r )
            if p.read.rssi != -1 and p.read.tagID == ahe.tdb[obj][0] ]

    neg = [ vsm( np.matrix([ p.ps_ant_map.pose.position.x,
                             p.ps_ant_map.pose.position.y,
                             p.ps_ant_map.pose.position.z ]).T,
                 np.matrix([ p.ps_ant_map.pose.orientation.x,
                             p.ps_ant_map.pose.orientation.y,
                             p.ps_ant_map.pose.orientation.z,
                             p.ps_ant_map.pose.orientation.w ]).T,
                 'arrow', '/map',
                 color = [0.2, 0.2, 0.2, 0.2], # rgba
                 duration = 10.0,
                 m_id = i + len(r) )
            for i,p in enumerate( r )
            if p.read.tagID != ahe.tdb[obj] ] # for no-read or other tag reads

    print 'Pos: ', len(pos), '\nNeg: ', len(neg)


    # Robot Trajectory
    tm = [ vsm( np.matrix([ ahe.pts[loc][1][0],
                            ahe.pts[loc][1][1],
                            ahe.pts[loc][1][2] ]).T,
                np.matrix([ [0.0], [0.0], [0.0], [1.0] ]),
                'sphere', '/map',
                color = [0.0, 1.0, 0.0, 1.0], # rgba
                duration = 10.0,
                m_id = 2*len(r) + 1 )]

    xyz = np.array([ [p.ps_base_map.pose.position.x,
                      p.ps_base_map.pose.position.y,
                      p.ps_base_map.pose.position.z ] for p in r ]).T
    pts = ru.np_to_pointcloud( xyz, '/map' )

    pub_pts = rospy.Publisher( '/robot_traj', sm.PointCloud )
    pub_mark = rospy.Publisher( '/tag_poses', vm.Marker )


    # Search and Servo Positions

    obj_name = ahe.tdb[obj][0]  # tagID
    tname = obj_name.replace( ' ', '' )


    # "Best" Location determined by search
    search_fname = 'search_aware_home/woot_150_' + str(trial) + '_tag_' + tname + '.yaml'
    try:
        f = open( search_fname )
    except:
        return
    
    y = yaml.load( f )
    f.close()

    search = [ vsm( np.matrix([ y['pose']['position']['x'],
                                y['pose']['position']['y'],
                                y['pose']['position']['z'] ]).T,
                    np.matrix([ y['pose']['orientation']['x'],
                                y['pose']['orientation']['y'],
                                y['pose']['orientation']['z'],
                                y['pose']['orientation']['w'] ]).T,
                   'arrow', '/map',
                   scale = [0.5, 1.0, 1.0],
                   color = [255./255, 123./255, 1./255, 1.0], # rgba
                   duration = 10.0,
                   m_id = 2 * len(r) + 2 )]


    # Location after Servoing
    servo_fname = 'search_aware_home/woot_150_' + str(trial) + '_tag_' + tname + '_end.txt'

    try:
        f = open( servo_fname )
    except:
        return
    y = f.readlines()
    f.close()
    # ['At time 1313069718.853\n',
    #   '- Translation: [2.811, 1.711, 0.051]\n',
    #   '- Rotation: in Quaternion [0.003, 0.001, -0.114, 0.993]\n',
    #   '            in RPY [0.005, 0.003, -0.229]\n',
    #   'At time 1313069719.853\n',
    #   '- Translation: [2.811, 1.711, 0.051]\n',
    #   '- Rotation: in Quaternion [0.003, 0.001, -0.114, 0.993]\n',
    #   '            in RPY [0.005, 0.002, -0.229]\n']

    quat = y[-2].find('Quaternion')+10
    quat_list = json.loads( y[-2][quat:] )

    sloc = y[-3].find('tion:')+5
    sloc_list = json.loads( y[-3][sloc:] )


    servo = [ vsm( np.matrix([ sloc_list ]).T,
                   np.matrix([ quat_list ]).T,
                   'arrow', '/map',
                   scale = [0.5, 1.0, 1.0],
                   color = [0./255, 205./255, 255./255, 1.0], # rgba
                   duration = 10.0,
                   m_id = 2 * len(r) + 3 )]


    marks = neg + pos + tm + search + servo
    
    t0 = time.time()
    while time.time() - t0 < pub_time and not rospy.is_shutdown():
        pts.header.stamp = rospy.Time.now()
        pub_pts.publish( pts )
        
        [ pub_mark.publish( x ) for x in marks ]
        rospy.sleep( 1.0 )

    if screen_cap:
        os.system( 'scrot -d 2 -u Obj%d_Trial%d.png' % ( obj, trial ))
        

    print 'Closing down... letting markers expire'
    rospy.sleep( 15 )

        

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    # p.add_option('--fname', action='store', type='string', dest='fname',
    #              help='File name. Should be woot_150_x_reads.pkl', default='')
    p.add_option('--trial', action='store', type='int', dest='trial',
                 help='trial number (0-8)')
    p.add_option('--obj', action='store', type='int', dest='obj',
                 help='object number (0-8)')
    # p.add_option('--loc', action='store', type='int', dest='loc',
    #              help='location number (0-8)')

    opt, args = p.parse_args()

    if opt.trial < 9:
        publish_readings( opt.trial, opt.obj )
    else:
        print 'Click on RVIZ!'
        time.sleep( 3 )
        #for trial in xrange(9):
        for trial in [1]:
            #for obj in xrange(9):
            for obj in [6]:
                print 'Change screen to RVIZ. Starting %d, %d' % (trial, obj)
                publish_readings( trial, obj, 15, screen_cap = True )
    

