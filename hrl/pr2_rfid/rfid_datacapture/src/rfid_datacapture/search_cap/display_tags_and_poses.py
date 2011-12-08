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
import tf
import tf.transformations as tft
import glob
import json
import yaml
import time
import os

rospy.init_node( 'derp' )
vsm = viz.single_marker
m_id = 0
# Tags
# tag_pts = np.array([ ahe.pts[k][1] for k in ahe.pts.keys() ]).T
# tm = [ viz.single_marker( tag_pts[:,i].reshape([3,1]),
#                           np.matrix([ [0.0], [0.0], [0.0], [1.0] ]),
#                           'sphere', '/map',
#                           color=[0.0, 1.0, 0.0, 1.0],
#                           m_id=i ) for i in xrange( tag_pts.shape[1] )]
# Results:
# 'Obj5_Trial8_Servo1_rel_results.pkl'
# {'best_pos': array([-2.165,  0.979,  0.055]),
#  'dtheta': 0.041245071031409619,
#  'dxy': 1.1365249667297239,
#  'loc': 7,
#  'obj_num': 5,
#  'orient_est': 2.8607549289685905,
#  'pos_readings': 717,
#  'servo_yn': True,
#  'tot_readings': 3580,
#  'trial_num': 2}


fnames = glob.glob('Obj[0-9]_Trial[0-9]_Servo1_rel_results.pkl')
def load( fn ):
    f = open( fn )
    dat = pkl.load( f )
    f.close()
    return dat

d = [ load( f ) for f in fnames ]
d = [ di for di in d if di['pos_readings'] > 0 ]

def arrow( m_id, res_dict, color ):
    m = vsm( np.matrix([ res_dict['best_pos'] ]).T,
             np.matrix([ tft.quaternion_from_euler( 0.0,
                                                    0.0,
                                                    res_dict['orient_est']) ]).T,
             'arrow', '/map',
             scale = [0.5, 1.0, 1.0],
             color = color, # rgba
             duration = 10.0,
             m_id = m_id )
    # color = [0./255, 205./255, 255./255, 1.0], # rgba

    return m

pub_mark = rospy.Publisher( '/tag_poses', vm.Marker )

tag_spheres = []
servo_arrows = []

def hextorgba( h ):
    return h * 1.0 / 255.0

colors = [ [ hextorgba( i ) for i in [ 0x00, 0x99, 0x00, 0xFF ]], # green
           [ hextorgba( i ) for i in [ 0xEE, 0x00, 0x00, 0xFF ]], # red
           [ hextorgba( i ) for i in [ 0x00, 0xCC, 0xFF, 0xFF ]]] # teal

for loc in range( 0, 9 ):
    m_id += 1
    color = colors[ loc % len(colors) ]
    tag_spheres.append( vsm( np.matrix([ ahe.pts[loc][1][0],
                                         ahe.pts[loc][1][1],
                                         ahe.pts[loc][1][2] ]).T,
                             np.matrix([ [0.0], [0.0], [0.0], [1.0] ]),
                             'sphere', '/map',
                             color = color, # rgba
                             duration = 10.0,
                             m_id = m_id ))

    for di in d:
        if di['loc'] == loc:
            m_id += 1
            servo_arrows.append( arrow( m_id, di, color ))

    

marks = tag_spheres + servo_arrows

while not rospy.is_shutdown():
    [ pub_mark.publish( x ) for x in marks ]
    rospy.sleep( 1.0 )
    print 'WOOT'


