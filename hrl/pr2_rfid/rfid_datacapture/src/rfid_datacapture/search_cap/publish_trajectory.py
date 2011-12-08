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


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--trial', action='store', type='int', dest='trial',
                 help='trial number (0-8)')
    # p.add_option('--fname', action='store', type='string', dest='fname',
    #              help='File name. Should be woot_150_x_reads.pkl', default='')

    opt, args = p.parse_args()
    
    fname = 'search_aware_home/woot_150_'+str(opt.trial)+'_reads.pkl'

    f = open( fname, 'r' )
    r = pkl.load(f)
    f.close()

    xyz = np.array([ [p.ps_base_map.pose.position.x,
                      p.ps_base_map.pose.position.y,
                      p.ps_base_map.pose.position.z ] for p in r ]).T

    rospy.init_node( 'pub_traj' )
    pts = ru.np_to_pointcloud( xyz, '/map' )
    pub_pts = rospy.Publisher( '/robot_traj', sm.PointCloud )
    pub_mark = rospy.Publisher( '/tag_poses', vm.Marker )
    rospy.sleep( 0.5 )

    tag_pts = np.array([ ahe.pts[k][1] for k in ahe.pts.keys() ]).T
    tm = [ viz.single_marker( tag_pts[:,i].reshape([3,1]),
                              np.matrix([ [0.0], [0.0], [0.0], [1.0] ]),
                              'sphere', '/map',
                              color=[0.0, 1.0, 0.0, 1.0],
                              m_id=i ) for i in xrange( tag_pts.shape[1] )]
    

    while not rospy.is_shutdown():
        pts.header.stamp = rospy.Time.now()
        pub_pts.publish( pts )
        [ pub_mark.publish( x ) for x in tm ]
        rospy.sleep( 1.0 )
        
