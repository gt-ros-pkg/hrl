#!/usr/bin/python

import roslib
roslib.load_manifest( 'rfid_people_following' )
import rospy

from scipy.spatial import KDTree
import numpy as np, math
import pickle as pkl
import markers
import point_cloud_utils as pcu
import time

rospy.init_node( 'traj_pub' )

print 'Loading data'

f = open( 'trajectory_caps/resutls.pkl' , 'r' )
dset = pkl.load( f )
f.close()

captures = dset['captures']

def Linf_norm( arr ):
    # arr is (2,)
    rv = np.max( np.abs( arr - np.array([7.0, 3.0]) ))
    #print 'arr: ', arr, ' rv: ', rv
    return rv

def filt( dist_min, dist_max ):
    # pts = 2xN array
    pts = np.array([]).reshape([4,0])
    for i in range(len(captures)):
        lp = load_points( captures[i][3] )
        if Linf_norm( lp[0:2,0] ) >= dist_min and Linf_norm( lp[0:2,0] ) <= dist_max:
            pts = np.column_stack([ pts, lp ])
    fpts_3d = np.row_stack([ pts[0], pts[1], np.zeros(pts.shape[1]) ])
    print fpts_3d.shape
    return np.matrix( fpts_3d )


def load_points( fname ):
    print 'Loading %s' % fname
    f = open( fname, 'r' )
    d = pkl.load( f )
    f.close()
    return np.array(d).T  # 4xN

# kd = KDTree( np.array([ [x,y] for x,y,ang,fname in captures ]))

#ind = kd.query_ball_point( np.array([ 3.0, 3.0 ]), r=0.5 )
#pts = np.column_stack([ load_points( captures[i][3] ) for i in ind ])

# pts = np.column_stack([ load_points( captures[i][3] ) for i in range(len(captures)) ])


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-x', action='store', type='float', dest='dist_max', default='20.0', help='max distance')
    p.add_option('-n', action='store', type='float', dest='dist_min', default='0.0', help='min distance')
    opt, args = p.parse_args()
                            
    pub = rospy.Publisher( 'traj_pub_pts', pcu.PointCloud )
    # ros_pts = pcu.np_points_to_ros( np.matrix(fpts_3d) )
    ros_pts = pcu.np_points_to_ros( filt( opt.dist_min, opt.dist_max ))
    ros_pts.header.frame_id = '/odom_combined'

    rate = rospy.Rate( 2 )

    print 'Publishing PointCloud'
    while not rospy.is_shutdown():
        ros_pts.header.stamp = rospy.Time.now()
        pub.publish( ros_pts )
        rate.sleep()



#     arr = np.array( res ).T
#     kd = KDTree( arr[0:2].T ) # xy-only.

#     X,Y = np.meshgrid( range(0,11), range(0,7) )
#     #xy = np.row_stack([ X.flatten(), Y.flatten() ])
#     Z = np.zeros( X.shape )

#     for i in xrange( X.shape[0] ):
#         for j in xrange( X.shape[1] ):
#             ind = kd.query_ball_point( np.array([ X[i,j], Y[i,j] ]), r=0.5 )
#             if ind:
#                 Z[i,j] = np.mean( arr[3,ind] )
#             else:
#                 #print i,j
#                 Z[i,j] = np.nan



