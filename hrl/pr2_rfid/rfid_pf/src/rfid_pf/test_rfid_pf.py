#!/usr/bin/python

import random as rd
import numpy as np, math

import rfid_model
import lib_pfilter

import roslib
roslib.load_manifest('visualization_msgs')
roslib.load_manifest('hrl_lib')
import rospy

import visualization_msgs.msg as vm
import hrl_lib.transforms as tr
import hrl_lib.viz as viz
vsm = viz.single_marker

import pylab as pl

class DisplayWeights:
    def __init__( self, pub_topic = '/particles' ):
        self.m = None
        self.pub_mark = rospy.Publisher( pub_topic, vm.Marker )
        self.mid = 0

    def create_mark( self, p, c = [1.0, 0.0, 0.0, 0.8], mid = None ):
        if mid == None:
            self.mid += 1
            mid = self.mid
            
        m =  vsm( np.matrix([ p[0], p[1], 0.0 ]).T,
                  np.matrix([ 0.0, 0.0, 0.0, 1.0 ]).T,
                  'sphere', '/map',
                  scale = [0.05, 0.05, 0.05],
                  color = [1.0, 0.0, 0.0, 0.8], # rgba,
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
        m = viz.list_marker( xyz, colors, [0.05, 0.05, 0.05], 'points', '/map', 300 )
        m.header.stamp = rospy.Time.now()
        for i in xrange( 10 ):
            self.pub_mark.publish( m )
            rospy.sleep( 0.2 )

        return
        
        # if not self.m:
        #     self.m = [ self.create_mark( p, mid=None ) for i,p in enumerate( particles )]
        # else:
        #     self.m = [ self.create_mark( p, mid=i ) for i,p in enumerate( particles )]

        # for i in xrange( 10 ):
        #     [ self.pub_mark.publish( m ) for m in self.m ]
        #     rospy.sleep( 0.2 )
        



# if __name__ == '__main__':

rospy.init_node( 'test_rfid_pf' )

# build particles
X,Y = np.meshgrid( np.arange(-10,10,0.1),
                   np.arange(-10,10,0.1) )
xyw = np.row_stack([ X.flatten(),  # Build Nx3
                     Y.flatten(),
                     np.ones( X.shape ).flatten() ]).T # weights (multiplicative)

xyw_orig = np.copy( xyw )

rm = rfid_model.RfidModel( rfid_model.yaml_fname )
# pf = lib_pfilter.PFilter( lib_pfilter.NoMotion(), rm, xyw )
# pf.measurement( 80 )


# Antenna reading at origin (max near 4,0)
# rv = np.copy( xyw )
rv = rm.weight_set( 83, xyw )

# Move antenna to 4,-4 and rotated 90 deg
xy = xyw[:,0:2]
trans = tr.composeHomogeneousTransform( tr.rotZ( math.radians(-90) ), np.matrix([5,5,0]).T )
xy_new = np.array((trans * tr.xyToHomogenous( xy.T) )[0:2].T)

xyw_new = np.column_stack([ xy_new, rv[:,2] ])  # take weights from last iteration
rv_new = rm.weight_set( 83, xyw_new )

to_print = np.copy( xyw_orig )
to_print[:,2] = rv_new[:,2]  # Just keep weights, but use original points in "map" frame



dw = DisplayWeights()

while not rospy.is_shutdown():
    print 'UPDATING'
    dw.update( to_print )
    # dw.update( rv )
    m = int( raw_input( 'New RSSI: ' ))
    # rv = rm.weight_set( m, xyw )
    # rospy.sleep( 150 )


