#!/usr/bin/python

import roslib
roslib.load_manifest( 'tf' )
roslib.load_manifest( 'hrl_lib' )
import rospy

import tf
from hrl_lib.cmd_process import CmdProcess

from threading import Thread
import time


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

class TFthread( Thread ):
    # Takes in a (PoseStamped-like) dictionary and publishes a new transform from frame_id -> frame_name
    # For example: (YAML syntax)
    #    child_frame: '/datacap'
    #    parent_frame: '/map'
    #    x_pos: 6.480
    #    y_pos: 2.865
    #    z_pos: 1.291
    #    x_orient: 0.0
    #    y_orient: 0.0
    #    z_orient: 0.0
    #    w_orient: 1.0
    
    def __init__( self, d ):
        self.d = d
        Thread.__init__( self )
        
        # cf = self.d[ 'child_frame' ]
        # rospy.init_node( 'tf_thread_' + cf.strip('/') )

        self.bc = tf.TransformBroadcaster()

        self.should_run = True
        self.start()

    def run( self ):
        rospy.logout( 'TFthread: Starting %s ' % self.d[ 'child_frame' ] )
        while self.should_run and not rospy.is_shutdown():
            self.bc.sendTransform( ( self.d[ 'x_pos' ],
                                     self.d[ 'y_pos' ],
                                     self.d[ 'z_pos' ] ),
                                   ( self.d[ 'x_orient' ],
                                     self.d[ 'y_orient' ],
                                     self.d[ 'z_orient' ],
                                     self.d[ 'w_orient' ] ),
                                   rospy.Time.now(),
                                   self.d[ 'child_frame' ],
                                   self.d[ 'parent_frame' ] )

            try:
                sim_safe_sleep( 0.10 )  # 10 Hz
            except:
                pass # ROS exception (eg. Ctrl-C).
        rospy.logout( 'TFthread: Stopping %s ' % self.d[ 'child_frame' ] )
        
    def stop( self ):
        # Kill off the poller thread.
        self.should_run = False
        self.join(5)
        if (self.isAlive()):
            raise RuntimeError('TFthread: Unable to stop thread %s ' % self.d[ 'child_frame' ] )
