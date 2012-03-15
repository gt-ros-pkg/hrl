#!/usr/bin/python

import roslib
roslib.load_manifest( 'tf' )
roslib.load_manifest( 'rosbag' )
roslib.load_manifest( 'hrl_rfid' )
roslib.load_manifest( 'std_msgs' )
roslib.load_manifest( 'geometry_msgs' )
import rospy

import tf
import tf.transformations as tft
import rosbag
from hrl_rfid.msg import RFIDreadArr, RFIDread
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Empty

import sys
import glob
import yaml
import time
import optparse
import cPickle as pkl
import numpy as np, math
from multiprocessing import Process

import friis
import process_bags_utils as pbut
import math_util as mu


# yaml_config:
#     rfid_arr_topic:
#     glob_files: 'cap_360/datacap/*.bag'
#     antennas:
#         EleLeftEar: '/ear_antenna_left'
#         EleRightEar: '/ear_antenna_right'
#         PR2_Head: '/head_rfid'
#     rotframes:
#         EleLeftEar: '/ear_pan_left'
#         EleRightEar: '/ear_pan_right
#         PR2_Head: '/head_pan_link'
#     tags:
#         tagid1:
#             child_frame: '/tagid1'
#             parent_frame: '/map'
#             x_pos:
#             y_pos:
#             z_pos:
#             x_orient:
#             y_orient:
#             z_orient:
#             w_orient:

class ProcessReads_Friis():
    # Processes a bagfile to and returns a dictionary that is ready to calculate Friis values:
    # d = { 'tagid1': [ PROC_READ1, PROC_READ2, ... ],
    #       ...
    #     }
    def __init__( self, yaml_config, tf_listener ):
        # Assume tf will work fine... we'll catch errors later.
        # self.listener = tf.TransformListener()
        self.listener = tf_listener

        # Initialize Variables
        self.d = {}
        self.yaml_config = yaml_config

        # Start Processing Incoming Reads
        rfid_arr_topic = self.yaml_config['rfid_arr_topic']  # eg. '/rfid/ears_reader_arr'
        self.sub = rospy.Subscriber( rfid_arr_topic, RFIDreadArr, self.rfid_cb )


    def proc_read( self, rr ):  # rr is RFIDread.msg
        # Check for antenna_frame definition
        if not self.yaml_config[ 'antennas' ].has_key( rr.antenna_name ):
            rospy.logout( 'Antenna name %s undefined in yaml_config.  Skipping' % rr.antenna_name )
            return None
        rdr_frame = self.yaml_config[ 'antennas' ][ rr.antenna_name ] # eg. /ear_antenna_left

        rdr_ps = PointStamped()
        rdr_ps.header.frame_id = rdr_frame
        rdr_ps.header.stamp = rospy.Time(0) # Will want to transform the point right now.
        

        # Check for tag ground truth definition
        if not self.yaml_config[ 'tags' ].has_key( rr.tagID ):
            rospy.logout( 'TagID %s undefined in yaml_config.  Skipping' % rr.tagID )
            return None
        tag_frame = self.yaml_config[ 'tags' ][ rr.tagID ][ 'child_frame' ] # eg. /datacap  (these are being published in other thread)

        tag_ps = PointStamped()
        tag_ps.header.frame_id = tag_frame
        tag_ps.header.stamp = rospy.Time(0) # Will want to transform the point right now.

        try:
            tag_in_rdr = self.listener.transformPoint( rdr_frame, tag_ps )
            rdr_in_tag = self.listener.transformPoint( tag_frame, rdr_ps )
        except Exception, e:
            rospy.logout('Transform(s) failed: %s.  Skipping' % e.__str__() )
            return None

        r_rdr, theta_rdr, phi_rdr = friis.CartToSphere( tag_in_rdr.point.x,
                                                        tag_in_rdr.point.y,
                                                        tag_in_rdr.point.z )

        r_tag, theta_tag, phi_tag = friis.CartToSphere( rdr_in_tag.point.x,
                                                        rdr_in_tag.point.y,
                                                        rdr_in_tag.point.z )


        tag_pos = PoseStamped()
        tag_pos.header.frame_id = tag_frame
        tag_pos.header.stamp = rospy.Time(0) # Will want to transform the point right now.
        tag_pos.pose.orientation.w = 1.0

        rdr_pos = PoseStamped()
        rdr_pos.header.frame_id = rdr_frame
        rdr_pos.header.stamp = rospy.Time(0) # Will want to transform the point right now.
        rdr_pos.pose.orientation.w = 1.0

        rot_pos = PoseStamped()  # Note this is POSE stamped!
        rot_frame = self.yaml_config[ 'rotframes' ][ rr.antenna_name ] # eg. /ear_pan_left
        rot_pos.header.frame_id = rot_frame
        rot_pos.header.stamp = rospy.Time(0)
        rot_pos.pose.orientation.w = 1.0

        base_pos = PoseStamped()  # Note this is POSE stamped!
        base_pos.header.frame_id = '/base_link'
        base_pos.header.stamp = rospy.Time(0)
        base_pos.pose.orientation.w = 1.0

        static_rot_frame = self.yaml_config[ 'staticrotframes' ][ rr.antenna_name ] # eg. /ear_pan_left
        rdr_p = PointStamped()
        rdr_p.header.frame_id = rdr_frame
        rdr_p.header.stamp = rospy.Time(0) # Will want to transform the point right now.
        rdr_p.point.x = 1.0

        tag_p = PointStamped()
        tag_p.header.frame_id = tag_frame
        tag_p.header.stamp = rospy.Time(0) # Will want to transform the point right now.

        try:
            tag_map = self.listener.transformPose( '/map', tag_pos )
            rdr_map = self.listener.transformPose( '/map', rdr_pos )
            rot_map = self.listener.transformPose( '/map', rot_pos )
            base_map = self.listener.transformPose( '/map', base_pos )
            rdr_p_rot = self.listener.transformPoint( static_rot_frame, rdr_p )
            tag_p_rot = self.listener.transformPoint( static_rot_frame, tag_p )
            
        except Exception, e:
            rospy.logout('Transform(s) failed (#2): %s.  Skipping' % e.__str__() )
            return None

        # theta_rot_map_all = tft.euler_from_quaternion([ rot_map.pose.orientation.x,
        #                                                 rot_map.pose.orientation.y,
        #                                                 rot_map.pose.orientation.z,
        #                                                 rot_map.pose.orientation.w ])
        # theta_rot_map = theta_rot_map_all[2] # roll, pitch, yaw

        # F_x = rot_map.pose.position.x
        # F_y = rot_map.pose.position.y

        # T_x = tag_map.pose.position.x
        # T_y = tag_map.pose.position.y

        # theta_tag_map = np.arctan2( T_y - F_y, T_x - F_x )

        # Rotation relative to starting angle given by rotation frame
        theta_rot_map = np.arctan2( rdr_p_rot.point.y, rdr_p_rot.point.x )
        # Rotation of the ground-truth tag position
        theta_tag_map = np.arctan2( tag_p_rot.point.y, tag_p_rot.point.x )

        return [ [r_rdr, theta_rdr, phi_rdr],  # all floats
                 [r_tag, theta_tag, phi_tag],  # all floats
                 [rr.rssi, rr.antenna_name, rr.tagID ], # int, string, string
                 [theta_rot_map, theta_tag_map], # floats (radians)
                 [ tag_map, rdr_map, rot_map, base_map, rdr_p_rot, tag_p_rot ] ] # geometry_msgs/PoseStamped


    def add_val_dict( self, val ):
        if not self.d.has_key( val[2][-1] ):  # rr.tagID
            self.d[ val[2][-1] ] = []
        self.d[ val[2][-1] ].append( val )
        
        
    def rfid_cb( self, msg ):
        # Note: if any exceptions are thrown in proc_read, it will return value None.
        #       Otherwise: PROC_READ
        pr = [ self.proc_read( r ) for r in msg.arr ]
        [ self.add_val_dict( v ) for v in pr if v != None ]
        return

    def stop_processing( self, fname ):
        # Explicitly stop registering new reads.
        self.sub.unregister()

        rospy.logout( 'ProcessReads_Friis: Dumping' )
        for k in self.d.keys():
            rospy.logout( '\tTag \'%s\':%d Reads' % ( k, len(self.d[k]) ))

        fn = fname
        fn_root = fn[ : fn.rfind( '/' )+1 ]  # eg. cap_360/datacap/
        fn_file = fn[ fn.rfind( '/' )+1 : ]  # eg. 1304590000_0_datacap2.bag
        fn_pkl = fn_file[ : fn_file.rfind( '.' ) ] + '_friis.pkl' # eg. 1304590000_0_datacap2_friis.pkl
        out_fn = fn_root + fn_pkl

        f = open( out_fn, 'w' )
        pkl.dump( self.d, f )
        f.close()


        return True


def order_bagfiles( fnames ):
    # I'm too lazy to figure out how to reset time and prevent "TF_OLD_DATA" errors / warnings.
    #   Instead, we're just going to order the bag playback in wall-clock order.

    rospy.logout( 'Ordering the bagfiles in increasing order of start time.' )
    def gettime( fname ):
        # returns the timestamp of the first message
        b = rosbag.Bag( fname )
        msg = b.read_messages().next()
        tt = msg[-1].to_sec()
        b.close()
        return tt

    start_times = [ gettime( f ) for f in fnames ]
    rospy.logout( 'Done ordering.' )
    return [ fnames[ind] for ind in np.argsort( start_times ) ]

            
if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('--yaml', action='store', type='string', dest='yaml', default='',
                 help='yaml file that describes this run.')
    opt, args = p.parse_args()

    if not opt.yaml:
        print 'YAML file required!'
        exit()

    rospy.set_param('use_sim_time',True)  # just to make sure!
    rospy.init_node( 'friis_bag_processor' )

    f = open( opt.yaml )
    yaml_config = yaml.load( f )
    f.close()

    # Start tf frame publishers for tag ground truth
    tf_threads = [ pbut.TFthread( yaml_config['tags'][k] ) for k in yaml_config['tags'].keys() ]
    tf_listener = tf.TransformListener()

    fnames = reduce( lambda x,y: x+y, [ glob.glob(i) for i in yaml_config['glob_files'] ], [] )
    ordered_fnames = order_bagfiles( fnames )
    for i,fname in enumerate( ordered_fnames ):
        rospy.logout( 'Processing [ %d of %d ]: %s' % (i+1, len(fnames), fname) )

        # Start the new bagplay
        bp = pbut.bagplay( fname )
        bp.run()

        prf = ProcessReads_Friis( yaml_config, tf_listener )

        while not bp.is_finished():
            pbut.sim_safe_sleep( 1.0 )  # Cannot use rostime, since it will stall when bag stops

        # bp.kill()

        prf.stop_processing( fname ) # will dump results to fname_friis.pkl (less .bag)

    # After Loop end (later)
    [ t.stop() for t in tf_threads ]

    


