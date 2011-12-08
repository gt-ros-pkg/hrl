#! /usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('tf')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')
roslib.load_manifest('rfid_behaviors')
import rospy

import tf
import tf.transformations as tft
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

import hrl_rfid.ros_M5e_client as rmc
import robotis.ros_robotis as rr
from hrl_rfid.msg import RFIDreadArr
import rfid_behaviors.rotate_backup_node as rb
from rfid_behaviors.cmd_process import CmdProcess
from rfid_behaviors.srv import String_Int32
from rfid_behaviors.srv import String_Int32Response
from rfid_behaviors.srv import FlapEarsSrv
from rfid_behaviors.srv import StringArr_Int32

import numpy as np, math
import time
from threading import Thread
from collections import deque
from functools import reduce

#PAN_RATE = 30.0
PAN_RATE = 10.0  # Used for datacapture.
#PAN_RATE = 3.0 

def calculate_angle( pt1 ):
    return np.arctan2( pt1.point.y, pt1.point.x )

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi


class OrientNode(  ):
    def __init__( self ):
        rospy.logout('orient_node: Initializing')
        rospy.init_node('orient_node')

        # After calling "flap ears", data will look something like this:
        # { 'TagID1': [[ang,rssi], [ang,rssi], ...]
        #   'TagID2': ... }
        #  * All angles are in /base_link and rssi's from both antennas
        self.data = {}

        # Will be transformed into base frame to determine best turn angle -- results in approximately 5-degrees (max) error for small angle assumption
        self.tag_gt = { 'EleLeftEar': PointStamped(), 'EleRightEar': PointStamped() }
        self.tag_gt[ 'EleLeftEar' ].header.frame_id = '/ear_antenna_left'
        self.tag_gt[ 'EleLeftEar' ].header.stamp = rospy.Time.now()
        self.tag_gt[ 'EleLeftEar' ].point.x = 10.0
        self.tag_gt[ 'EleRightEar' ].header.frame_id = '/ear_antenna_right'
        self.tag_gt[ 'EleRightEar' ].header.stamp = rospy.Time.now()
        self.tag_gt[ 'EleRightEar' ].point.x = 10.0

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/base_link', '/ear_antenna_left',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/base_link', '/ear_antenna_right',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        rospy.logout('orient_node: Transforms ready')

        # For movement...
        self.rotate_backup_client = rb.RotateBackupClient()
        
        # "Ears" Setup
        self.p_left = rr.ROS_Robotis_Client( 'left_pan' )
        self.t_left = rr.ROS_Robotis_Client( 'left_tilt' )
        self.p_right = rr.ROS_Robotis_Client( 'right_pan' )
        self.t_right = rr.ROS_Robotis_Client( 'right_tilt' )
    
        self.EX_1 = 1.350
        self.EX_2 = 0.920 

        self.p_left.move_angle( self.EX_1, math.radians(10), blocking = False )
        self.p_right.move_angle( -1.0 * self.EX_1, math.radians(10), blocking = True )
        self.t_left.move_angle( 0.0, math.radians(10), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(10), blocking = True )
        while self.p_left.is_moving() or self.p_right.is_moving():
            time.sleep( 0.01 )

        self.bag_pid = None

        self.r = rmc.ROS_M5e_Client('ears')
        self.__service_flap = rospy.Service( '/rfid_orient/flap',
                                             FlapEarsSrv,
                                             self.flap_ears )
        self.__service_bag = rospy.Service( '/rfid_orient/bag',
                                             StringArr_Int32,
                                             self.bag_cap )
        self.__service_orient = rospy.Service( '/rfid_orient/orient',
                                             String_Int32,
                                             self.orient )
        self.tag_arr_sub = rospy.Subscriber( '/rfid/ears_reader_arr', 
                                             RFIDreadArr, 
                                             self.add_tags )
        rospy.logout( 'orient_node: Waiting for service calls.' )

    def bag_cap( self, request ):
        # request.data => String array
        # sample args: ['rosbag', 'record', '/tf', '/rfid/ears_reader_arr', '-o', 'data/data']
        if (request.data == [] or request.data[0] == 'kill'):
            if self.bag_pid == None:
                rospy.logout( 'orient_node: No open bag to kill.' )
            else:
                rospy.logout( 'orient_node: Killing open bag.' )
                self.bag_pid.kill()
                self.bag_pid = None
            return int( True )
        
        s = reduce( lambda x,y: x+' '+y, request.data )
        rospy.logout( 'orient_node: Calling CmdProcess with args: %s' % s )
        self.bag_pid = CmdProcess( request.data )
        self.bag_pid.run()
        return int( True )

    def orient( self, request ):
        tagid = request.data
        if not self.data.has_key( tagid ):
            rospy.logout( 'Tag id \'%s\' not found during last scan.' % tagid )
            return String_Int32Response( int( False ))
        arr = np.array( self.data[ tagid ]).T
        arr = arr[:,np.argsort( arr[0] )]
        h, bins = np.histogram( arr[0], 36, ( -np.pi, np.pi ))
        ind = np.sum(arr[0][:, np.newaxis] > bins, axis = 1) - 1  # Gives indices for data into bins
        bin_centers = (bins[:-1] + bins[1:]) / 2.0

        best_dir = 0.0
        best_rssi = 0.0
        for i in np.unique( ind ):
            avg_rssi = np.mean(arr[1,np.argwhere( ind == i )])
            if  avg_rssi > best_rssi:
                best_rssi = avg_rssi
                best_dir = bin_centers[i]

        rospy.logout( 'orient_node: Best dir (deg): %2.2f with avg rssi: %2.1f' %
                      ( math.degrees(best_dir), best_rssi ))

        self.rotate_backup_client.rotate_backup( best_dir, 0.0 )
        return String_Int32Response( int( True ))
            

    def add_tags( self, msg ):
        for read in msg.arr:
            if read.rssi == -1:
                return False

            self.tag_gt[ read.antenna_name ].header.stamp = rospy.Time(0)
            try:
                pt = self.listener.transformPoint( '/base_link', 
                                                   self.tag_gt[ read.antenna_name ])
            except:
                rospy.logout( 'orient_node: Transform failed' )
                return False

            if not self.data.has_key( read.tagID ):
                self.data[ read.tagID ] = []
            self.data[ read.tagID ].append([ calculate_angle( pt ), 1.0 * read.rssi ])
        return True

    def flap_ears( self, request ):
        if request.panrate == 0.0:
            rpan_rate = 30.0
        else:
            rpan_rate = request.panrate
        self.data = {}
        tagid = request.data
        if tagid == '':
            rospy.logout( 'orient_node: capture for tagid: \'\' requested. Using QueryEnv.' )
            self.r.query_mode( )
        else:
            rospy.logout( 'orient_node: capture for tagid: \'%s\' requested' % tagid )
            self.r.track_mode( tagid )
            
        forward = False
        tilt_angs = [ math.radians( 0.0 ),
                      math.radians( 0.0 ) ]

        for ta in tilt_angs:
            # Tilt
            self.t_left.move_angle( ta, math.radians( 30.0 ), blocking = False )
            self.t_right.move_angle( -1.0 * ta, math.radians( 30.0 ), blocking = False )
            while self.t_left.is_moving() or self.t_right.is_moving():
                time.sleep(0.01)

            # Pan
            if forward:
                self.p_left.move_angle( self.EX_1, math.radians( rpan_rate ), blocking = False )
                self.p_right.move_angle( -1.0 * self.EX_1, math.radians( rpan_rate ), blocking = True )
                forward = False
            else:
                self.p_left.move_angle( -1.0 * self.EX_2, math.radians( rpan_rate ), blocking = False )
                self.p_right.move_angle( self.EX_2, math.radians( rpan_rate ), blocking = True )
                forward = True

            while self.p_left.is_moving() or self.p_right.is_moving():
                time.sleep(0.01)

        time.sleep(0.1)

        self.r.stop()
        print self.data.keys()

        # Reset / Stow
        self.p_left.move_angle( self.EX_1, math.radians(10), blocking = False )
        self.t_left.move_angle( 0.0, math.radians(10), blocking = False )
    
        self.p_right.move_angle( -1.0 * self.EX_1, math.radians(10), blocking = False )
        self.t_right.move_angle( 0.0, math.radians(10), blocking = False )

        rospy.logout( 'orient_node: capture completed' )
        # print self.data

        return [self.data.keys()]
        


if __name__ == '__main__':
    on = OrientNode()
    rospy.spin()
