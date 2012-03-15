#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_behaviors')
import rospy

import tf

from rfid_behaviors.srv import RecorderSrv, RecorderSrvResponse
from rfid_behaviors.srv import NextBestVantage
from rfid_behaviors.msg import RecorderReads
import hrl_rfid.ros_M5e_client as rmc
from geometry_msgs.msg import PoseStamped

import numpy as np, math

class Recorder( ):
    def __init__( self, serv_name = 'rfid_recorder', node_name = 'rfid_recorder_py' ):
        rospy.logout( 'rfid_recorder: initializing' )
        try:
            rospy.init_node(node_name)
        except:
            pass

        self.should_rec = False

        self._servrec = rospy.Service( serv_name + '/record',
                                       RecorderSrv,
                                       self.process_service )

        self._servbest = rospy.Service( serv_name + '/best_vantage',
                                        NextBestVantage,
                                        self.bestvantage )

        self.listener = tf.TransformListener()
        rospy.logout( 'RFID Recorder: Waiting on transforms' )
        self.listener.waitForTransform('/ear_antenna_left', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/ear_antenna_right', '/map',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        

        rospy.logout( 'rfid_recorder: ready' )

    def process_service( self, req ):
        self.should_rec = not self.should_rec  # toggle state.  (bad way to do this...)

        if self.should_rec == True:
            self.data = []
            self.rec = rmc.ROS_M5e_Client('ears', callbacks = [self.add_datum])
            rospy.logout( 'RFID Recorder: Logging Reads.' )
            rv = RecorderSrvResponse()
            rv.rfid_reads = []
        else:
            rospy.logout( 'RFID Recorder: Halting recorder.' )
            self.rec.unregister() # Stop processing new reads
            rospy.sleep( 0.5 ) # Give it some time to settle
            rv = RecorderSrvResponse()
            rv.rfid_reads = list(self.data) # Save the data.

        self.recorder_data = list( self.data )
        return rv

    def process_datum( self, datum ):
        # Hooray for lexical scope (listener)!
        ant_lookup = { 'EleLeftEar': '/ear_antenna_left',
                       'EleRightEar': '/ear_antenna_right' }

        ps_ant = PoseStamped()
        ps_ant.header.stamp = rospy.Time( 0 )
        ps_ant.header.frame_id = ant_lookup[ datum.antenna_name ]

        ps_base = PoseStamped()
        ps_base.header.stamp = rospy.Time( 0 )
        ps_base.header.frame_id = '/base_link'

        try:
            ps_ant_map = self.listener.transformPose( '/map', ps_ant )
            ps_base_map = self.listener.transformPose( '/map', ps_base )
            rv = RecorderReads()
            rv.read = datum
            rv.ps_ant_map = ps_ant_map
            rv.ps_base_map = ps_base_map 
        except:
            rospy.logout( 'RFID Recorder: TF failed. Ignoring read.' )
            rv = None
        return rv

    def add_datum( self, datum ):
        # Hooray for lexical scope (data)!
        pd = self.process_datum( datum )
        if pd != None:
            self.data.append( pd )

    def bestvantage(self, req):
        rospy.logout('Recorder: Calculating best vantage for tag \'%s\'' % req.tagid)

        d = {}
        for rr in self.recorder_data:  # rr is RecorderRead
            if not d.has_key( rr.read.tagID ):
                d[rr.read.tagID] = []
            d[rr.read.tagID].append( rr )

        pos_reads = []
        if d.has_key( req.tagid ):
            pos_reads = [ q for q in d[ req.tagid ] if q.read.rssi != -1 ]  # list of RecorderReads

        if not pos_reads: # check at least one positive reading
            rospy.warn( 'Recorder: Desired tag had no readings.' )
            rv = PoseStamped()
            rv.header.frame_id = 'base_link'
            rv.header.stamp = rospy.Time.now()
            rv.pose.orientation.w = 1.0
            return rv

        # Select the RecorderRead with greatest RSSI
        rssi = [ r.read.rssi for r in pos_reads ]
        ind = np.argmax( rssi )

        best = pos_reads[ ind ] # RecorderRead
        best_read = best.read
        best_ant = best.ps_ant_map
        best_base = best.ps_base_map

        #print best_read, best_ant, best_base

        # We're going to keep the <x,y> location from the baselink (mapframe),
        # but keep <ang> (mapframe) from the antenna.

        rv = PoseStamped()
        rv.header.stamp = rospy.Time.now()
        rv.header.frame_id = best_base.header.frame_id
        rv.pose.position = best_base.pose.position
        rv.pose.orientation = best_ant.pose.orientation

        return rv
    
    
        
if __name__ == '__main__':
    rec = Recorder()
    rospy.spin()
