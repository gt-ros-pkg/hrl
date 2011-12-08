#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
roslib.load_manifest('rosbag')
import rospy

import tf

from rfid_behaviors.srv import RecorderSrv, RecorderSrvResponse
from rfid_behaviors.srv import NextBestVantage
from rfid_behaviors.msg import RecorderReads
import hrl_rfid.ros_M5e_client as rmc
from geometry_msgs.msg import PoseStamped
from hrl_rfid.msg import RFIDread
from hrl_lib import util

import process_bags_utils as pbut
from rfid_behaviors import recorder
import rosbag
import glob
import numpy as np,math

SERVO_FNAMES = 'search_cap/search_aware_home/*_servo.bag'


# This is a modified version of rfid_behaviors.recorder.py
class TmpRecorder( ):
    def __init__( self, serv_name = 'rfid_recorder', node_name = 'rfid_recorder_py' ):
        rospy.logout( 'rfid_recorder: initializing' )
        try:
            rospy.init_node(node_name)
        except:
            pass

        self.name = 'ears'
        
        self.should_rec = False

        self.listener = tf.TransformListener()
        # rospy.logout( 'RFID Recorder: Waiting on transforms' )
        # self.listener.waitForTransform('/ear_antenna_left', '/map',
        #                                rospy.Time(0), timeout = rospy.Duration(100) )
        # self.listener.waitForTransform('/ear_antenna_right', '/map',
        #                                rospy.Time(0), timeout = rospy.Duration(100) )

        self.data = []
        self._sub = rospy.Subscriber( '/rfid/' + self.name + '_reader', RFIDread, self.add_datum)

        rospy.logout( 'rfid_recorder: ready' )

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
    import optparse
    p = optparse.OptionParser()
    p.add_option('--fname', action='store', dest='fname',
                 help='filename', default = '')
    opt, args = p.parse_args()

    
    rospy.init_node( 'process_servo_reads' )

    ordered_fnames = order_bagfiles( glob.glob(SERVO_FNAMES) )

    print 'starting recorder.'
    rec = TmpRecorder( serv_name = 'temp_recorder', node_name = 'temp_recorder_py' )
    print 'done starting'

    for i,fname in enumerate( ordered_fnames ):
        rospy.logout( 'Processing [ %d of %d ]: %s' % (i+1, len(ordered_fnames), fname) )

        rec.data = []

        # Start the new bagplay
        bp = pbut.bagplay( fname )
        bp.run()

        while not bp.is_finished():
            print 'Still waiting...'
            pbut.sim_safe_sleep( 1.0 )  # Cannot use rostime, since it will stall when bag stops
        print 'Done Waiting.'

        print 'Saving recorder pickle data.'
        util.save_pickle( rec.data, fname.replace('.bag','.pkl') )
