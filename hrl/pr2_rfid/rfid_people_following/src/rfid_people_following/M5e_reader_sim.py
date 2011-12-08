import roslib
roslib.load_manifest( 'rfid_people_following' )
roslib.load_manifest( 'tf' )
roslib.load_manifest( 'geometry_msgs' )
roslib.load_manifest('hrl_rfid')
import rospy

from hrl_rfid.msg import RFIDread
from hrl_rfid.msg import RFIDreadArr
from hrl_rfid.srv import RfidSrv
import tf
from geometry_msgs.msg import PointStamped

import sys, serial, time, string
from threading import Thread
import friis
import pickle as pkl
import numpy as np, math

# A nice functional print function (Norvig)
def prin(x): print x

f = open( '/home/travis/svn/data_robot1/rfid_data/pan_captures/model_fit.pkl', 'r' )
d = pkl.load( f )
f.close()

def calculate_angle( pt1 ):
    return np.arctan2( pt1.point.y, pt1.point.x )
    
class M5e:
    "Interface to Mercury M5e and M5e-Compact"
    def __init__(self, portSTR='/dev/robot/RFIDreader', baudrate=9600, 
        TXport=1, RXport=1, readPwr=2300, protocol='GEN2', compact=True, verbosity_func=prin):

        # verbosity_func should alawys take string as an input.  Lets us choose a selective 
        #   print function (like rospy.logout) instead of built-in print.

        self.TXport = TXport        # Initialized transmit antenna
        self.RXport = RXport        # Initialized receive antenna
        self.readPwr = readPwr      # Initialized read TX power in centi-dBm
        self.protocol = protocol    # Initialized protocol
        self.compact = compact
        self.prin = verbosity_func
        self.ser = None

        try:
            rospy.init_node( 'RFID_Simulator_Friis' )
        except rospy.ROSException:
            pass
        
        self.prin( 'Initializing M5e (or M5e-Compact)' )
        self.listener = tf.TransformListener()

        rospy.logout('Simulated M5e: Waiting for odom_combined-antenna tf')
        self.listener.waitForTransform('/ear_antenna_left', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/ear_antenna_right', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/left_rotation_center', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/right_rotation_center', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/base_link', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self.listener.waitForTransform('/tag_gt', '/odom_combined',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        rospy.logout('Simulated M5e: Transforms ready')
        self.prin( 'M5e Initialized' )
        
        
    def ChangeAntennaPorts(self, TXport, RXport):
        "Changes TX and RX ports"
        self.TXport = TXport
        self.RXport = RXport
        
    # Currently Unimplemented!
    def QueryEnvironment(self, timeout=50):
        rospy.sleep( timeout * 0.001 )
        return []

    # Relies on a single tag whose coord frame is '/tag_gt'
    #   Lots of hardcoding... boo!
    def TrackSingleTag(self, tagID, timeout=50):
        rospy.sleep( timeout * 0.001 )

        # 6DOF params -- Calculate these using tf
        ant_zp = PointStamped()
        ant_zp.header.stamp = rospy.Time(0)

        tag_zp = PointStamped()
        tag_zp.header.stamp = rospy.Time(0)
        tag_zp.header.frame_id = '/tag_gt'
        
        if self.TXport == 1 and self.RXport == 1: # 'EleLeftEar'
            ear_frame = '/ear_antenna_left'
        else: # 'EleRightEar'
            ear_frame = '/ear_antenna_right'

        ant_zp.header.frame_id = ear_frame

        # should probably catch exceptions and throw big warnings...
        tag_in_ant = self.listener.transformPoint( ear_frame, tag_zp )
        ant_in_tag = self.listener.transformPoint( '/tag_gt', ant_zp )


        tag_in_ant_sphere = np.array( friis.CartToSphere2( tag_in_ant.point.x,
                                                           tag_in_ant.point.y,
                                                           tag_in_ant.point.z ))
        ant_in_tag_sphere = np.array( friis.CartToSphere2( ant_in_tag.point.x,
                                                           ant_in_tag.point.y,
                                                           ant_in_tag.point.z ))
        

        # Determine Friis Powers
        Ptag = friis.pwr_inc_tag( tag_in_ant_sphere[0], # radius
                                  friis.patch.G, tag_in_ant_sphere[1], tag_in_ant_sphere[2],
                                  friis.dipole.G, ant_in_tag_sphere[1], ant_in_tag_sphere[2] )
        Prdr = friis.pwr_inc_rdr( tag_in_ant_sphere[0], # radius
                                  friis.patch.G, tag_in_ant_sphere[1], tag_in_ant_sphere[2],
                                  friis.dipole.G, ant_in_tag_sphere[1], ant_in_tag_sphere[2] )
        Prdr_dBm = friis.WattsToDBm( Prdr )

        # Calculate expectation (read/no-read and RSSI if applicable)
        sim_read = Ptag > friis.DBmToWatts( -18.0 )
        sim_rssi_ind = np.sum( Prdr_dBm > d['bins'][:-1] ) - 1
        sim_rssi_ind = np.clip( sim_rssi_ind, 0, sim_rssi_ind ) # if less than 0 (power lower than in previous data)
        sim_rssi = d['mean_rssi'][sim_rssi_ind]

        if sim_read:
            return int(sim_rssi) # be a little more accurate by returning int
        else:
            return -1




# Modeled off lib_M5e.M5e_Poller
class ROS_M5e( Thread ):
    QUERY_MODE = 'query'
    TRACK_MODE = 'track'
    
    def __init__(self, name = 'reader1', readPwr = 2300,
                 portStr = '/dev/robot/RFIDreader',
                 antFuncs = [], callbacks = []):

        Thread.__init__(self)
        self.should_run =  True

        try:
            rospy.init_node( 'rfid_m5e_' + name )
        except rospy.ROSException:
            pass

        self.mode = ''
        self.name = name + '_reader'

        rospy.logout( 'ROS_M5e: Launching RFID Reader' )
        rospy.logout( 'ROS_M5e: Please check out our related work @ http://www.hsi.gatech.edu/hrl/project_rfid.shtml' )
        rospy.logout( 'ROS_M5e: '+self.name+' Building & Connecting to reader' )

        def prin( x ): rospy.logout( 'ROS_M5e: lib_M5e: ' + x ) # use rospy.logout in underlying lib's output

        self.reader = M5e(readPwr=readPwr, portSTR = portStr, verbosity_func = prin)
        self.antFuncs = antFuncs
        self.callbacks = callbacks + [self.broadcast]

        rospy.logout( 'ROS_M5e: publishing RFID reader with type RFIDread to channel /rfid/'+name+'_reader' )
        self.channel       = rospy.Publisher('/rfid/'+name+'_reader', RFIDread)
        self.pub_arr = rospy.Publisher('/rfid/'+name+'_reader_arr', RFIDreadArr)
        self._mode_service_obj = rospy.Service('/rfid/'+name+'_mode',
                                                RfidSrv, self._mode_service)

        rospy.logout( 'ROS_M5e: '+self.name+' Inialized and awaiting instructions' )

        self.start()  # Thread: calls self.run()

    def run( self ): 
        while self.should_run:
            if self.mode == self.QUERY_MODE:
                for aF in self.antFuncs:
                    antennaName = aF(self.reader)    # let current antFunc make appropriate changes
                    results = self.reader.QueryEnvironment()
                    if len(results) == 0:
                        results = [[ '', -1 ]] # [[ tagid, rssi ], ...]
                        #datum = [antennaName, '', -1]
                        #[cF(datum) for cF in self.callbacks]
                    arr = []
                    t_now = rospy.Time.now()
                    for tagid, rssi in results:
                        rv = RFIDread( None, antennaName, tagid, rssi )
                        rv.header.stamp = t_now
                        arr.append( rv )
                        datum = [antennaName, tagid, rssi]
                        [cF(datum) for cF in self.callbacks]
                    rfid_arr = RFIDreadArr()
                    rfid_arr.header.stamp = t_now
                    rfid_arr.arr = arr
                    self.pub_arr.publish( rfid_arr )
            elif self.mode == self.TRACK_MODE:
                for aF in self.antFuncs:
                    antennaName = aF(self.reader)    # let current antFunc make appropriate changes
                    tagid = self.tag_to_track
                    rssi = self.reader.TrackSingleTag(tagid, timeout=100)
                    t_now = rospy.Time.now()
                    rv = RFIDread( None, antennaName, tagid, rssi )
                    rv.header.stamp = t_now
                    rfid_arr = RFIDreadArr()
                    rfid_arr.header.stamp = t_now
                    rfid_arr.arr = [rv]
                    self.pub_arr.publish( rfid_arr )
                    #if rssi != -1:
                    datum = [antennaName, tagid, rssi]
                    [cF(datum) for cF in self.callbacks]
            else:
                time.sleep(0.005)

        rospy.logout( 'ROS_M5e: '+self.name+' Shutting down reader' )

    def stop( self ):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ROS_M5e: unable to stop thread")            
            
    def broadcast(self, data):
        antName, tagid, rssi = data
        rv = RFIDread( None, antName, tagid, rssi )
        rv.header.stamp = rospy.Time.now()
        self.channel.publish( rv )
    
    # For internal use only
    def _mode_service(self, data):
        val = data.data
        if len(val) == 0:
            rospy.logout( 'ROS_M5e: Mode Service called with invalid argument: ' + str(val) )
        elif len(val) == 1:
            if val[0] == self.QUERY_MODE:
                rospy.logout( 'ROS_M5e: '+self.name+' Entering Query Mode' )
                self.mode = self.QUERY_MODE
            else:
                rospy.logout( 'ROS_M5e: '+self.name+' Stopping Reader' )
                self.mode = ''
        elif len(val) == 2:
            if val[0] == self.TRACK_MODE and len(val[1]) == 12:
                rospy.logout( 'ROS_M5e: '+self.name+' Entering Track Mode: ' + str(val[1]) )
                self.mode = self.TRACK_MODE
                self.tag_to_track = val[1]
            else:
                rospy.logout( 'ROS_M5e: Mode Service called with invalid argument: ' + str(val) )
        else:
            rospy.logout( 'ROS_M5e: Mode Service called with invalid argument: ' + str(val) )
        return True



# -----------------------------------------------
# Likely Callbacks: (various antennas)
# -----------------------------------------------

def EleLeftEar(M5e):
    M5e.ChangeAntennaPorts(1,1)
    rospy.sleep(0.010)
    return 'EleLeftEar'

def EleRightEar(M5e):
    M5e.ChangeAntennaPorts(2,2)
    rospy.sleep(0.010)
    return 'EleRightEar'

def Hand_Right_1(M5e):
    # GPIO1 = 1, GPIO2 = 0
    M5e.TransmitCommand('\x02\x96\x01\x01')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x00')
    M5e.ReceiveResponse()
    return 'Hand_Right_1'

def Hand_Right_2(M5e):
    # GPIO1 = 1, GPIO2 = 1
    M5e.TransmitCommand('\x02\x96\x01\x01')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x01')
    M5e.ReceiveResponse()
    return 'Hand_Right_2'

def Hand_Left_1(M5e):
    # GPIO1 = 0, GPIO2 = 0
    M5e.TransmitCommand('\x02\x96\x01\x00')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x00')
    M5e.ReceiveResponse()
    return 'Hand_Left_1'

def Hand_Left_2(M5e):
    # GPIO1 = 0, GPIO2 = 1
    M5e.TransmitCommand('\x02\x96\x01\x00')
    M5e.ReceiveResponse()
    M5e.TransmitCommand('\x02\x96\x02\x01')
    M5e.ReceiveResponse()
    return 'Hand_Left_2'

def PrintDatum(data):
    ant, ids, rssi = data
    print data

