#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('sound_play')
roslib.load_manifest('hrl_lib')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
import actionlib
import tf
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped, Quaternion
from sound_play.msg import SoundRequest
from rfid_datacapture.srv import BagCapture
from sound_play.msg import SoundRequest

from hrl_lib.cmd_process import CmdProcess

import yaml
import numpy as np, math
import os
import time

class BagCap():
    def __init__( self, topic_name = 'capture' ):
        self._srv = rospy.Service( 'bag_cap/'+topic_name, BagCapture, self.process_srv )

    def process_srv( self, req ):
        cmd = 'rosbag record -O %s ' % req.dest
        cmd += req.topics
        if req.topics != '':
            self.bag_cap = CmdProcess( cmd.split() )
            self.bag_cap.run()
        else:
            self.bag_cap.kill()
        return True
            

class YAMLproc(smach.State):
    def __init__(self, fname):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             output_keys = ['next_move_pose','bagfile_name',
                                            'bagfile_topics','tagid'])
        self.fname = fname
        self.d = None

    def execute(self, userdata):
        if not self.d:
            f = open( self.fname, 'r' )
            self.d = yaml.load( f )
            self.size = len( self.d['servo_cap']['captures'] ) 
            self.ind = 0
            f.close()

        if self.ind < self.size:
            rospy.logout( 'YamlProc: issuing %d of %d' % (self.ind+1, self.size))
            ps = PoseStamped()
            ps.header.frame_id = '/map'
            ps.header.stamp = rospy.Time(0)

            ps.pose.position.x = self.d['servo_cap']['captures'][ self.ind ][0]
            ps.pose.position.y = self.d['servo_cap']['captures'][ self.ind ][1]
            ang = self.d['servo_cap']['captures'][ self.ind ][2]
            q = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, math.radians( ang )))
            ps.pose.orientation = q

            self.ind += 1
            
            userdata.next_move_pose = ps
            userdata.tagid = self.d['servo_cap']['tagid']
            userdata.bagfile_name = self.d['servo_cap']['bagfile_path'] + str(int(rospy.Time.now().to_sec()))
            userdata.bagfile_topics = self.d['servo_cap']['bagfile_topics']
            return 'succeeded'
        else:
            return 'aborted' # done!


class YAMLprocPoses(smach.State):
    def __init__(self, fname):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             output_keys = ['next_move_pose'])
        self.fname = fname
        self.d = None

    def execute(self, userdata):
        if not self.d:
            f = open( self.fname, 'r' )
            self.d = yaml.load( f )
            self.size = len( self.d['rad_cap']['captures'] ) 
            self.ind = 0
            f.close()
            self.t0 = rospy.Time.now().to_sec()

        if self.ind < self.size:
            rospy.logout( 'YamlProcPoses: issuing %d of %d' % (self.ind+1, self.size))
            rospy.logout( 'YamlProcPoses: Time to capture %2.2f' % ( rospy.Time.now().to_sec() - self.t0 ))
            self.t0 = rospy.Time.now().to_sec()
            ps = PoseStamped()
            ps.header.frame_id = '/map'
            ps.header.stamp = rospy.Time(0)

            ps.pose.position.x = self.d['rad_cap']['captures'][ self.ind ][0]
            ps.pose.position.y = self.d['rad_cap']['captures'][ self.ind ][1]
            ang = self.d['rad_cap']['captures'][ self.ind ][2]
            q = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, math.radians( ang )))
            ps.pose.orientation = q

            self.ind += 1
            
            userdata.next_move_pose = ps
            return 'succeeded'
        else:
            return 'aborted' # done!

class YAMLprocMultitag(smach.State):
    def __init__(self, fname):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             output_keys = ['bagfile_name',
                                            'bagfile_topics',
                                            'tagid',
                                            'panrate',
                                            'tilt_left',
                                            'tilt_right',
                                            'tilt_rate',
                                            'tilt_block'])
        self.fname = fname
        self.d = None

    def execute(self, userdata):
        if not self.d:
            f = open( self.fname, 'r' )
            self.d = yaml.load( f )
            self.tagids = self.d['rad_cap']['tagids'].keys()
            self.keys_list = list( self.tagids )
            f.close()
            self.tilt_angs = []

        if not self.keys_list and not self.tilt_angs: # Done! reset.
            self.keys_list = list( self.tagids )
            return 'aborted'

        if not self.tilt_angs: # get next tagid
            self.tid = self.keys_list.pop()
            self.tilt_angs = list( self.d['rad_cap']['tagids'][self.tid]['tilts'] )
            rospy.logout( 'setting tid: %s' % self.tid )
            print 'Tilt angs: ', self.tilt_angs

        ta = self.tilt_angs.pop()[0]
        rospy.logout( 'Using tilt angle: %2.2f' % ta )
        userdata.tilt_left = -1.0 * ta
        userdata.tilt_right = ta
        userdata.tilt_rate = 0.4
        userdata.tilt_block = 1
        tad = math.degrees( ta )

        rospy.logout( 'YamlProcMultitag: issuing %d of %d for tag \'%s\'' %
                      (len(self.tagids) - len(self.keys_list), len(self.tagids), self.tid))

        userdata.tagid = self.tid

        tmp = self.tid
        path = self.d['rad_cap']['bagfile_path']
        tsec = str(int(rospy.Time.now().to_sec()))
        tads = str(int( tad ))
        userdata.bagfile_name = path + tsec + '_' + tads + '_' + tmp.replace(' ', '' )
        userdata.bagfile_topics = self.d['rad_cap']['bagfile_topics']
        userdata.panrate = 30.0
        return 'succeeded'


class ManualSkip(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succeeded', 'aborted'])
        self.init = None

    def execute(self, userdata):
        if not self.init:
            self.init = True
            self.pub = rospy.Publisher( 'robotsound', SoundRequest )
            rospy.sleep( 1.0 )

        self.pub.publish( SoundRequest( -3, 1, 'MANUAL or SKIP' ))
        ui = ''
        while not (ui == '0' or ui == '1'):
            print '\'0\' to Position Manually (position robot first)'
            print '\'1\' to skip this position'
            ui = raw_input()
        if ui == '0':
            return 'succeeded'  # Robot manually positioned
        else:
            return 'aborted'    # Forget this position

class MoveNotify(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succeeded', 'aborted'])
        self.init = None

    def execute(self, userdata):
        if not self.init:
            self.init = True
            self.pub = rospy.Publisher( 'robotsound', SoundRequest )
            rospy.sleep( 1.0 )

        self.pub.publish( SoundRequest( -3, 1, 'Ready to Move' ))
        ui = ''
        while not (ui == '0' or ui == '1'):
            print '\'0\' or \'1\' when Ready.'
            ui = raw_input()

        return 'succeeded'



if __name__ == '__main__':
    rospy.init_node( 'bag_capture_services' )
    
    bc = BagCap()
    rospy.spin()

    # rosservice call /bag_cap/capture '/tf' 'out'
    # rosservice call /bag_cap/capture '' ''

