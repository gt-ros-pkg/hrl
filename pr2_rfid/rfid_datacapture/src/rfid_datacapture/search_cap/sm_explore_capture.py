#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
import rospy

import smach
import actionlib

from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from rfid_demos import sm_rfid_explore
from rfid_behaviors import recorder
from hrl_lib import util
from rfid_datacapture.srv import BagCapture, BagCaptureRequest

class DummyClass():
    def __init__(self, tagid):
        self.tagid = tagid

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--fname', action='store', type='string', dest='fname',
                 help='File name.  Should be without extension. [eg. \'trial\']', default='')
    p.add_option('--radius', action='store', type='float', dest='radius',
                 help='Exploration radius in meters.', default=4.0)

    opt, args = p.parse_args()

    if opt.fname == '':
        print 'Fname required'
        exit()
    fname_base = '/u/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/search_aware_home/'
    fname = fname_base + opt.fname



    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'bagfile_name',
                                           'bagfile_topics',
                                           'tagid',
                                           'explore_radius' ])
    with sm:
        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'SEARCH'})

        sm_search = sm_rfid_explore.sm_search()

        smach.StateMachine.add(
            'SEARCH',
            sm_search,
            transitions = {'succeeded':'STOP_BAG_CAPTURE'},
            remapping = {'tagid':'tagid',
                         'explore_radius':'explore_radius'})

        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'succeeded'})



    rospy.init_node('smach_datacap_rfid_explore')

    rec = recorder.Recorder( serv_name = 'temp_recorder', node_name = 'temp_recorder_py' )
    rec.process_service( None )  # start recording


    sm.userdata.tagid = ''
    sm.userdata.explore_radius = opt.radius
    sm.userdata.bagfile_name = fname
    sm.userdata.bagfile_topics = '/tf /visarr /rfid/ears_reader /rfid/ears_reader_arr /map /robot_pose_ekf/odom_combined'

    outcome = sm.execute()

    rec.process_service( None )  # stop recording

    print 'Saving recorder pickle data.'
    util.save_pickle( rec.recorder_data, fname + '_reads.pkl' )

    print 'Saving best read locations.'
    tagids = ['OrangeMedBot','TravisTVremo','RedBottle   ',
              'OnMetalKeys ','WhiteMedsBot','BlueMedsBox ',
              'TeddyBearToy','CordlessPhon','BlueHairBrus']

    for t in tagids:
        print '\tTagid: \'%s\'' % t,
        tname = t.replace( ' ', '' )

        try:
            pos = rec.bestvantage( DummyClass( t ))
            pos.header.stamp = rospy.Time(0)
            dat = pos.__str__() + '\n'

            f = open( fname + '_tag_' + tname + '.yaml', 'w' )
            f.write( dat )
            f.close()
            print ' Done.'
        except:
            print ' NO READS.  Done.'
            pass
    
