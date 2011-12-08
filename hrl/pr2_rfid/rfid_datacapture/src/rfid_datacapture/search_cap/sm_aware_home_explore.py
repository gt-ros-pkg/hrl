#! /usr/bin/python
import roslib
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
roslib.load_manifest('rfid_demos')
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('hrl_lib')
import rospy


tdb = { 0: ['OrangeMedBot',[]],
        1: ['TravisTVremo',[]],
        2: ['RedBottle   ',[]],
        3: ['OnMetalKeys ',[]],
        4: ['WhiteMedsBot',[]],
        5: ['BlueMedsBox ',[]],
        6: ['TeddyBearToy',[]],
        7: ['CordlessPhon',[]],
        8: ['BlueHairBrus',[]]}

pts = { 0: ['BehindTree',[3.757, 6.017, 0.036]],
        1: ['FireplaceMantle',[5.090, 4.238, 1.514]],
        2: ['CircleEndTable',[5.399, 2.857, 0.607]],
        3: ['Couch',[3.944, 1.425, 0.527]],
        4: ['RectEndTable',[3.302, 0.932, 0.534]],
        5: ['BehindKitchenTable',[-0.339, -2.393, 0.793]],
        6: ['NearDishwaser',[-1.926, -0.835, 0.946]],
        7: ['InCupboard',[-3.257, 1.294, 1.397]],
        8: ['OnFilingCabinet',[-0.083, 2.332, 0.670]]}

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
    fname_base = '/u/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/search_bags/'
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
    tagids = ['person      ','OrangeMedBot' ,'SpectrMedBot','OnMetalKeys ',
              'TravisTVremo','Red Mug     ']

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
            print ' No reads.  Done.'
            pass
    
