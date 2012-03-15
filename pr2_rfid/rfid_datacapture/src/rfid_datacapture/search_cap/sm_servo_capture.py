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
from rfid_demos import sm_rfid_servo_approach

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--fname', action='store', type='string', dest='fname',
                 help='File name.  Should be without extension. [eg. \'trial\']', default='')

    p.add_option('--tag', action='store', type='string', dest='tagid',
                 help='Tagid to approach', default='person      ')
    opt, args = p.parse_args()

    if opt.fname == '':
        print 'Fname required'
        exit()
    fname_base = '/u/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/search_aware_home/'
    fname = fname_base + opt.fname

    print 'SERVO APPROACH to ID: \'%s\'' % (opt.tagid)

    rospy.init_node('smach_servo_datacapture')

    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'bagfile_name',
                                           'bagfile_topics',
                                           'tagid'])
    with sm:
        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'SERVO'})

        sm_servo = sm_rfid_servo_approach.sm_rfid_servo_approach()

        smach.StateMachine.add(
            'SERVO',
            sm_servo,
            transitions = {'succeeded':'STOP_BAG_CAPTURE'},
            remapping = {'tagid':'tagid'})

        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'succeeded'})

    sm.userdata.tagid = opt.tagid
    sm.userdata.bagfile_name = fname + '_servo'
    sm.userdata.bagfile_topics = '/tf /rfid/ears_reader /rfid/ears_reader_arr /map /robot_pose_ekf/odom_combined /navigation/cmd_vel'

    outcome = sm.execute()

