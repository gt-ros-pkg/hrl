#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('smach_ros')
roslib.load_manifest('actionlib')
roslib.load_manifest('rfid_datacapture')
import rospy

import smach
import actionlib

from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from rfid_datacapture.srv import BagCapture, BagCaptureRequest

def head_capture( ):
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'bagfile_name', 'bagfile_topics' ])
    with sm:

        def PointAdd( x, y, z, dur, state, res ):
            pgoal = PointHeadGoal()
            pgoal.target.header.frame_id = '/torso_lift_link'
            pgoal.target.point.x = x
            pgoal.target.point.y = y
            pgoal.target.point.z = z
            pgoal.min_duration = rospy.Duration( dur )
            pgoal.max_velocity = 1.0
            smach.StateMachine.add(
                state,
                SimpleActionState( '/head_traj_controller/point_head_action',
                                   PointHeadAction,
                                   goal = pgoal ),
                transitions = { 'succeeded' : res })
            return

        PointAdd( 0.00, -1.00, -0.60, 5.0, 'PH1', 'START_BAG_CAPTURE' )

        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'PH2'})
        
        
        PointAdd( 0.00, 1.00, -0.60, 15.0, 'PH2', 'PH3' )
        PointAdd( 0.00, 1.00, -0.20, 3.0, 'PH3', 'PH4' )
        PointAdd( 0.00, -1.00, -0.20, 15.0, 'PH4', 'PH5' )
        PointAdd( 0.00, -1.00, 0.30, 3.0, 'PH5', 'PH6' )
        PointAdd( 0.00, 1.00, 0.30, 15.0, 'PH6', 'PH7' )
        PointAdd( 1.00, 0.00, 0.00, 7.5, 'PH7', 'STOP_BAG_CAPTURE' )

        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'succeeded'})

        return sm


class DelayState( smach.State ):
    def __init__( self, delay = 3.0 ):
        smach.State.__init__(self,outcomes=['succeeded', 'aborted'])
        self.delay = delay

    def execute( self, userdata ):
        rospy.sleep( self.delay )
        return 'succeeded'
    

def cam_capture( ):
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                            input_keys = [ 'bagfile_name', 'bagfile_topics' ])
    with sm:

        smach.StateMachine.add(
            'START_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request_slots = ['topics','dest'] ),
            remapping = {'topics':'bagfile_topics',
                         'dest':'bagfile_name'},
            transitions = {'succeeded':'DELAY'})

        smach.StateMachine.add(
            'DELAY',
            DelayState(),
            transitions = {'succeeded':'STOP_BAG_CAPTURE'})

        smach.StateMachine.add(
            'STOP_BAG_CAPTURE',
            ServiceState( '/bag_cap/capture',
                          BagCapture,
                          request = BagCaptureRequest('','') ),
            transitions = {'succeeded':'succeeded'})

        return sm
    

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--fname', action='store', type='string', dest='fname',
                 help='File name.  Should be without extension. [eg. \'trial\']', default='')

    opt, args = p.parse_args()

    if opt.fname == '':
        print 'Fname required'
        exit()
    
    rospy.init_node('smach_head_capture')

    sm = head_capture()

    sis = IntrospectionServer('sm_head_capture', sm, '/SM_HEAD_CAPTURE')
    sis.start()

    fname_base = '/u/travis/svn/robot1/src/projects/rfid_datacapture/src/rfid_datacapture/search_cap/search_aware_home/'
    fname = fname_base + opt.fname
    
    sm.userdata.bagfile_name = fname
    #sm.userdata.bagfile_topics = '/tf /kinect_head/rgb/points_throttled /kinect_head/rgb/image_color'
    sm.userdata.bagfile_topics = '/tf /kinect_head/rgb/points_throttled'

    outcome = sm.execute()

    #raw_input( 'Hit [ENTER] to begin capturing camera.' )

    sm_cam = cam_capture()
    sm_cam.userdata.bagfile_name = fname + '_cam'
    sm_cam.userdata.bagfile_topics = '/tf /kinect_head/rgb/image_color'

    sm_cam.execute()

    sis.stop()

    

