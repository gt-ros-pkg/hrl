#! /usr/bin/python

# Without the in-hand RFID reader, use ARToolKit instead for upclose
# (nearfield) validation

import roslib
roslib.load_manifest('rfid_artoolkit')
import rospy

import rfid_servoing.servo as servo
import costmap_services.python_client as costmap
import artoolkit_detector as artag
import actionlib

from rfid_artoolkit.srv import ARServoSrv, ARServoSrvResponse
from rfid_artoolkit.msg import UpCloseAction, UpCloseResult, UpCloseGoal

class ARservoScan():
    # Uses ARtagScan along with costmap functionality (ie. in rfid_servoing) to
    # determine when to preempt the ARtagScan and continue servoing.

    # This class is used to keep the artoolkit_detector somewhat decoupled from all this other jazz.
    def __init__( self ):
        try:
            rospy.init_node('artag_superclass')
        except: # Parent already initialized
            pass

        rospy.logout( 'ARservoScan: Initializing' )

        self.scanner = artag.ARtagScan()

        # Create Costmap Services obj
        self.cs = costmap.CostmapServices( accum = 15 )
        self._service = rospy.Service( '/rfid_artoolkit/upclose', ARServoSrv, self.service_request )
        self._as = actionlib.SimpleActionServer( '/rfid_artoolkit/upclose_act',
                                                 UpCloseAction, execute_cb = self.action_request )
        self._as.start()

        # Note: After moving, this will require accum * -1's before stopping.

        rospy.logout( 'ARservoScan: Ready and Running.' )


    def service_request( self, req ):
        success, found_tag, pos, frame = self.scan( req.tagid )
        
        rv = ARServoSrvResponse()
        rv.success = success and found_tag # not preempted & found tag
        if success and found_tag:
            rv.ps.header.frame_id = frame
            rv.ps.header.stamp = rospy.Time.now()
            rv.ps.point.x = pos[0]
            rv.ps.point.y = pos[1]
            rv.ps.point.z = pos[2]

        return rv

    
    def action_request( self, goal ):
        rospy.logout( 'ARservoScan: action_request received for tagid: \'%s\'' % goal.tagid )

        zc = servo.ZedCalc( filt_len = 5, tag_id = goal.tagid )

        def free_to_move_preempt():
            # Note: I believe the variables are scoped at time of func def
            zed_next = zc.next_pub() # Where does RFID want to go?
            costmap_rv = self.cs.scoreTraj_NegHyst( 0.1, 0.0, zed_next )
            #print zed_next
            #print costmap_rv
            if costmap_rv == -1.0:
                #rospy.logout( 'Obstacle still present.' )
                return False # Still keep scaning
            else:
                rospy.logout( 'ARservoScan: Obstacle Gone... PREEMPT!' )
                return True # Whatever was in way is gone. Should go back to servoing.

        def actionlib_preempt():
            check = self._as.is_preempt_requested()
            if check:
                rospy.logout( 'ARservoScan: actionlib Preempt requested.  PREEMPT!' )
            return check

        def preempt_func():
            return free_to_move_preempt() or actionlib_preempt()
        
        success, found_tag, pos, frame = self.scanner.scan( preempt_func = preempt_func )
        # Success will be false if it was preempted (eg. preempt_func returns True)

        zc.stop() # Stop the RFID reader...

        if success and found_tag:
            found_tag, pos, frame = self.scanner.settle( pos, frame )

        if success:  # Weren't preempted
            if found_tag:
                status = 'SUCCEEDED'
            else:
                status = 'FAILED'
        else:
            if actionlib_preempt(): # preempt caused by actionlib
                status = 'PREEMPTED'
            else: # obstacle is clear
                status = 'RESERVO'
                
        rospy.logout( 'ARservoScan: Scan completed.' )

        rv = UpCloseResult()
        rv.success = success and found_tag # not preempted & found tag
        rv.status = status
        if success and found_tag:
            rv.ps.header.frame_id = frame
            rv.ps.header.stamp = rospy.Time.now()
            rv.ps.point.x = pos[0]
            rv.ps.point.y = pos[1]
            rv.ps.point.z = pos[2]
        
        self._as.set_succeeded( rv )


    def scan( self, tag_id ):
        # Note: action_request does pretty much the same thing, but with preemption.

        rospy.logout( 'ARservoScan: Scan requested for tagid \'%s\'' % tag_id )
        # Subscribe to RFID reader to figure out where it wants to go.
        zc = servo.ZedCalc( filt_len = 5, tag_id = tag_id )

        def preempt_func():
            # Note: I believe the variables are scoped at time of func def
            zed_next = zc.next_pub() # Where does RFID want to go?
            costmap_rv = self.cs.scoreTraj_NegHyst( 0.1, 0.0, zed_next )
            #print zed_next
            #print costmap_rv
            if costmap_rv == -1.0:
                #rospy.logout( 'Obstacle still present.' )
                return False # Still keep scaning
            else:
                rospy.logout( 'ARservoScan: Obstacle Gone... PREEMPT!' )
                return True # Whatever was in way is gone. Should go back to servoing.
        
        success, found_tag, pos, frame = self.scanner.scan( preempt_func = preempt_func )
        # Success will be false if it was preempted (eg. preempt_func returns True)

        zc.stop() # Stop the RFID reader...

        if success and found_tag:
            found_tag, pos, frame = self.scanner.settle( pos, frame )

        rospy.logout( 'ARservoScan: Scan completed.' )
        #print 'scan result: ', success, found_tag, pos, frame
        return ( success, found_tag, pos, frame )


def testScan():
    rospy.init_node( 'testScan' )

    servoScan = ARservoScan()
    success, found_tag, pos, frame = servoScan.scan( 'person      ' )
    print 'results: ', success, found_tag, pos, frame

if __name__ == '__main__':

    ss = ARservoScan()
    rospy.spin()


    # There are (at least) three ways to call this code.
    # 1) Python:
    
    # testScan()

    # 2) ROS service:

    # rosservice call /rfid_artoolkit/upclose "'person      '"

    # 3) ROS actionlib:

    # try:
    #     client = actionlib.SimpleActionClient( '/rfid_artoolkit/upclose_act', UpCloseAction )
    #     client.wait_for_server()
    #     client.send_goal( UpCloseGoal( 'person      ' ))
    #     # Test Preempt
    #     # rospy.sleep( 10 )
    #     # client.cancel_all_goals()
    #     client.wait_for_result()
    #     print client.get_result()

    # except rospy.ROSInterruptException:
    #     print 'Program interrupted before completion'

    # rospy.spin()

    
