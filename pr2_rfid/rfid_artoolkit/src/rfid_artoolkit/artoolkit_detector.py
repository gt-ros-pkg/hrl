#! /usr/bin/python

import roslib
roslib.load_manifest('rfid_artoolkit')
import rospy

import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from ar_pose.msg import ARMarkers

import time
from threading import Thread, Lock
import numpy as np, math
from collections import deque

def ctime():
    return rospy.Time.now().to_time()

def retfalse():
    return False

def goal( frame, pos, d ):
    g = PointHeadGoal()
    g.target.header.frame_id = frame
    g.target.point.x = pos[0]
    g.target.point.y = pos[1]
    g.target.point.z = pos[2]
    g.min_duration = rospy.Duration( d )
    # Note, looking @ head action source, it seems pointing_frame unimplemented...
    #   Just account with a static offset (for now) where desired.
    return g

class ARtagDetect():
    def __init__( self ):
        try:
            rospy.init_node('ar_detect_tracker')
        except: # Parent already initialized
            pass

        rospy.logout('ARtagDetect: Initializing.')

        self.lock = Lock() # for last_pos & found_tag
        self.found_tag = False
        self.init_pos = [ 0.54, 0.0, 0.35 ]
        self.last_pos = [ 0.54, 0.0, 0.35 ]

        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/torso_lift_link', '/openni_rgb_optical_frame',
                                       rospy.Time(0), timeout = rospy.Duration(100) )

        self.artag_sub = rospy.Subscriber( '/artag_marker_handoff',
                                           ARMarkers,
                                           self.processARtag )
        rospy.logout('ARtagDetect: Ready and Running.')

    def processARtag( self, msg ):
        if msg.markers != []:
            frame = msg.markers[0].header.frame_id
            p = msg.markers[0].pose.pose.position

            pt = PointStamped()
            pt.header.frame_id = frame
            pt.point.x = p.x
            pt.point.y = p.y 
            pt.point.z = p.z
            pt.header.stamp = rospy.Time(0)

            try:
                pt_bl = self.listener.transformPoint( '/torso_lift_link', pt )
                pbl = pt_bl.point
                # pointing_frame manual offset (should actually be on p.y,
                #   but this is a good approximation with less noise in robot frame).
                pbl.z = pbl.z - 0.2

                self.lock.acquire()
                self.last_pos = [ pbl.x, pbl.y, pbl.z ]
                self.found_tag = True
                self.lock.release()
            except:
                rospy.logout( 'ARtagDetect: Transform failed' )

    def init_new_detection( self ):
        self.lock.acquire()
        self.last_pos = self.init_pos
        self.found_tag = False
        self.lock.release()

    def found_new_tag( self ):
        self.lock.acquire()
        rv = ( self.found_tag, self.last_pos, '/torso_lift_link' )
        self.lock.release()
        return rv


class ARtagScan():
    def __init__( self ):
        try:
            rospy.init_node('artag_scanner')
        except: # Parent already initialized
            pass

        rospy.logout( 'ARtagScan: Initializing' )

        self.detector = ARtagDetect()
        
        self.hc = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction) # headclient
        self.hc.wait_for_server()
        
        rospy.logout( 'ARtagScan: Ready and Running.' )

    def scan( self, preempt_func = retfalse ):
        # self.hc.send_goal( goal( 'torso_lift_link', [0.54, 0.2, 0.25], 2.0 ))
        # self.hc.wait_for_result( rospy.Duration( 2.0 ))

        rospy.logout('ARtagScan: starting sweep')
        
        cmds = [ [0.54, -0.2, 0.25],
                 [0.54, 0.2, -0.10],
                 [0.54, -0.2, 0.0],
                 [0.54, 0.2, 0.20],
                 [0.54, -0.2, 0.30],
                 [0.54, 0.0, 0.35] ]

        self.detector.init_new_detection()

        found_tag, pos, frame = self.detector.found_new_tag()
        for cmd in cmds:
            if not rospy.is_shutdown() and not found_tag and not preempt_func():
                self.hc.send_goal( goal( 'torso_lift_link', cmd, 6.0 ))
                found_tag, pos, frame = self.detector.found_new_tag()

                goal_done = False
                while not rospy.is_shutdown() and not goal_done and not found_tag and not preempt_func():
                    found_tag, pos, frame = self.detector.found_new_tag()
                    goal_done = self.hc.wait_for_result( rospy.Duration( 0.05 )) # ~20Hz

                self.hc.cancel_all_goals() # just in case

        if preempt_func():
            rospy.logout( 'ARtagScan: Preempt was requested. May not have finished.' )
            rospy.logout( 'ARtagScan: Exiting' )
            return ( False, found_tag, pos, frame )
        else:
            rospy.logout( 'ARtagScan: Exiting' )
            return ( True, found_tag, pos, frame )


    def settle( self, init_pos, init_frame, preempt_func = retfalse ):
        rospy.logout('ARtagScan: Starting settle. Looking at detection')

        # Look in direction of initial read
        self.hc.send_goal( goal( init_frame, init_pos, 2.0 ))
        self.hc.wait_for_result( rospy.Duration( 2.0 ))
        
        buff = deque()
        buff.append( init_pos )

        rospy.sleep( 0.3 )
        settle = 5.0
        stime = ctime()

        rospy.logout('ARtagScan: Averaging scans')

        # Refine the estimate
        found_tag = False
        self.detector.init_new_detection()
        self.hc.send_goal( goal( init_frame, init_pos, 0.5 ))
        goal_reached = False
        r = rospy.Rate( 20 )
        while not rospy.is_shutdown() and ctime() - stime < settle:
            #print 'Times: ', ctime(), stime
            found_tag, pos, frame = self.detector.found_new_tag()
            if found_tag:
                buff.append( pos )
                found_tag = False
                self.detector.init_new_detection()

            goal_reached = self.hc.wait_for_result( rospy.Duration( 0.02 ))
            if goal_reached:
                mean_pos = np.mean( buff, axis = 0 )
                self.hc.send_goal( goal( init_frame, mean_pos, 0.5 ))

            r.sleep()

        self.hc.wait_for_result( rospy.Duration( 2.0 ))
        mean_pos = np.mean( buff, axis = 0 )
        return ( True, mean_pos, init_frame )


def testDetect():
    rospy.init_node( 'testDetect' )
    rate = rospy.Rate( 1.0 )
    detector = ARtagDetect()
    detector.init_new_detection()

    found_tag, pos, frame = detector.found_new_tag()

    while not rospy.is_shutdown():
        found_tag, pos, frame = detector.found_new_tag()
        print found_tag, pos, frame
        rate.sleep()

def testScan():
    rospy.init_node( 'testScan' )
    rate = rospy.Rate( 1.0 )
    scanner = ARtagScan()

    rv = scanner.scan()
    print 'scanner res: ', rv
    success, found_tag, pos, frame = rv

    rv = scanner.settle( pos, frame )
    print 'settle res: ', rv


if __name__ == '__main__':
    #testDetect()
    testScan()
