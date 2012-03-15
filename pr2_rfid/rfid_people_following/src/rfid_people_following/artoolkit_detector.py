#! /usr/bin/python
# Taken from Tiffany & Kelsey and readapted to my needs.

import roslib
roslib.load_manifest('hrl_pr2_lib')
roslib.load_manifest('ar_pose')
roslib.load_manifest('rfid_people_following')
import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from rfid_people_following.srv import FloatFloatFloatFloat_Int32 as arm_srv
from ar_pose.msg import ARMarkers

import time
from threading import Thread
import numpy as np, math

def ctime():
    return rospy.Time.now().to_time()

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

        self.found_tag = False
        self.settling = False
        self.last_pos = [ 0.54, 0.0, 0.35 ]
        self.last_logout = ctime()

        self.hc = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction) # headclient
        self.hc.wait_for_server()
        
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/torso_lift_link', '/openni_rgb_optical_frame',
                                       rospy.Time(0), timeout = rospy.Duration(100) )

        self.artag_sub = rospy.Subscriber( '/artag_marker_handoff',
                                           ARMarkers,
                                           self.processARtag )
        self.abort = False
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
                self.last_pos = [ pbl.x, pbl.y, pbl.z ]
                if not self.found_tag:
                    self.hc.cancel_all_goals() # stop searching
                self.found_tag = True
            except:
                print pt
                rospy.logout( 'ARtagDetect: Transform failed' )
                return False
            
        else:
            if not self.found_tag and ctime() - self.last_logout > 3.0:
                rospy.logout( 'ARtag still undetected...' )
                self.last_logout = ctime()

    def scan( self ):
        rate = rospy.Rate(10)
        
        self.hc.send_goal( goal( 'torso_lift_link', [0.54, 0.2, 0.25], 2.0 ))
        self.hc.wait_for_result()
        rospy.logout('ARtagDetect: starting sweep')
        
        cmds = [ [0.54, -0.2, 0.25],
                 [0.54, 0.2, -0.10],
                 [0.54, -0.2, 0.0],
                 [0.54, 0.2, 0.20],
                 [0.54, -0.2, 0.30],
                 [0.54, 0.0, 0.35] ]

        q_rate = rospy.Rate(30)
        for cmd in cmds:
            self.hc.send_goal( goal( 'torso_lift_link', cmd, 6.0 ))
            self.hc.wait_for_result( rospy.Duration( 6.0 ))
            if self.found_tag or self.abort:
                break

        if not self.found_tag or self.abort:
            self.hc.cancel_all_goals() # just in case
            self.hc.send_goal( goal( 'torso_lift_link', [0.54, 0.0, 0.35], 2.0 ))
            self.hc.wait_for_result()
            return (False, 'torso_lift_link', self.last_pos)

        self.hc.cancel_all_goals() # just in case
        rate.sleep()

        rospy.logout('ARtagDetect: Tag detected!')
        self.settling = True

        settle = ctime()
        pos = list( self.last_pos )
        
        #while not rospy.is_shutdown():
        while ctime() - settle < 1.5 and not self.abort:
            self.hc.send_goal( goal( 'torso_lift_link', self.last_pos, 1.0 ))
            self.hc.stop_tracking_goal()
            if not np.allclose( pos, self.last_pos, rtol=0.15 ): # not settled within 5cm per axis?
                print 'Pos: ', pos, ' LastPos: ', self.last_pos
                settle = ctime()    # reset counter
                pos = list( self.last_pos ) # reset basis pose
            rate.sleep()

        self.hc.cancel_all_goals() # just in case
        rate.sleep()
        # return (success, link, pose)
        rospy.logout( 'Tag found!' )
        self.settling = False
        return (True, 'torso_lift_link', pos)


class ARthread( Thread ):
    def __init__( self ):
        Thread.__init__( self )
        self.should_run = True

        self.d = ARtagDetect()
        self.should_scan = False
        self.res = False, 'linkname', [1.0, 1.0, 1.0]
        self.success = False
        self.logtime = ctime()
        self.logtime1 = ctime()

        self.start()

    def run( self ):
        r = rospy.Rate( 10 )
        while self.should_run and not rospy.is_shutdown():
            if self.should_scan and not self.success:
                self.d.abort = False
                self.d.found_tag = False
                self.d.settling = False
                self.res = self.d.scan()
                self.success, link, pos = self.res
            r.sleep()

    def scan( self ):
        if ctime() - self.logtime > 3.0:
            self.logtime = ctime()
            rospy.logout('ARthread: started scanning')
        #self.res = False, 'linkname', [1.0, 1.0, 1.0]
        self.should_scan = True

    def found_tag( self ):
        return self.d.found_tag

    def settling( self ):
        return self.d.settling

    def abort( self ):
        if ctime() - self.logtime1 > 3.0:
            self.logtime1 = ctime()
            rospy.logout('ARthread: scanning aborted.')
        self.should_scan = False
        self.d.abort = True
            
    def stop( self ):
        self.should_run = False
        self.join(10)
        if (self.isAlive()):
            raise RuntimeError("ARNode: unable to stop thread")            

if __name__ == '__main__':
    ar = ARthread()
    print 'Starting scann'
    ar.scan()
    print 'Sleeping 10 sec.'
    time.sleep( 10.0 )
    ar.abort()
    print 'Aborted...'
    print 'Result: ', ar.res
    ar.stop()


# if __name__ == '__main__':
#     detector = ARtagDetect( )
#     print 'done'
#     rospy.wait_for_service( '/rfid_handoff/handoff_pos' )
#     print 'done1'
#     _hopos = rospy.ServiceProxy( '/rfid_handoff/handoff_pos', arm_srv )
#     hopos = lambda x,y,z,a: _hopos( x,y,z,a )

#     success, link, pos = detector.scan()
#     print 'Pos: ', pos
#     x,y,z = pos
#     ang = 0.0
#     if success == True:
#         # hacky! Pull the handoff point (wrist) 36cm closer to robot in xy-plane
#         r = np.sqrt( x ** 2.0 + y ** 2.0 )
#         theta = np.arctan2( y, x )
#         r = r - 0.36 
#         x_new = r * np.cos( theta )
#         print 'x_new: ', x_new
#         y_new = r * np.sin( theta )
#         hopos( x_new, y_new, z + 0.20, ang )
