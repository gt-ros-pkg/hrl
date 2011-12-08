#!/usr/bin/python

import roslib
roslib.load_manifest( 'robotis' )
roslib.load_manifest( 'hrl_rfid' )
roslib.load_manifest( 'geometry_msgs' )
import rospy
import robotis.ros_robotis as rr
import hrl_rfid.ros_M5e_client as rmc
from geometry_msgs.msg import PointStamped

import time
import math
from threading import Thread
import optparse

class TagGroundTruth( Thread ):
    def __init__( self, tag_x, tag_y, tag_z, baseframe = '/map', date = '' ):
        Thread.__init__( self )
        self.should_run = True

        self.tag_gt = PointStamped()
        self.tag_gt.header.stamp = rospy.Time(0)
        self.tag_gt.header.frame_id = baseframe
        self.tag_gt.point.x = tag_x
        self.tag_gt.point.y = tag_y
        self.tag_gt.point.z = tag_z

        rospy.logout('Publishing tag groundtruth at <%4.3f, %4.3f, %4.3f> in frame %s' % ( tag_x, tag_y, tag_z, baseframe ))
        self.pub = rospy.Publisher( 'tag_gt/' + date, PointStamped )
        self.start()

    def run( self ):
        rate = rospy.Rate( 5 )
        while self.should_run and not rospy.is_shutdown():
            self.tag_gt.header.stamp = rospy.Time.now()
            self.pub.publish( self.tag_gt )
            rate.sleep()
            
    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("Unable to stop thread")
        



if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-x', action='store', type='float', dest='x', default=None,
                 help='tag groundtruth pose in x')
    p.add_option('-y', action='store', type='float', dest='y', default=None,
                 help='tag groundtruth pose in y')
    p.add_option('-z', action='store', type='float', dest='z', default=None,
                 help='tag groundtruth pose in z')
    p.add_option('-d', action='store', type='string', dest='date', default=None,
                 help='date (yyyymmdd)')
    opt, args = p.parse_args()

    rospy.init_node( 'flapper' )

    if opt.x == None or opt.y == None or opt.z == None or opt.date == None:
        print 'Requires <x,y,z> and date arguments!'
        exit()

    p_left = rr.ROS_Robotis_Client( 'left_pan' )
    t_left = rr.ROS_Robotis_Client( 'left_tilt' )
    
    p_right = rr.ROS_Robotis_Client( 'right_pan' )
    t_right = rr.ROS_Robotis_Client( 'right_tilt' )

    EX_1 = 1.350
    EX_2 = 0.520  # 30 deg.
#    EX_2 = -1.000 

    tgt = TagGroundTruth( opt.x, opt.y, opt.z, '/map', opt.date )

    p_left.move_angle( -1.0 * EX_1, math.radians(10), blocking = False )
    t_left.move_angle( 0.0, math.radians(10), blocking = False )
    
    p_right.move_angle( EX_1, math.radians(10), blocking = False )
    t_right.move_angle( 0.0, math.radians(10), blocking = False )

    forward = True

    while not rospy.is_shutdown():
        if not p_left.is_moving() and not p_right.is_moving():
            if forward:
                p_left.move_angle( -1.0 * EX_1, math.radians(30), blocking = False )
                p_right.move_angle( EX_1, math.radians(30), blocking = False )
                forward = False
            else:
                p_left.move_angle( EX_2, math.radians(30), blocking = False )
                p_right.move_angle( -1.0 * EX_2, math.radians(30), blocking = False )
                forward = True
        time.sleep(0.1)
    tgt.stop()
