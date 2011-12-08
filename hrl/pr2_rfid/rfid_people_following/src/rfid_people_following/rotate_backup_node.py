#! /usr/bin/python
import time
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest( 'move_base_msgs' )
roslib.load_manifest('tf')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('std_msgs')
roslib.load_manifest('hrl_rfid')
roslib.load_manifest('robotis')
roslib.load_manifest('rfid_people_following')
import rospy

import tf
import tf.transformations as tft
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

from rfid_people_following.srv import FloatFloat_Int32
from rfid_people_following.srv import FloatFloat_Int32Response
from rfid_people_following.srv import FloatFloatFloatFloat_Int32

import numpy as np, math
import time
from threading import Thread
from collections import deque

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi


class RotateBackup():
    def __init__( self, service_name = '/rotate_backup' ):
        try:
            rospy.init_node( 'rotater' )
        except:
            pass
        rospy.logout( 'rotate_backup: Initializing service: \'%s\'' % service_name )
        self.pub = rospy.Publisher( '/move_base_simple/goal', PoseStamped )
        self.pub_direct = rospy.Publisher( '/navigation/cmd_vel', Twist )
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('/odom_combined', '/base_link',
                                       rospy.Time(0), timeout = rospy.Duration(100) )
        self._service_rb = rospy.Service( service_name, FloatFloat_Int32, self.move)
        self._service_rb_navstack = rospy.Service( service_name+'/navstack', FloatFloatFloatFloat_Int32, self.navstack)
        rospy.logout( 'rotate_backup: Service ready' )

    def non_nav_rotate( self, r ):
        success, pose, orient = self.get_pose() # orient = rx, ry, rz
        if not success:
            rospy.logout( 'rotate_backup: Rotate transform fail. Exiting.' )
            return
        
        t_rz = standard_rad( orient[-1] + r )
        mov = Twist()
        mov.linear.x = 0.05
        mov.angular.z = 0.6 * np.sign( r )

        rate = rospy.Rate( 10 )
        while not np.allclose( [t_rz], [orient[-1]], atol=[0.08] ): # Not within 5deg of target
            success, pose, orient = self.get_pose() # orient = rx, ry, rz
            orient[-1] = standard_rad( orient[-1] )
            if not success:
                rospy.logout( 'rotate_backup: Rotate transform fail. Exiting.' )
                return
            self.pub_direct.publish( mov )
            rate.sleep()


    def non_nav_backup( self, d ):
        # Hacky way to backup without relying on nav_stack's bizarre circling.
        mov = Twist()
        mov.linear.x = 0.1 * np.sign( d )
        t0 = time.time()
        rate = rospy.Rate( 10.0 )
        while time.time() - t0 < np.abs(d) / 0.1:
            self.pub_direct.publish( mov )
            rate.sleep()

    def get_pose( self ):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time(0)
        ps.header.frame_id = '/base_link'
        #ps.pose.position.x = d
        
        try:
            ps_odom = self.listener.transformPose( '/odom_combined', ps )
        except:
            rospy.logout( 'rotate_backup: Failed transform #1.' )
            time.sleep( 2.0 )
            return False, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
            
        orient = ps_odom.pose.orientation
        rx, ry, rz = tft.euler_from_quaternion([ orient.x, orient.y, orient.z, orient.w ])
        pose = [ ps_odom.pose.position.x, ps_odom.pose.position.y, ps_odom.pose.position.z ]
        return True, pose, [rx, ry, rz]
        
# Hacky version not using actionlib
#     def wait_for_stop( self, duration = 0.5 ):
#         rospy.logout( 'rotate_backup: Waiting for movement to stop.' )
#         t0 = time.time()
#         rate = rospy.Rate( 10 )
#         success, pose, orient = self.get_pose()        
#         if not success:
#             rospy.logout( 'rotate_backup: Waiting 2 sec.' )
#             time.sleep( 2.0 )
#             return

#         sp = np.array([ pose[0], pose[1], pose[2], orient[-1] ])

#         while time.time() - t0 < duration:
#             success, pose, orient = self.get_pose()
#             if not success:
#                 rospy.logout( 'rotate_backup: Waiting 2 sec.' )
#                 time.sleep( 2.0 )
#                 return
#             qp = np.array([ pose[0], pose[1], pose[2], orient[-1] ])
#             if not np.allclose( sp, qp, atol=[0.01, 0.01, 0.01, 0.005] ):
#                 t0 = time.time()
#                 sp = qp
#             rate.sleep()
#         return
            
    def move( self, request ):
        r = request.rotate
        d = request.displace
        rospy.logout( 'rotate_backup: Asked to rotate: %3.2f (deg)' % math.degrees(r))
        rospy.logout( 'rotate_backup: Asked to translate (forward-backward): %3.2f (m)' % d)

        self.non_nav_backup( d )
        self.non_nav_rotate( r )

#         success, pose, orient = self.get_pose()        
#         if not success:
#             return FloatFloat_Int32Response( int(False) )

#         new_point = Point( pose[0], pose[1], pose[2] )
#         old_rx, old_ry, old_rz = orient
#         new_orient = tft.quaternion_from_euler( old_rx, old_ry, old_rz + r  )
#         new_quat = Quaternion( *new_orient )

#         new_ps = PoseStamped()
#         new_ps.header.stamp = rospy.Time(0)
#         new_ps.header.frame_id = '/odom_combined'
#         new_ps.pose.position = new_point
#         new_ps.pose.orientation = new_quat

#         self.pub.publish( new_ps )
#         self.wait_for_stop()
#         rospy.logout( 'rotate_backup: Done with call.' )
        return FloatFloat_Int32Response( int(True) )

# Hacky version not using actionlib
#     def navstack( self, request ):
#         new_orient = tft.quaternion_from_euler( 0.0, 0.0, request.ang  ) # rx, ry, rz
#         new_quat = Quaternion( *new_orient )

#         new_ps = PoseStamped()
#         new_ps.header.stamp = rospy.Time(0)
#         new_ps.header.frame_id = '/map'
#         new_ps.pose.position.x = request.x
#         new_ps.pose.position.y = request.y
#         new_ps.pose.orientation = new_quat

#         rospy.logout( 'rotate_backup: Requesting navstack move to <x,y,ang-deg> %3.3f %3.3f %3.3f.' % (request.x, request.y, math.degrees(request.ang)) )
#         self.pub.publish( new_ps )

#         rospy.logout( 'rotate_backup: Waiting for base to stop moving.' )
#         self.wait_for_stop( 7.0 )
#         return int( True )


    def navstack( self, request ):
        rospy.logout( 'rotate_backup: Requesting navstack move to <x,y,ang-deg> %3.3f %3.3f %3.3f.' % (request.x, request.y, math.degrees(request.ang)) )

        client = actionlib.SimpleActionClient( 'move_base', MoveBaseAction )
        client.wait_for_server()

        ps = PoseStamped()
        ps.header.frame_id = '/map'
        ps.header.stamp = rospy.Time(0)
        ps.pose.position.x = request.x
        ps.pose.position.y = request.y
        ps.pose.orientation = Quaternion( *tft.quaternion_from_euler( 0.0, 0.0, request.ang ))

        goal = MoveBaseGoal( ps )
        client.send_goal( goal )
        rospy.logout( 'rotate_backup: Waiting for base to stop moving.' )
        client.wait_for_result()
        return int( True )
        

class RotateBackupClient():
    def __init__( self, service_name = '/rotate_backup' ):
        rospy.logout( 'rotate_backup_client: Waiting for service: \'%s\'' % service_name )
        rospy.wait_for_service( service_name )
        rospy.logout( 'rotate_backup_client: Service ready.' )
        
        self._rb_service = rospy.ServiceProxy( service_name, FloatFloat_Int32 )

    def rotate_backup( self, rotate, displace ):
        return self._rb_service( rotate, displace )

if __name__ == '__main__':
    rb = RotateBackup()
    rospy.spin()
