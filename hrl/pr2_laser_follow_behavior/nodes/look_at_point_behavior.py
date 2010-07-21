#!/usr/bin/python
import roslib
roslib.load_manifest("pr2_laser_follow_behavior")
import rospy

import numpy as np
import math

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String
from move_base_msgs.msg import *

import actionlib
import tf
import laser_interface.camera as cam
import hrl_lib.tf_utils as tfu
from threading import RLock


def in_bounds(p2d, xlim, ylim):
    return (xlim[0] <= p2d[0,0]) and (p2d[0,0] <= xlim[1]) \
            and (ylim[0] <= p2d[1,0]) and (p2d[1,0] <= ylim[1])

class LookAtBehavior:

    def __init__(self, camera_root_topic):
        #self.wait = False
        #self.point3d = None
        self.state = 'ready'
        self.lock = RLock()
        self.lock.acquire()
        self.message = None 
        self.STATES = {'ready':'ready',                      # none
                       'head_turn': 'head_turn',             # something
                       #'head_turn_drive': 'head_turn_drive', # something
                       'driving': 'driving'}                 # something

        rospy.init_node('look_at_point_behavior', anonymous=True)
        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.point_pub = rospy.Publisher('cursor3dcentered', PointStamped)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.move_base_double_click)
        self.double_click2 = rospy.Subscriber('mouse_left_double_click', String, self.cancel_move_base_double_click)
        self.camera_model = cam.ROSStereoCalibration('/' + camera_root_topic + '/left/camera_info' , 
                                                     '/' + camera_root_topic + '/right/camera_info')
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
        #self.head_client.wait_for_server()
        self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.base_client.wait_for_server()
        #self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
        self.move_pub = rospy.Publisher('look_at_point_goal', PoseStamped)
        #self.move_pub2 = rospy.Publisher('hai_constant', PoseStamped)
        self.tflistener = tf.TransformListener()
        self.lock.release()
        print 'running'


    def move_base_double_click(self, a_str):
        if self.message == None:
            rospy.logwarn('Unable to go, no message heard.')
            return
        else:
            self.lock.acquire()
            self.state = self.STATES['driving']
            #Looking at the point last clicked on... (maybe keep doing this as we drive?)
            #self.look_at(self.message)

            #Move base
            self.move_base(self.message)
            self.message = None
            self.state = self.STATES['ready']
            self.lock.release()

    def transform_point(self, point_stamped):
        point_head = point_stamped.point

        #Tranform into base link
        target_link = '/base_link'
        base_T_head = tfu.transform(target_link, point_stamped.header.frame_id, self.tflistener)
        point_mat_head = tfu.translation_matrix([point_head.x, point_head.y, point_head.z])
        point_mat_base = base_T_head * point_mat_head
        t_base, o_base = tfu.matrix_as_tf(point_mat_base)

        #Calculate angle robot should face
        angle = math.atan2(t_base[1], t_base[0])
        q = tf.transformations.quaternion_from_euler(0, 0, angle)
        return (t_base, q, target_link)

        
    def move_base(self, point, wait=False):
        t_base, q, target_link = point
        ps = PoseStamped()
        ps.header.frame_id  = target_link
        ps.pose.position    = geometry_msgs.msg.Point(t_base[0], t_base[1], 0)
        ps.pose.orientation = geometry_msgs.msg.Quaternion(*q)
        self.move_pub.publish(ps)

        #Uncomment to actually move
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = target_link
        goal.target_pose.pose.position = geometry_msgs.msg.Point(t_base[0], t_base[1], 0)
        goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*q)
        self.base_client.send_goal(goal)
        print 'Sent GOAL'
        if wait:
            self.base_client.wait_for_result()
        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def laser_point_handler(self, point_stamped):
        p = np.matrix([point_stamped.point.x, point_stamped.point.y, point_stamped.point.z, 1.]).T
        p2d = self.camera_model.left.P * p
        p2d = p2d / p2d[2,0]
        bx = ((self.camera_model.left.w/2.) * .9)
        by = ((self.camera_model.left.h/2.) * .9)
        xlim = [bx, self.camera_model.left.w - bx]
        ylim = [by, self.camera_model.left.h - by]

        if (self.state == self.STATES['driving']):
            return
        self.message = self.transform_point(point_stamped)

        if not in_bounds(p2d, xlim, ylim):
            if self.state != self.STATES['head_turn']:
                self.lock.acquire()
                self.state = self.STATES['head_turn']
                self.lock.release()
        #else if we are in bounds, we do nothing 
        #always update laser's location

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            r.sleep()
            if self.state == self.STATES['head_turn']:
                self.lock.acquire()
                result = self.look_at(self.message, False)
                self.state = self.STATES['ready']
                self.lock.release()

    def look_at(self, message, wait=True):
        g = PointHeadGoal()
        g.target.header.frame_id = message[2]
        g.target.point = geometry_msgs.msg.Point(*message[0])
        g.min_duration = rospy.Duration(1.0)
        g.max_velocity = 10.

        self.head_client.send_goal(g)
        #rospy.loginfo('Sent look at goal ' + str(g))
        if wait:
            self.head_client.wait_for_result()

        if self.head_client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

if __name__ == '__main__':
    lab = LookAtBehavior('wide_stereo')
    lab.run()



    
