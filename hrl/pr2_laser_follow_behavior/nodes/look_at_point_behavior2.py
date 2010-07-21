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
        self.state = 'turn'
        self.move_state = None
        self.laser_point_base = None
        self.message = None
        self.double_click = None
        self.seq = 0

        rospy.init_node('look_at_point_behavior', anonymous=True)
        rospy.Subscriber('cursor3d', PointStamped, self.laser_point_handler)
        self.double_click = rospy.Subscriber('mouse_left_double_click', String, self.double_click_cb)
        self.camera_model = cam.ROSStereoCalibration('/' + camera_root_topic + '/left/camera_info' , 
                                                     '/' + camera_root_topic + '/right/camera_info')
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
        self.base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_pub = rospy.Publisher('look_at_point_goal', PoseStamped)
        self.tflistener = tf.TransformListener()
        rospy.loginfo( 'Running')

    def double_click_cb(self, a_str):
        rospy.loginfo('Double CLICKED')
        self.double_click = True

    def laser_point_handler(self, point_stamped):
        if self.state == 'turn':
            self.laser_point_base = self.transform_point(point_stamped)
            self.message = point_stamped

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
        q_base = tf.transformations.quaternion_from_euler(0, 0, angle)
        return (t_base, q_base, target_link)
        
    def move_base(self, point, wait=True):
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
        if wait:
            self.base_client.wait_for_result()
        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def look_at(self, message, wait=True):
        g = PointHeadGoal()
        g.target.header.frame_id = message[2]
        g.target.point = geometry_msgs.msg.Point(*message[0])
        g.min_duration = rospy.Duration(1.0)
        g.max_velocity = 10.

        self.head_client.send_goal(g)
        if wait:
            self.head_client.wait_for_result(rospy.Duration(1.))
        if self.head_client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def run(self):
        r = rospy.Rate(100)
        timeout_time = None
        self.double_click = None

        while not rospy.is_shutdown():
            if self.state == 'turn':
                if self.laser_point_base is not None:
                    if self.double_click is None:
                        if self.message.header.seq != self.seq:
                            self.seq = self.message.header.seq
                            point_stamped = self.message
                            p = np.matrix([point_stamped.point.x, point_stamped.point.y, point_stamped.point.z, 1.]).T
                            p2d = self.camera_model.left.P * p
                            p2d = p2d / p2d[2,0]
                            bx = ((self.camera_model.left.w/2.) * .9)
                            by = ((self.camera_model.left.h/2.) * .9)
                            xlim = [bx, self.camera_model.left.w - bx]
                            ylim = [by, self.camera_model.left.h - by]
                            if not in_bounds(p2d, xlim, ylim):
                                rospy.loginfo('\'turn\': Looking at laser point msg #: ' + str(self.message.header.seq))
                                self.look_at(self.laser_point_base, True)
                    else:
                        rospy.loginfo('\'turn\': double clicked. Transitioning to \'move\'.')
                        self.state = 'move'
                        self.move_state = 'send_cmd'
                        self.double_click = None

            elif self.state == 'move':
                if self.move_state == 'send_cmd':
                    if self.laser_point_base is not None:
                        rospy.loginfo('\'move\': Sending move command.')
                        self.move_base(self.laser_point_base, False)
                        self.move_state = 'check_status'
                        self.laser_point_base = None
                        self.message = None
                    else:
                        raise RuntimeError('laser_point_base is none!')

                elif self.move_state == 'check_status':
                    if self.double_click is not None:
                        rospy.loginfo('\'move\': Canceling goal. Transitioning back to \'turn\'.')
                        self.base_client.cancel_goal()
                        self.state = 'turn'
                        self.move_state = None
                        self.double_click = None
                    else:
                        if self.base_client.get_state() == GoalStatus.SUCCEEDED or \
                           self.base_client.simple_state == actionlib.SimpleGoalState.DONE:
                            rospy.loginfo('\'move\': Reached goal. Transitioning to \'turn\'.')
                            self.state = 'turn'
                            self.move_state = None
                            self.double_click = None
                        #only if we exceed our wait oime
                        #else:
                        #    return False???

                else:
                    raise RuntimeError('invalid state for self.move_state')
            else:
                raise RuntimeError('invalid state for self.state')

            r.sleep()

if __name__ == '__main__':
    lab = LookAtBehavior('wide_stereo')
    lab.run()



    
