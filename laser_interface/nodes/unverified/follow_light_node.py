# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## @author Hai Nguyen/hai@gatech.edu
from pkg import *
from std_msgs.msg import Position
from std_msgs.msg import BaseVel
from std_msgs.msg import RobotBase2DOdom
import sys
import numpy as np
import nodes as n
import camera as cam
import math
from pyrob.voice import say
from StringIO import StringIO

class FollowBehavior:
    def __init__(self, velocity_topic):
        self.not_inited     = True
        self.velocity_topic = velocity_topic
        self.last_said      = ''

    def init_pose(self, robot_pose):
        R = cam.Rx(math.radians(90)) * cam.Ry(math.radians(-90))
        T = np.matrix([-.095, 0,.162]).T
        self.base_T_camera = cam.homo_transform3d(R, T)
        self.robot_pose     = n.ConstNode(robot_pose)
        self.local_pos      = n.nav.V_KeepLocal_P2d_V(self.robot_pose, n.ConstNode(np.matrix([0.0, 0.0]).T))
        self.attr	        = n.nav.V_LinearAttract_V(self.local_pos, dead_zone = 0.20, far_dist = 1.0)
        self.cmd            = n.nav.R_Conv_V(self.attr, allow_backwards_driving = False)
        self.should_stop    = self.attr.is_done()

        self.cmd.max_vel    = .5
        self.has_stopped    = False
        self.count          = 0
        self.attr.verbosity = -1
        print 'FollowBehavior: ready!'

    def cursor_moved(self, point):
        #Transform into base's coordinate frame
        point3d   = np.matrix([point.x, point.y, point.z, 1.0]).T
        print 'FollowBehavior.cursor_moved: got point', point3d.T
        new_point = self.base_T_camera * point3d
        sio = StringIO()
        print >>sio, int(round(np.linalg.norm(point3d[0:3]) * 100.0))
        new_saying = sio.getvalue()
        if new_saying != self.last_said:
            say(new_saying)
            self.last_said = new_saying

        #Drop the z, store as homogeneous coordinates
        goal2d = new_point[0:2, 0]
        print 'FollowBehavior.cursor_moved: 2d goal', goal2d.T
        self.local_pos.remember(n.ConstNode(goal2d))
        self.has_stopped = False

    def robot_moved(self, update):
        if self.not_inited:
            self.init_pose(n.Pose2D(update.pos.x, update.pos.y, update.pos.th))
            self.not_inited = False
        else:
            self.robot_pose.const = n.Pose2D(update.pos.x, update.pos.y, update.pos.th)

    def run(self):
        if self.not_inited:
            return
        #base_command = self.cmd.val(self.count)
        #print 'publishing', base_command.forward_vel, base_command.rot_vel
        if self.should_stop.val(self.count) and not self.has_stopped:
            self.velocity_topic.publish(BaseVel(0,0))
            #print 'stoppping'
            self.has_stopped = True 
            say('done')
        elif not self.has_stopped:
            #print 'HAS STOPPED'
            base_command = self.cmd.val(self.count)
            msg = BaseVel(base_command.forward_vel, base_command.rot_vel)
            #print 'publishing', base_command.forward_vel, base_command.rot_vel
            self.velocity_topic.publish(msg)
        self.count = self.count + 1

class FakeTopic:
    def publish(self, something):
        pass

class FakePoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


if __name__ == '__main__':
    import time
    pub = rospy.TopicPub('cmd_vel', BaseVel)
    follow_behavior = FollowBehavior(pub)
    rospy.TopicSub('odom', RobotBase2DOdom, follow_behavior.robot_moved)
    rospy.TopicSub(CURSOR_TOPIC, Position, follow_behavior.cursor_moved)
    rospy.ready(sys.argv[0])
    #follow_behavior.cursor_moved(FakePoint(0.0,0.0,1.0))
    while not rospy.isShutdown():
        follow_behavior.run()
        time.sleep(0.016)

#follow_behavior = FollowBehavior(FakeTopic())
















































#def transform2D(o_p, o_angle):
#    """ 
#        Takes as input o_P_* and angle, both descriptions of the new coordinate
#        frame in the original frame (frame o).  Returns a transfrom *_T_o that 
#        describes points in the new coordinate frame (frame *).  
#        The transformation applies a translation then a rotation.
#    """
#    t = numpy.matrix(numpy.eye(3))
#    t[0:2][:,2] = -1 * o_p
#    return rot_mat(o_angle) * t   
#
#def rot_mat(a):
#    """ 
#        Makes a homogeneous rotation matrix that rotates the coordinate system 
#        counter cw
#        i.e. rotates points clockwise
#    """
#    return numpy.matrix([[math.cos(a),    math.sin(a), 0],
#                         [-1*math.sin(a), math.cos(a), 0],
#                         [0,              0,           1]])
#
#class FollowLight:
#    def __init__(self, velocity_topic):
#        self.base_T_camera  = np.matrix(np.zeros((3,4)))
#        self.dead_zone      = 0.02
#        self.far_dist       = 1.0   #Distance beyond which we don't care
#		self.min_attr       = 0.3   #Set min attraction to be 30% of maximum speed
#
#        self.velocity_topic = velocity_topic
#        self.goal_b         = [0.0, 0.0]
#        self.A_T_O          = transform2d(np.matrix([0.0, 0.0]).T, 0.0)
#
#    def cursor_moved(self, point):
#        #Transform into base's coordinate frame
#        point3d   = np.matrix([point.x, point.y, point.z, 1.0]).T
#        new_point = self.base_T_camera * point3d
#
#        #Drop the z, store as homogeneous coordinates
#        self.goal_b = new_point[0:2, 0]
#
#    def robot_moved(self, update):
#        #Frames
#        # A   previous position of robot in global frame
#        # B   current position of robot in global frame
#        # O   global frame
#        B_T_O = transform2d(np.matrix([update.pos.x, update.pos.y]).T, 
#                update.pos.th)
#        
#        #transform goal with this update
#        goal_a        = self.goal_b
#        B_T_A         = B_T_O * np.linalg.inv(self.A_T_O) 
#        self.goal_b   = B_T_A * goal_a
#        self.A_T_O    = B_T_O
#
#    def run(self):
#        #given goal position, calculate velocity vector
#        distance = np.linalg.norm(goal)
#        if distance > self.dead_zone:
#            mag = ((1 - self.min_attr) / (self.far_dist - self.dead_zone)) * norm_dist + self.min_attr
#			mag = min(max(mag, self.min_attr), 1.0)
#			out = mag * (goal / dist)
#            vx = 
#        else:
#        vw = 
#
#        #send to robot
#        self.velocity_topic.publish(BaseVel(vx,vw))


























