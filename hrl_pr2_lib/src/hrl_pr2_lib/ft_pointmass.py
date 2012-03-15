#!/usr/bin/python

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
import math
from geometry_msgs.msg import WrenchStamped, PoseStamped, Point, PointStamped
from tf import TransformListener, transformations
from visualization_msgs.msg import Marker

class Pointmass_Adjust:
   
    pub_marker = True #Set to (True) or to not(False) publish rviz marker showing force vector

    mass = 1.0463 #kg 1.0213
    pos_x = 0.0853 #m, in 'l_wrist_roll_link'
    pos_y = 0.0
    pos_z = 0.0
    x_force_offset = 5.62 #5.70 -- These values determined from experiment, used values are adjusted for better qualitative results using rviz
    y_force_offset = -13.56 #14.10
    z_force_offset = 4.30 #-3.88
    x_torque_offset = -0.4025 #0.3899
    y_torque_offset = -0.4175 #0.3804
    z_torque_offset = -0.21875
    adjustment = WrenchStamped()

    def __init__(self):
        rospy.init_node("ft_pointmass_adjustment")
        rospy.Subscriber("netft_data", WrenchStamped, self.adjust)
        self.ft_out = rospy.Publisher('ft_data_pm_adjusted', WrenchStamped)
        self.force_vec_out = rospy.Publisher('ft_force_vector_marker', Marker)
        self.tfl = TransformListener()

    def adjust(self, ft_in):
        ft_out = WrenchStamped()
        ft_out.header.stamp = rospy.Time.now()
        ft_out.header.frame_id = ft_in.header.frame_id
        ft_out.wrench.force.x = ft_in.wrench.force.x  - self.x_force_offset + self.adjustment.wrench.force.x
        ft_out.wrench.force.y = ft_in.wrench.force.y  - self.y_force_offset + self.adjustment.wrench.force.y
        ft_out.wrench.force.z = ft_in.wrench.force.z  - self.z_force_offset + self.adjustment.wrench.force.z
        ft_out.wrench.torque.x = ft_in.wrench.torque.x - self.x_torque_offset  - self.adjustment.wrench.torque.x
        ft_out.wrench.torque.y = ft_in.wrench.torque.y  - self.y_torque_offset - self.adjustment.wrench.torque.y
        ft_out.wrench.torque.z = ft_in.wrench.torque.z  - self.z_torque_offset - self.adjustment.wrench.torque.z

        self.ft_out.publish(ft_out)
        if self.pub_marker:
            marker = self.form_marker(ft_out)
            self.force_vec_out.publish(marker) 

    def form_marker(self, ws):
        origin = Point()
        force_point = Point()
        force_point.x = 0.1*ws.wrench.force.x
        force_point.y = 0.1*ws.wrench.force.y
        force_point.z = 0.1*ws.wrench.force.z
        force_vec = Marker()
        force_vec.header.stamp = rospy.Time.now()
        force_vec.header.frame_id = '/l_netft_frame'
        force_vec.ns = "ft_sensor"
        force_vec.action = 0
        force_vec.type = 0
        force_vec.scale.x = 0.1
        force_vec.scale.y = 0.2
        force_vec.scale.z = 1
        force_vec.color.a = 0.75
        force_vec.color.r = 0.0
        force_vec.color.g = 1.0
        force_vec.color.b = 0.1

        force_vec.lifetime = rospy.Duration(1)
        force_vec.points.append(origin)
        force_vec.points.append(force_point)
        return force_vec

    def calc_adjustment(self):
        try:
            (pos, quat) = self.tfl.lookupTransform('/base_link', '/l_netft_frame', rospy.Time(0))
        except:
            return
        rot = transformations.euler_from_quaternion(quat) 
        self.adjustment.wrench.torque.x = self.mass*9.80665*self.pos_x*math.sin(rot[0])
        self.adjustment.wrench.torque.y = self.mass*9.80665*self.pos_x*math.sin(rot[1])
        self.adjustment.wrench.torque.z = 0
      
        grav = PointStamped() # Generate a 'vector' of the force due to gravity at the ft sensor 
        grav.header.stamp = rospy.Time(0) #Used to tell tf to grab latest available transform in transformVector3
        grav.header.frame_id = '/base_link'
        grav.point.x = pos[0]
        grav.point.y = pos[1]
        grav.point.z = pos[2] - 9.80665*self.mass 

        netft_grav = self.tfl.transformPoint('/l_netft_frame', grav) #Returns components of the 'gravity force' in each axis of the 'l_netft_frame'
        self.adjustment.wrench.force.x = netft_grav.point.x
        self.adjustment.wrench.force.y = netft_grav.point.y
        self.adjustment.wrench.force.z = netft_grav.point.z

        self.adjustment.header.stamp=rospy.Time.now()
        #self.form_marker(self.adjustment)

if __name__ == "__main__":
    PMC = Pointmass_Adjust()
    r=rospy.Rate(1000)
    while not rospy.is_shutdown():
        PMC.calc_adjustment()
        r.sleep()
