#!/usr/bin/python

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
import math
from geometry_msgs.msg import WrenchStamped, PoseStamped, Point, PointStamped
from tf import TransformListener, transformations
from visualization_msgs.msg import Marker

class Pointmass_Adjust:
    
    mass = 1.0213 #kg
    pos_x = 0.0853 #m, in 'l_wrist_roll_link'
    pos_y = 0.0
    pos_z = 0.0
    x_force_offset = 3.85 #3.88 -- These values determined from experiment, used values are adjusted for better qualitative results using rviz
    y_force_offset = 13.96 #14.10
    z_force_offset = -5.72 #-5.70
    x_torque_offset = -0.21875
    y_torque_offset = 0.3804
    z_torque_offset = 0.3899
    adjustment = WrenchStamped()

    def __init__(self):
        rospy.init_node("ft_pointmass_adjustment")
        rospy.Subscriber("netft_data", WrenchStamped, self.transform)
        self.ft_out = rospy.Publisher('ft_data_pm_adjusted', WrenchStamped)
        self.l_wrist_ft_out = rospy.Publisher('ft_data_raw_framed', WrenchStamped)
        self.force_vec_out = rospy.Publisher('ft_force_vector_marker', Marker)
        self.tfl = TransformListener()

    def transform(self, ft_in):
        ft_in_wrist = WrenchStamped()
        ft_in_wrist.header.stamp = ft_in.header.stamp
        ft_in_wrist.header.frame_id = '/l_netft_frame'
        ft_in_wrist.wrench.force.x = ft_in.wrench.force.z
        ft_in_wrist.wrench.torque.x = ft_in.wrench.torque.z
        ft_in_wrist.wrench.force.y = -ft_in.wrench.force.y
        ft_in_wrist.wrench.torque.y = -ft_in.wrench.torque.y
        ft_in_wrist.wrench.force.z = -ft_in.wrench.force.x
        ft_in_wrist.wrench.torque.z = -ft_in.wrench.torque.x
        self.l_wrist_ft_out.publish(ft_in_wrist)
        #self.calc_adjustment()
        self.adjust(ft_in_wrist)

    def adjust(self, ft_in):
        ft_out = WrenchStamped()
        ft_out.header.stamp = rospy.Time.now()
        ft_out.header.frame_id = ft_in.header.frame_id #'l_wrist_roll_link' # OR PROPER FRAME FOR FT SENSOR
        ft_out.wrench.force.x = ft_in.wrench.force.x - self.adjustment.wrench.force.x - self.x_force_offset
        ft_out.wrench.force.y = ft_in.wrench.force.y - self.adjustment.wrench.force.y - self.y_force_offset
        ft_out.wrench.force.z = ft_in.wrench.force.z - self.adjustment.wrench.force.z - self.z_force_offset
        ft_out.wrench.torque.x = ft_in.wrench.torque.x - self.x_torque_offset 
        ft_out.wrench.torque.y = ft_in.wrench.torque.y - self.adjustment.wrench.torque.y - self.y_torque_offset
        ft_out.wrench.torque.z = ft_in.wrench.torque.z - self.adjustment.wrench.torque.z - self.z_torque_offset

        self.ft_out.publish(ft_out)

        origin = Point()
        force_point = Point()
        force_point.x = 0.1*ft_out.wrench.force.x
        force_point.y = 0.1*ft_out.wrench.force.y
        force_point.z = 0.1*ft_out.wrench.force.z
        force_vec = Marker()
        force_vec.header.stamp = rospy.Time.now()
        force_vec.header.frame_id = '/l_netft_frame'
        force_vec.ns = "ft_sensor"
        force_vec.id = 1
        force_vec.action = 0
        force_vec.type = 0
        force_vec.scale.x = 0.1
        force_vec.scale.y = 0.2
        force_vec.scale.z = 1
        force_vec.color.a = 1.0
        force_vec.color.r = 0.0
        force_vec.color.g = 1.0
        force_vec.color.b = 0.1

        force_vec.lifetime = rospy.Duration(1)
        force_vec.points.append(origin)
        force_vec.points.append(force_point)
        self.force_vec_out.publish(force_vec) 

    def calc_adjustment(self):
        try:
            (pos, quat) = self.tfl.lookupTransform('/base_link', '/l_netft_frame', rospy.Time(0))
        except:
            return
        rot = transformations.euler_from_quaternion(quat) 
        self.adjustment.wrench.torque.y = self.mass*self.pos_x*math.cos(rot[1])
        self.adjustment.wrench.torque.z = self.mass*self.pos_x*math.sin(rot[0])
      
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

if __name__ == "__main__":
    PMC = Pointmass_Adjust()
    r=rospy.Rate(1000)
    while not rospy.is_shutdown():
        PMC.calc_adjustment()
        r.sleep()
