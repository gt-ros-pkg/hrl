#!/usr/bin/python

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy
from geometry_msgs.msg import WrenchStamped
from tf import TransformListener, TransformBroadcaster

class Frame_Transformer:
    
    mass = 1 #kg
    pos_x = 0.1 #m, in 'l_wrist_roll_link'
    pos_y = 0.0
    pos_z = 0.0
    adjustment = WrenchStamped()


    def __init__(self):
        rospy.init_node("FT_Pointmass_adjustment")
        rospy.Subscriber("netft_data", WrenchStamped, self.transform)
        self.l_wrist_ft_out = rospy.Publisher('l_ft_frame_data', WrenchStamped)
        self.tfl = TransformListener()
        self.tfb = TransformBroadcaster()

    def transform(self, ft_in):

        ft_in_wrist = WrenchStamped()
        ft_in_wrist.header.stamp = ft_in.header.stamp
        ft_in_wrist.header.frame_id = 'l_netft_frame'
        ft_in_wrist.wrench.force.x = ft_in.wrench.force.z
        ft_in_wrist.wrench.torque.x = ft_in.wrench.torque.z
        ft_in_wrist.wrench.force.y = -ft_in.wrench.force.y
        ft_in_wrist.wrench.torque.y = -ft_in.wrench.torque.y
        ft_in_wrist.wrench.force.z = -ft_in.wrench.force.x
        ft_in_wrist.wrench.torque.z = -ft_in.wrench.torque.x
        self.l_wrist_ft_out.publish(ft_in_wrist)

if __name__ == "__main__":
    FT = Frame_Transformer()
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        FT.tfb.sendTransform((0.0522,0,0),(0.13459031 ,0. ,0. ,0.99090133), rospy.Time.now(), 'l_netft_frame', 'l_wrist_roll_link')
        #Approximately 15.5 degree rotation relative to wrist_roll_link, to account for the netft being mounted slightly rotated
        r.sleep()
