
import roslib; roslib.load_manifest('pr2_doors_epc')
import rospy
import hrl_lib.transforms as tr
import force_torque.FTClient as ftc
import math, numpy as np

from rviz_marker_test import *

if __name__ == '__main__':

    print 'Hello World'
    ft_client = ftc.FTClient('force_torque_ft2')
    ft_client.bias()
    marker_pub = rospy.Publisher('/test_marker', Marker)

    p_st = np.matrix([0.,0.,0.]).T
    while not rospy.is_shutdown():
        ft = ft_client.read(fresh = True)
        rmat =  tr.Rx(math.radians(180.)) * tr.Ry(math.radians(-90.)) * tr.Rz(math.radians(30.))
        force = rmat * ft[0:3,:]
        print 'Force:', force.A1
        # force is now in the 'robot' coordinate frame.
        
        force_scale = 0.1
        p_end = p_st + force * force_scale
        marker = get_marker_arrow(p_st, p_end, 'base_link')
        marker_pub.publish(marker)
        rospy.sleep(0.1)



