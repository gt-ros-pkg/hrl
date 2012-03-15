import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import sensor_msgs.msg as sm
import hrl_lib.rutils as ru
import numpy as np
import pr2_msgs.msg as pm 
import geometry_msgs.msg as gm
import tf
import hrl_lib.tf_utils as tfu
import tf.transformations as tr
import time
import hrl_lib.util as ut
import pdb

def np_to_pointcloud(points_mat, frame):
    pc = sm.PointCloud()
    pc.header.stamp = rospy.get_rostime()
    pc.header.frame_id = frame
    #pdb.set_trace()
    for i in range(points_mat.shape[1]):
        p32 = gm.Point32()
        p32.x = points_mat[0,i]
        p32.y = points_mat[1,i]
        p32.z = points_mat[2,i]
        pc.points.append(p32)
    return pc

if __name__ == '__main__':
    #load pickle
    import sys
    #import pdb
    pname = sys.argv[1]

    #which frame are these contact points in? (base_link)
    scene, contact_points = ut.load_pickle(pname)
    #pdb.set_trace()
    #t, tip_locs
    #    [len4 list, len4 list... ]

    #plot 3D cloud & contact location! = > using? rviz?
    rospy.init_node('test10')
    contact_pub = rospy.Publisher('contact_cloud', sm.PointCloud)
    touchll_pub = rospy.Publisher('touch_ll', sm.PointCloud)
    touchlr_pub = rospy.Publisher('touch_lr', sm.PointCloud)

    left_contact, right_contact = zip(*[(np.matrix(l[1][2]).T, np.matrix(l[1][3]).T) for l in contact_points])
    left_contact = np.column_stack(left_contact)
    right_contact = np.column_stack(right_contact)

    scene_pc = np_to_pointcloud(scene, 'base_footprint')
    left_con_pc = np_to_pointcloud(left_contact, 'base_footprint')
    right_con_pc = np_to_pointcloud(right_contact, 'base_footprint')

    r = rospy.Rate(10)
    rospy.loginfo('test10: publishing')
    while not rospy.is_shutdown():
        contact_pub.publish(scene_pc)
        touchll_pub.publish(left_con_pc)
        touchlr_pub.publish(right_con_pc)
        r.sleep()
        







































