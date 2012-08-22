#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
#import roslib; roslib.load_manifest("rcommander_ar_tour")
import pypr2.pr2_utils as pu
import rcommander_ar_tour.rcommander_ar_server2 as rcs
import sys
import tf 
import rospy

rospy.init_node('rcommander_pr2_server_trf')
action_database_name = sys.argv[1]
ar_tag_database_name = sys.argv[2]
path_to_rcommander_files = sys.argv[3]

tf = tf.TransformListener()
pr2 = pu.PR2(tf)
rcs.run(pr2, tf, action_database_name, ar_tag_database_name, path_to_rcommander_files)
