#!/usr/bin/python
import roslib; roslib.load_manifest('trf_learn')
import rcommander.rcommander as rc
import rospy
import tf
import pypr2.pr2_utils as pu


rospy.init_node('rcommander_trf', anonymous=True)

tf    = tf.TransformListener()
pr2 = pu.PR2(tf)
rc.run_rcommander(['default', 'default_frame', 'trf', 'pr2'], pr2, tf, width=800, height=600)
