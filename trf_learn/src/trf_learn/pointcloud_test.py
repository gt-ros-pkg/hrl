import roslib; roslib.load_manifest('trf_learn')
import rospy
import pointclouds as pc
import sensor_msgs.msg as sm
import pdb

kinect_name = '/head_mount_kinect' 
pc_topic = kinect_name + '/depth/points'

rospy.init_node('pctest')
pc_message = rospy.wait_for_message(pc_topic, sm.PointCloud2)
pdb.set_trace()
pc_array = pc.pointcloud2_to_array(pc_message, padding=1)
pc_xyz = pc.get_xyz_points(pc_array)

print 'done!'

