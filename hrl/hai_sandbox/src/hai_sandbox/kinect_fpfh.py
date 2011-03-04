import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import feature_extractor_fpfh.msg as fmsg
import pdb


def callback(image, fpfh_hist):
  print "got messages!"
  #print image.header.frame_id, fpfh_hist.header.frame_id
  print image.header.stamp.to_sec(), fpfh_hist.header.stamp.to_sec()

def fpfh_cb(fpfh):
  #print fpfh.header.frame_id
  print '>>', fpfh.header.stamp.to_sec()
  #pdb.set_trace()

def image_cb(image):
  #print "image", image.header.frame_id
  print image.header.stamp.to_sec()

rospy.init_node('kinect_features')
image_sub = message_filters.Subscriber('/camera/rgb/image_color', Image)
fpfh_hist_sub = message_filters.Subscriber('fpfh_hist', fmsg.FPFHHist)
depth_sub = message_filters.Subscriber('/camera/depth/points2', PointCloud2)

#ts = message_filters.TimeSynchronizer([image_sub, fpfh_hist_sub], 10)
ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
ts.registerCallback(callback)
#rospy.Subscriber('fpfh_hist', fmsg.FPFHHist, fpfh_cb)
#rospy.Subscriber('/camera/rgb/image_color', Image, image_cb)
print 'reading and spinning!'
rospy.spin()

