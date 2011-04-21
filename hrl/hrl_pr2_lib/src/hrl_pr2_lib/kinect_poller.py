#! /usr/bin/python
import roslib; roslib.load_manifest("hrl_pr2_lib")
import rospy

from sensor_msgs.msg import PointCloud2

class Poller:
    def __init__(self):
        self.ind = 0
    def callback(self,msg):
        self.ind += 1

node_name = 'kinect_poller'
rospy.init_node(node_name)
pc_topic = rospy.get_param(node_name+'/topic_name')
poller = Poller()
rospy.sleep(8.0)
sub = rospy.Subscriber(pc_topic, PointCloud2, poller.callback)
print "Subcribed to %s" % pc_topic
rospy.spin()
