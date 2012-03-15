import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.util as ut
import sensor_msgs.msg as sm
import sys

class CamInfoCB:
    def __init__(self, topic):
        rospy.init_node('grab_cam_info')
        rospy.Subscriber(topic, sm.CameraInfo, self.cb)
        self.msg = None

    def cb(self, msg):
        self.msg = msg

topic = sys.argv[1]
save_name = sys.argv[2]
c = CamInfoCB(topic)
r = rospy.Rate(10)
while not rospy.is_shutdown() and c.msg == None:
    r.sleep()
ut.save_pickle(c.msg, save_name)


