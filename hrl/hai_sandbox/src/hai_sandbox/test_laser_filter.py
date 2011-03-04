import roslib; roslib.load_manifest('hai_sandbox')

import rospy
from sensor_msgs.msg import PointCloud

def callback(data):
    print len(data.points)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tilt_scan_shadow_filtered", PointCloud, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

