#! /usr/bin/python

import roslib; roslib.load_manifest('kelsey_sandbox')
import rospy
import roslib.message
import rosbag
import sys
import rosservice
from pr2_msgs.msg import AccelerometerState
import scipy.io

class AccelSaver:
    def __init__(self):
        self.data = []
        self.first = True

    def proc_acc(self, msg):
        if len(msg.samples) > 0:
            if self.first:
                self.beg_time = msg.header.stamp.to_sec()
                self.first = False
            sum_x, sum_y, sum_z = 0, 0, 0
            for sample in msg.samples:
                sum_x += sample.x
                sum_y += sample.y
                sum_z += sample.z
            self.data.append((msg.header.stamp.to_sec() - self.beg_time, sum_x / len(msg.samples), sum_y / len(msg.samples), sum_z / len(msg.samples)))
def main():
    rospy.init_node('accel_save')
    asave = AccelSaver()
    rospy.Subscriber('/accelerometer/l_gripper_motor', AccelerometerState, asave.proc_acc)
    print "Starting:"
    rospy.spin()
    mat_data = {}
    mat_data['accel'] = asave.data
    scipy.io.savemat('accel_data.mat', mat_data)

if __name__ == "__main__":
    main()

