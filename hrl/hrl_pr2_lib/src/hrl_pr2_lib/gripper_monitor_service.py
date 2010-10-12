#!/usr/bin/python
import roslib; roslib.load_manifest("pr2_laser_pointer_grasp")
import rospy
import copy

from std_msgs.msg import Bool, Float64
from pr2_msgs.msg import PressureState
from pr2_laser_pointer_grasp.srv import GripperMonitor
from numpy import sqrt

def loginfo(str):
    rospy.loginfo("PressureMonitor: " + str)

class PressureMonitor():

    def __init__(self, grip_top, bias_thresh):
        self.bias_pub = rospy.Publisher("/pressure/" + grip_top + "/bias_dist", Float64)

        self.NUM_SENSORS = 22
        self.SENSOR_WEIGHTS = [1.0] * self.NUM_SENSORS 
        if bias_thresh == 0.0:
            self.BIAS_DIST_THRESH = 900.0
        else:
            self.BIAS_DIST_THRESH = bias_thresh
        self.BIAS_TIME = 0.3 
        self.setup()

    def setup(self):
        self.l_bias_sum = [0.0] * self.NUM_SENSORS
        self.r_bias_sum = [0.0] * self.NUM_SENSORS
        self.l_bias = [0.0] * self.NUM_SENSORS
        self.r_bias = [0.0] * self.NUM_SENSORS
        self.bias_done = False
        self.st_bias_time = None 
        self.num_samples = 0
        self.pressure_trigger = False

    def bias_dist(self, x, y):
        def subsq(a, b): return (a - b) ** 2
        return sqrt(sum(map(subsq,x,y)))

    def pressure_biaser(self, msg):
        if self.st_bias_time is None:
            self.st_bias_time = rospy.Time.now().to_sec()
            self.l_bias_sum = copy.copy(msg.l_finger_tip)
            self.r_bias_sum = copy.copy(msg.r_finger_tip)
            self.num_samples = 1
        elapsed = rospy.Time.now().to_sec() - self.st_bias_time
        if elapsed > self.BIAS_TIME:
            def div(x): return x/self.num_samples
            self.l_bias = map(div, self.l_bias_sum)
            self.r_bias = map(div, self.r_bias_sum)
            self.bias_done = True
        def add(x,y): return x+y
        self.l_bias_sum = map(add, self.l_bias_sum, msg.l_finger_tip)
        self.r_bias_sum = map(add, self.r_bias_sum, msg.r_finger_tip)
        self.num_samples += 1

    def pressure_monitor(self, msg):
        total_bias_dist = (self.bias_dist(msg.l_finger_tip,self.l_bias) + 
                                self.bias_dist(msg.r_finger_tip,self.r_bias))
        self.bias_pub.publish(total_bias_dist)
#       import pdb; pdb.set_trace()
        if total_bias_dist > self.BIAS_DIST_THRESH:
            self.pressure_trigger = True

def monitor(msg):
    pm = PressureMonitor(msg.gripper_topic, msg.bias_thresh)
    pbsub = rospy.Subscriber('/pressure/' + msg.gripper_topic, PressureState, pm.pressure_biaser)
    loginfo("Subscribing to " + msg.gripper_topic + ", biasing starting")
    while not pm.bias_done:
        rospy.sleep(0.4)
    pbsub.unregister()
    loginfo("Biasing complete, monitoring...")
    pmsub = rospy.Subscriber('/pressure/' + msg.gripper_topic, PressureState, pm.pressure_monitor)
    while not pm.pressure_trigger:
        rospy.sleep(0.1)
    loginfo("Pressure difference detected!")
    pmsub.unregister()
    pm.setup()
    return True   

if __name__ == "__main__":
    rospy.init_node('gripper_monitor_service')
    svc = rospy.Service('gripper_monitor', GripperMonitor, monitor)
    loginfo("Offering gripper_monitor service.")
    rospy.spin()
