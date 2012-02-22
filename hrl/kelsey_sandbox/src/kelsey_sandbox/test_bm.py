#! /usr/bin/python

import roslib
roslib.load_manifest("kelsey_sandbox")
import rospy

from behavior_manager import BehaviorManagerClient

class BMCWrapper(object):
    def __init__(self, name, priority):
        self.name = name
        self.priority = priority
        self.bmc = BehaviorManagerClient('l_arm', self.preempt_cb, self.resume_cb, priority=priority, name=name)

    def request(self):
        print "Request", self.name
        result = self.bmc.request()
        print "Request result:", self.name, result
        return result

    def relinquish(self):
        print "Relinquish", self.name
        self.bmc.relinquish()

    def preempt_cb(self):
        print "preempt", self.name, self.priority

    def resume_cb(self):
        print "resume", self.name, self.priority

    def create_timer(self, start_time, sleep_time):
        self.sleep_time = sleep_time
        def thread(te):
            if self.request():
                rospy.sleep(self.sleep_time)
                self.relinquish()
        rospy.Timer(rospy.Duration(start_time), thread, oneshot=True)

def main():
    rospy.init_node("test_bm")
    p_low = BehaviorManagerClient.PRIORITIES.LOW
    p_med = BehaviorManagerClient.PRIORITIES.MEDIUM
    p_high = BehaviorManagerClient.PRIORITIES.HIGH

    bmcw_low = BMCWrapper("test_low", p_low)
    bmcw_med = BMCWrapper("test_med", p_med)
    bmcw_high = BMCWrapper("test_high", p_high)

    if False:
        bmcw_low.clear_manager()
        bmcw_low.create_timer(0.01, 15)
        bmcw_med.create_timer(5, 5)

        bmcw_low.create_timer(25, 25)
        bmcw_med.create_timer(30, 10)
        bmcw_high.create_timer(35, 10)
        rospy.spin()

    if True:
        bmcw_med.create_timer(0.01, 20)
        bmcw_low.create_timer(5, 10)
        bmcw_high.create_timer(10, 5)
        rospy.spin()

if __name__ == "__main__":
    main()
