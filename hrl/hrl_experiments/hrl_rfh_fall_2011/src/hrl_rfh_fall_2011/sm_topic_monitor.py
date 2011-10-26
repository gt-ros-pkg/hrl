import roslib
roslib.load_manifest('smach_ros')

import rospy
import smach

class TopicMonitor(smach.State):
    def __init__(self, topic, msg_type):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'],
                                   output_keys=['output'])
        self.cur_msg = None
        rospy.Subscriber(topic, msg_type, self.topic_cb)

    def topic_cb(self, msg):
        self.cur_msg = msg

    def execute(self, userdata):
        self.cur_msg = None
        while not rospy.is_shutdown():
            if self.cur_msg is not None:
                userdata.output = self.cur_msg
                return 'succeeded'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.01)
        return 'aborted'

    def request_preempt(self):
        smach.State.request_preempt(self)
