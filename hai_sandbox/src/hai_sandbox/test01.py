import roslib; roslib.load_manifest('hai_sandbox')
import rospy

import pr2_msgs.msg as pm
import time
import hrl_lib.util as hru

class DataAccumulate:
    def __init__(self, topic, type):
        rospy.Subscriber(topic, type, self.callback)
        self.data = []
        self.headers = []
        self.t = None

    def callback(self, msg):
        msg.header.stamp = msg.header.stamp.to_time()
        self.data.append(msg)
        self.t = time.time()

    def done(self):
        if self.t != None:
            return (time.time() - self.t) > 2.
        else:
            return False

if __name__ == '__main__':
    import sys
    rospy.init_node('test01')
    d = DataAccumulate('/pressure/l_gripper_motor', pm.PressureState)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if d.done():
            break
    print 'saved to', sys.argv[1]
    hru.save_pickle(d.data, sys.argv[1])














































    #print len(d.data)
    #hru.save_pickle(d.data[0]['stamp'], sys.argv[1])
    #hru.save_pickle(d.data[0]['frame_id'], sys.argv[1])
    #hru.save_pickle(d.data[0]['l_finger_tip'], sys.argv[1])
    #hru.save_pickle(d.data[0]['r_finger_tip'], sys.argv[1])
    #hru.save_pickle(d.headers, 'headers.pkl')

        #self.data.app
        #self.data.append({"stamp": msg.header.stamp.to_time(),
        #                  "frame_id": msg.header.frame_id,
        #                  "l_finger_tip": msg.l_finger_tip,
        #                  "r_finger_tip": msg.r_finger_tip})
        #self.headers.append(msg.header)
