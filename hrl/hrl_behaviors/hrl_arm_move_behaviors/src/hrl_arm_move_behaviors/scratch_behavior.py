#! /usr/bin/python
import random
import sys
import numpy as np

import roslib; roslib.load_manifest("hrl_arm_move_behaviors")
import rospy
import actionlib
import tf.transformations as tf_trans
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from hrl_arm_move_behaviors.srv import ScratchBehavior, ScratchBehaviorResponse

class ScratchBehaviorSrv(object):
    def __init__(self):
        args = sys.argv
        self.scratch_srv = rospy.Service('arm_move_behaviors/scratch_behavior', ScratchBehavior, self.execute_scratch_behavior)

    def execute_scratch_behavior(self, req):
        print "execute_scratch_behavior"
        print req
        return ScratchBehaviorResponse()

def main():
    rospy.init_node('scratch_behavior')
    sb = ScratchBehaviorSrv()
    rospy.spin()

if __name__ == "__main__":
    main()
