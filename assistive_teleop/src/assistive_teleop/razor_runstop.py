#!/usr/bin/env python

import roslib; roslib.load_manifest('assistive_teleop')
import rospy
from std_msgs.msg import Bool
from pr2_msgs.msg import PowerBoardState

class RazorRunStop(object):
    def __init__(self):
        rospy.Subscriber('/power_board/state', PowerBoardState, self.powbd_cb)
        rospy.Subscriber('/tool_state', Bool, self.tool_state_cb)
        self.tool_pub = rospy.Publisher('/tool_toggle', Bool)

    def powbd_cb(self, pbs_msg):
        """Check for run-stop activation"""
        if pbs_msg.run_stop or pbs_msg.wireless_stop:
            self.stop_razor()

    def tool_state_cb(self, tool_msg):
        """Get current tool state"""
        self.tool_state = tool_msg.data

    def stop_razor(self):
        """Toggle Tool State to False if True"""
        #TODO: TOOLSTATE VARIABLE HAS NO CONNECTION TO REAL ON/OFF STATE OF TOOL
        # THIS MAKES IT IMPOSSIBLE TO SET THE TOOL TO OFF, WE CAN ONLY TOGGLE
       # THIS MAKES THE CURRENT NODE USELESS, NEED TO FIX @ Arduino level first.
        if self.tool_state:
            self.tool_pub.publish(Bool(!self.tool_state))
           
if __name__=='__main__':
    rrs=RazorRunStop()
    while not rospy.is_shutdown():
        rospy.spin()
