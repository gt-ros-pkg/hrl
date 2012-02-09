#!/usr/env python

import roslib; roslib.load_manifest('wouse')
import rospy
from pr2_power_board.srv import PowerBoardCommand2, PowerBoardCommand2Request
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Bool, String

CIRCUITS=[0,1,2] #Base, Right arm, Left Arm circuits

class RunStopUtil(object):
    """Provide utility functions for starting/stopping PR2."""
    def __init__(self):
        """Establish service connections for motors, power board."""
        self.init_successful = True
        rospy.loginfo("Waiting for halt motors service")
        try:
            rospy.wait_for_service('pr2_etherCAT/halt_motors', 5) 
            self.halt_motors_client=rospy.ServiceProxy('pr2_etherCAT/halt_motors',Empty)
        except:
            rospy.logerr("Cannot find halt motors service")
            self.init_successful = False

        rospy.loginfo("Waiting for reset motors service")
        try:
            rospy.wait_for_service('pr2_etherCAT/reset_motors',5)
            self.reset_motors_client=rospy.ServiceProxy('pr2_etherCAT/reset_motors',Empty)
        except:
            rospy.logerr("Cannot find halt motors service")
            self.init_successful = False

        rospy.loginfo("Waiting for power_board/control service")
        try:
            rospy.wait_for_service('power_board/control2',5)
            self.power_board_client=rospy.ServiceProxy('power_board/control2',PowerBoardCommand2)
        except:
            rospy.logerr("Cannot find power_board/control2 service")
            self.init_successful = False

    def run_stop(self):
        """Halt motors, place power board into standboy. Stops robot."""
        self.halt_motors_client(EmptyRequest()) #Halt motors immediately
        success = [False, False, False]
        for circuit in CIRCUITS:
            success[circuit] = self.standby_power(circuit)
        if success[0] and success[1] and success[2]:
            return True
        else:
            return False

    def run_start(self):
        """Reset power board, reset motors.  Un-does 'run_stop'."""
        success = [False, False, False]
        for circuit in CIRCUITS:
            success[circuit] = self.reset_power(circuit)
        if success[0] and success[1] and success[2]:
            rospy.sleep(2.0)
            self.reset_motors_client(EmptyRequest())
            return True
        else:
            return False

    def standby_power(self, circuit):
        """Place PR2 power board into standby"""
        stdby_cmd = PowerBoardCommand2Request()
        stdby_cmd.circuit = circuit
        stdby_cmd.command = "stop"
        return self.power_board_client(stdby_cmd)

    def reset_power(self,circuit):
        """Reset PR2 power board to active from standby"""
        reset_cmd = PowerBoardCommand2Request()
        reset_cmd.circuit = circuit
        reset_cmd.command = "start"
        return self.power_board_client(reset_cmd)
