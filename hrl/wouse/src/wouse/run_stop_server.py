#!/usr/bin/env python

import roslib; roslib.load_manifest('wouse')
import rospy
from std_msgs.msg import Header, Bool, String
from pr2_power_board.srv import PowerBoardCommand2, PowerBoardCommand2Request
from std_srvs.srv import Empty, EmptyRequest

from wouse.srv import WouseRunStop

CIRCUITS=[0,1,2] #Base, Right arm, Left Arm circuits
DEAD_MAN_CONFIGURATION=False


class RunStop(object):
    """Provide utility functions for starting/stopping PR2."""
    def __init__(self):
        """Establish service connections for motors, power board."""
        self.init_successful = True
        try:
            rospy.wait_for_service('pr2_etherCAT/halt_motors', 5) 
            self.halt_motors_client=rospy.ServiceProxy('pr2_etherCAT/halt_motors',Empty)
            rospy.loginfo("Found halt motors service")
        except:
            rospy.logerr("Cannot find halt motors service")
            self.init_successful = False

        try:
            rospy.wait_for_service('pr2_etherCAT/reset_motors',5)
            self.reset_motors_client=rospy.ServiceProxy('pr2_etherCAT/reset_motors',Empty)
            rospy.loginfo("Found reset motors service")
        except:
            rospy.logerr("Cannot find halt motors service")
            self.init_successful = False

        try:
            rospy.wait_for_service('power_board/control2',5)
            self.power_board_client=rospy.ServiceProxy('power_board/control2',PowerBoardCommand2)
            rospy.loginfo("Found power_board/control2 service")
        except:
            rospy.logerr("Cannot find power_board/control2 service")
            self.init_successful = False

    def stop(self):
        """Halt motors, place power board into standboy. Stops robot."""
        self.halt_motors_client(EmptyRequest()) #Halt motors immediately
        success = [False, False, False]
        for circuit in CIRCUITS:
            success[circuit] = self.standby_power(circuit)
        if success[0] and success[1] and success[2]:
            return True
        else:
            return False

    def start(self):
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

class RunStopServer(object):
    def __init__(self):
        """Provide dead-man-switch like server for handling wouse run-stops."""
        rospy.Service("wouse_run_stop", WouseRunStop, self.service_cb)
        self.run_stop = RunStop()
        if DEAD_MAN_CONFIGURATION:
            rospy.Subscriber('runstop_alive_ping', Header, self.check_in)
            self.timeout = rospy.Duration(rospy.get_param('wouse_timeout', 0.1))
            self.last_active_time = None
            rospy.Timer(rospy.Duration(5), self.check_receiving, oneshot=True)

    def check_receiving(self, event):
        """After timeout, check to ensure that activity is seen from wouse."""
        if isinstance(self.last_active_time, type(Header().stamp)):
            if self.last_active_time - rospy.Time.now() < rospy.Duration(5):
                rospy.loginfo("RunStopServer receiving Dead-man switch pings")
            else:
                rospy.logwarn("RunStopServer has contact time, but it is old")
        else:
            rospy.logwarn("RunStopServer has not received a message from Wouse")

    def check_in(self, hdr):
        """Update most recent active time, or do run-stop if past timeout."""
        if self.last_active_time is None:
            self.last_active_time = hdr.stamp
            return
        if hdr.stamp - self.last_active_time > self.timeout:
            self.run_stop.stop()
            rospy.logwarn("No Signal Received from Wouse for %s sec."
                                                            %self.timeout)
        self.last_active_time = hdr.stamp

    def service_cb(self, req):
        """Handle service requests to start/stop run-stop.  Used to reset."""
        if req.stop:
            return self.run_stop.stop()
        elif req.start:
            return self.run_stop.start()

if __name__=='__main__':
    rospy.init_node('run_stop_server')
    rss = RunStopServer()
    while not rospy.is_shutdown():
        rospy.spin()
