
import numpy as np, math
import copy
import roslib; roslib.load_manifest('equilibrium_point_control')
import rospy
from std_msgs.msg import Bool

##
# Abstract class to be implemented when using equilibrium point control.
# If generate_ep produces equilibrium points which are used by control_ep
# to move the arm, this object can be passed into EPC.epc_motion to control
# the arm.  Equilibrium points can be of any type so long as generate_ep,
# control_ep, and clamp_ep are all written with this type in mind.
class EPGenerator(object):
    #----------------- abstract functions ---------------------
    ##
    # Generates a new equilibrium point.
    # @return (stop, ep)
    #         stop: EPStopConditions.CONTINUE or non-empty string to terminate
    #         ep: equilibrium point to be sent to control_function
    def generate_ep(self):
        raise RuntimeError('Unimplemented Function')

    ##
    # Commands the arm to move towards the specified equilibrium point.
    # @param ep equilibrium point to command the arm towards
    def control_ep(self, ep):
        raise RuntimeError('Unimplemented Function')

    ##
    # Takes an equilibrium point and clamps it to reasonable control values.
    # To be overwritten if needed by the child class.
    # @param ep equilibrium point to clamp
    # @return clamped equilibrium point
    def clamp_ep(self, ep):
        return ep

    ##
    # Termination check for collision or goal reaching.
    # To be overwritten if needed by the child class.
    # @return EPStopConditions.CONTINUE or non-empty string to terminate
    def terminate_check(self):
        return EPStopConditions.CONTINUE
    #----------------------------------------------------------

##
# Enumerated constants for EPC termination conditions
class EPStopConditions:
    CONTINUE = ''
    ROSPY_SHUTDOWN = 'rospy shutdown'
    ROS_STOP = 'stop_command_over_ROS'
    TIMEOUT = 'timed out'
    RESET_TIMING = 'reset timing'
    SUCCESSFUL = 'epc motion successful'
    COLLISION = 'collision detected'

##
# Simple class containing the core EPC function: a control loop paradigm.
class EPC(object):

    ##
    # Initializes variables and subscribers
    def __init__(self, epc_name = 'epc'):
        self.epc_name = epc_name
        self.stop_epc = False
        self.pause_epc = False
        rospy.Subscriber('/'+epc_name+'/stop', Bool, self._stop_cb)
        rospy.Subscriber('/'+epc_name+'/pause', Bool, self._pause_cb)

    ##
    # Subscriber callback for stopping the arm's motion
    def _stop_cb(self, msg):
        self.stop_epc = msg.data
        self.pause_epc = False # stop/start overrides pause.

    ##
    # Subscriber callback for pausing the arm's motion
    def _pause_cb(self, msg):
        self.pause_epc = msg.data

    ##
    # Control loop for equilibrium point control.  For each time step, ep_gen
    # provides 4 functions, a termination check, an ep generation step, 
    # a clamping step, and an ep control step.  These functions are called in this
    # order along with helper functionality for stopping and pausing control.
    # @param ep_gen - Object of type EPGenerator.
    # @param time_step: Time between successive calls to equi_pt_generator
    # @param timeout - time after which the epc motion will stop.
    # @return stop (the string which has the reason why the epc
    # motion stopped.), ea (last commanded equilibrium angles)
    def epc_motion(self, ep_gen, time_step, timeout=np.inf):
        rospy.loginfo("[epc] epc_motion started")
        timeout_at = rospy.get_time() + timeout
        stop = EPStopConditions.CONTINUE
        ea = None
        while True:
            # basic rospy shutdown termination
            if rospy.is_shutdown():
                stop = EPStopConditions.ROSPY_SHUTDOWN
                break

            # check to see if we should stop (stop_epc changed from another thread)
            if self.stop_epc:
                stop = EPStopConditions.ROS_STOP
                break

            # check to see if the generator thinks we should stop
            stop = ep_gen.terminate_check()
            if stop != EPStopConditions.CONTINUE:
                break
            
            # check to see if we're paused
            if self.pause_epc:
                rospy.sleep(0.1)
                timeout_at += 0.101 # approximate.
                continue

            # timeout check
            if timeout_at < rospy.get_time():
                stop = EPStopConditions.TIMEOUT
                break

            # create a new ep
            stop, ep = ep_gen.generate_ep() 
            if stop != EPStopConditions.CONTINUE:
                break

            # if a post-processing function exits, use it to process the ep
            ep = ep_gen.clamp_ep(ep)

            # command the arm to move to the ep
            ep_gen.control_ep(ep)

            rospy.sleep(time_step)

        rospy.loginfo("[epc] epc_motion stopped with termination condition: %s" % stop)
        return stop, ea


