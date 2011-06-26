
import numpy as np, math
import copy
import roslib; roslib.load_manifest('epc_core')
import rospy
from std_msgs.msg import Bool


class EP_Generator():
    # @param ep_gen_func: function that returns stop, ea  where ea is the param to the control_function and  stop: string which is EPC.StopConditions.CONTINUE for epc motion to continue
    def __init__(self, ep_gen_func, control_function,
                 ep_clamp_func=None):
        self.ep_gen_func = ep_gen_func
        self.control_function = control_function
        self.ep_clamp_func = ep_clamp_func


## Class defining the core EPC function and a few simple examples.
# More complex behaviors that use EPC should have their own ROS
# packages.
class EPC():

    ##
    # Initializes variables and subscribers
    def __init__(self, epc_name = 'epc'):
        self.stop_epc = False
        self.pause_epc = False
        rospy.Subscriber('/'+epc_name+'/stop', Bool, self._stop_cb)
        rospy.Subscriber('/'+epc_name+'/pause', Bool, self._pause_cb)

    ##
    # Enumerated constants for EPC termination conditions
    class StopConditions:
        CONTINUE = ''
        ROSPY_SHUTDOWN = 'rospy shutdown'
        ROS_SHUTDOWN = 'stop_command_over_ROS'
        TIMEOUT = 'timed out'
        RESET_TIMING = 'reset timing'
        COMPLETED = 'epc motion completed'

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
    # @param ep_gen - object of EP_Generator. can include any state that you want.
    # @param time_step: time between successive calls to equi_pt_generator
    # @param timeout - time after which the epc motion will stop.
    # @return stop (the string which has the reason why the epc
    # motion stopped.), ea (last commanded equilibrium angles)
    def epc_motion(self, ep_gen, time_step, timeout=np.inf):
        ep_gen_func = ep_gen.ep_gen_func
        control_function = ep_gen.control_function
        ep_clamp_func = ep_gen.ep_clamp_func

        rt = rospy.Rate(1/time_step)
        timeout_at = rospy.get_time() + timeout
        stop = EPC.StopConditions.CONTINUE
        ea = None
        while True:
            # basic rospy shutdown termination
            if rospy.is_shutdown():
                stop = EPC.StopConditions.ROSPY_SHUTDOWN
                break

            # check to see if we should stop (stop_epc changed from another thread)
            if self.stop_epc:
                stop = EPC.StopConditions.ROS_SHUTDOWN
                break
            
            # check to see if we're paused
            if self.pause_epc:
                rospy.sleep(0.1)
                timeout_at += 0.101 # approximate.
                continue

            # timeout check
            if timeout_at < rospy.get_time():
                stop = EPC.StopConditions.TIMEOUT
                break

            # create a new ep
            stop, ea = ep_gen_func(ep_gen) 

            # check to see if the generator function wants to stop
            if stop != EPC.StopConditions.CONTINUE:
                break

            # if a post-processing function exits, use it to process the ep
            if ep_clamp_func != None:
                ep = ea[0]
                ea = list(ea)
                ea[0] = ep_clamp_func(ep)
                ea = tuple(ea)

            # command the arm to move to the ep
            control_function(*ea)

            rt.sleep()

        return stop, ea


