
import numpy as np, math
import roslib; roslib.load_manifest('epc_core')
import rospy
from std_msgs.msg import Bool


## Class defining the core EPC function and a few simple examples.
# More complex behaviors that use EPC should have their own ROS
# packages.
class EPC():
    def __init__(self, epc_name = 'epc'):
        self.stop_epc = False
        self.pause_epc = False
        rospy.Subscriber('/'+epc_name+'/stop', Bool, self.stop_cb)
        rospy.Subscriber('/'+epc_name+'/pause', Bool, self.pause_cb)

    def stop_cb(self, msg):
        self.stop_epc = msg.data
        self.pause_epc = False # stop/start overrides pause.

    def pause_cb(self, msg):
        self.pause_epc = msg.data

    ##
    # @param equi_pt_generator: function that returns stop, ea  where ea: equilibrium angles and  stop: string which is '' for epc motion to continue
    # @param time_step: time between successive calls to equi_pt_generator
    # @param timeout - time after which the epc motion will stop.
    # @return stop (the string which has the reason why the epc
    # motion stopped.), ea (last commanded equilibrium angles)
    def epc_motion(self, ep_gen_func, ep_gen_state, time_step,
                   control_function=None, ep_clamp_func=None,
                   timeout=np.inf):
        rt = rospy.Rate(1/time_step)
        timeout_at = rospy.get_time() + timeout
        stop = ''
        ea = None
        while stop == '':
            if rospy.is_shutdown():
                stop = 'rospy shutdown'
                continue

            if self.stop_epc:
                stop = 'stop_command_over_ROS'
                continue
            
            if self.pause_epc:
                rospy.sleep(0.1)
                timeout_at += 0.101 # approximate.
                continue

            if timeout_at < rospy.get_time():
                stop = 'timed out'
            if stop == '':
                stop, ea = equi_pt_generator(ep_gen_state)
            if stop == 'reset timing':
                stop = ''
                t_end = rospy.get_time()

            if stop == '':
                if clamp_func != None:
                    ep = ea[0]
                    ea = list(ea)
                    ea[0] = ep_clamp_func(ep)
                    ea = tuple(ea)

                control_function(*ea)

            rt.sleep()

        return stop, ea





