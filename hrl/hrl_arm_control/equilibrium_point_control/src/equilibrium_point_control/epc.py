
import numpy as np, math
import copy
import roslib; roslib.load_manifest('equilibrium_point_control')
import rospy
from std_msgs.msg import Bool


class EP_Generator():
    # @param ep_gen_func: function that returns stop, ea  where ea is the param to the control_function and  stop: string which is '' for epc motion to continue
    def __init__(self, ep_gen_func, control_function,
                 ep_clamp_func=None):
        self.ep_gen_func = ep_gen_func
        self.control_function = control_function
        self.ep_clamp_func = ep_clamp_func


## Class defining the core EPC function and a few simple examples.
# More complex behaviors that use EPC should have their own ROS
# packages.
class EPC():
    def __init__(self, robot, epc_name = 'epc'):
        self.robot = robot
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
                stop, ea = ep_gen_func(ep_gen)
            if stop == 'reset timing':
                stop = ''
                t_end = rospy.get_time()

            if stop == '':
                if ep_clamp_func != None:
                    ep = ea[0]
                    ea = list(ea)
                    ea[0] = ep_clamp_func(ep)
                    ea = tuple(ea)

                control_function(*ea)

            rt.sleep()

        return stop, ea

    ##
    # this function only makes sense for arms where we have Joint
    # space Equilibrium Point Control.
    def go_jep(self, goal_jep, speed=math.radians(20)):
        start_jep = self.robot.get_ep()
        diff_jep = np.array(goal_jep) - np.array(start_jep)
        time_step = 0.02
        max_ch = np.max(np.abs(diff_jep))
        total_time = max_ch / speed
        n_steps = max(np.round(total_time / time_step + 0.5), 1)
        jep_step = diff_jep / n_steps

        def eq_gen_func(ep_gen):
            jep = ep_gen.jep
            step_num = ep_gen.step_num
            if step_num < n_steps:
                q = list(np.array(jep) + jep_step)
                stop = ''
            else:
                q = None
                stop = 'Reached'
            step_num += 1
            ep_gen.jep = q
            ep_gen.step_num = step_num
            return stop, (q, time_step*1.2)
        
        ep_gen = EP_Generator(eq_gen_func, self.robot.set_ep)
        ep_gen.step_num = 0
        ep_gen.jep = copy.copy(start_jep)
        return self.epc_motion(ep_gen, time_step)


