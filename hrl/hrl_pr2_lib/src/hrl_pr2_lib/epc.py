
#
# Temoprarily in this package. Advait needs to move it to a better
# location.
#

import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy

import hrl_lib.util as ut

## Class defining the core EPC function and a few simple examples.
# More complex behaviors that use EPC should have their own ROS
# packages.
class EPC():
    def __init__(self, robot):
        self.robot = robot
        self.f_list = []
        self.ee_list = []
        self.cep_list = []

    ##
    # @param equi_pt_generator: function that returns stop, ea  where ea: equilibrium angles and  stop: string which is '' for epc motion to continue
    # @param rapid_call_func: called in the time between calls to the equi_pt_generator can be used for logging, safety etc.  returns string which is '' for epc motion to continue
    # @param time_step: time between successive calls to equi_pt_generator
    # @param arg_list - list of arguments to be passed to the equi_pt_generator
    # @return stop (the string which has the reason why the epc
    # motion stopped.), ea (last commanded equilibrium angles)
    def epc_motion(self, equi_pt_generator, time_step, arm, arg_list,
                   rapid_call_func=None, control_function=None):

        stop, ea = equi_pt_generator(*arg_list)
        t_end = rospy.get_time()
        while stop == '':
            if rospy.is_shutdown():
                stop = 'rospy shutdown'
                continue
            t_end += time_step
            #self.robot.set_jointangles(arm, ea)
            #import pdb; pdb.set_trace()
            control_function(arm, *ea)

            # self.robot.step() this should be within the rapid_call_func for the meka arms.
            t1 = rospy.get_time()
            while t1<t_end:
                if rapid_call_func != None:
                    stop = rapid_call_func(arm)
                    if stop != '':
                        break
                #self.robot.step() this should be within the rapid_call_func for the meka arms
                rospy.sleep(0.01)
                t1 = rospy.get_time()

            if stop == '':
                stop, ea = equi_pt_generator(*arg_list)
            if stop == 'reset timing':
                stop = ''
                t_end = rospy.get_time()

        return stop, ea


    ## Pull back along a straight line (-ve x direction)
    # @param arm - 'right_arm' or 'left_arm'
    # @param distance - how far back to pull.
    def pull_back_cartesian_control(self, arm, distance, logging_fn):
        cep, _ = self.robot.get_cep_jtt(arm)
        cep_end = cep + distance * np.matrix([-1., 0., 0.]).T
        self.dist_left = distance

        def eq_gen_pull_back(cep):
            logging_fn(arm)
            if self.dist_left <= 0.:
                return 'done', None
            step_size = 0.01
            cep[0,0] -= step_size
            self.dist_left -= step_size
            if cep[0,0] < 0.4:
                return 'very close to the body: %.3f'%cep[0,0], None
            return '', (cep, None)
        
        arg_list = [cep]
        s = self.epc_motion(eq_gen_pull_back, 0.1, arm, arg_list,
                    control_function = self.robot.set_cep_jtt)
        print s

    def move_till_hit(self, arm, vec=np.matrix([0.3,0.,0.]).T, force_threshold=3.0,
                      speed=0.1, bias_FT=True):
        unit_vec =  vec/np.linalg.norm(vec)
        time_step = 0.1
        dist = np.linalg.norm(vec)
        step_size = speed * time_step
        cep_start, _ = self.robot.get_cep_jtt(arm)
        cep = copy.copy(cep_start)
        def eq_gen(cep):
            force = self.robot.get_wrist_force(arm, base_frame = True)
            force_projection = force.T*unit_vec *-1 # projection in direction opposite to motion.
            print 'force_projection:', force_projection
            if force_projection>force_threshold:
                return 'done', None
            elif np.linalg.norm(force)>45.:
                return 'large force', None
            elif np.linalg.norm(cep_start-cep) >= dist:
                return 'reached without contact', None
            else:
                cep_t = cep + unit_vec * step_size
                cep[0,0] = cep_t[0,0]
                cep[1,0] = cep_t[1,0]
                cep[2,0] = cep_t[2,0]
                return '', (cep, None)

        if bias_FT:
            self.robot.bias_wrist_ft(arm)
        rospy.sleep(0.5)
        return self.epc_motion(eq_gen, time_step, arm, [cep],
                               control_function = self.robot.set_cep_jtt)

    def cep_gen_surface_follow(self, arm, move_dir, force_threshold,
                               cep, cep_start):
        wrist_force = self.robot.get_wrist_force(arm, base_frame=True)
        if wrist_force[0,0] < -3.:
            cep[0,0] -= 0.002
        if wrist_force[0,0] > -1.:
            cep[0,0] += 0.003
    
        if cep[0,0] > (cep_start[0,0]+0.05):
            cep[0,0] = cep_start[0,0]+0.05
    
        step_size = 0.002
        cep_t = cep + move_dir * step_size
        cep[0,0] = cep_t[0,0]
        cep[1,0] = cep_t[1,0]
        cep[2,0] = cep_t[2,0]

        v = cep - cep_start
        if (wrist_force.T * move_dir)[0,0] < -force_threshold:
            stop = 'got a hook'
        elif np.linalg.norm(wrist_force) > 50.:
            stop = 'force is large %f'%(np.linalg.norm(wrist_force))
        elif (v.T * move_dir)[0,0] > 0.20:
            stop = 'moved a lot without a hook'
        else:
            stop = ''
        return stop, (cep, None)

if __name__ == '__main__':
    import pr2_arms as pa
    rospy.init_node('epc_pr2', anonymous = True)
    rospy.logout('epc_pr2: ready')

    pr2_arms = pa.PR2Arms()
    epc = EPC(pr2_arms)

    r_arm, l_arm = 0, 1
    arm = r_arm

#    #----- testing move_till_hit ------
#    p1 = np.matrix([0.6, -0.22, -0.05]).T
#    epc.robot.go_cep_jtt(arm, p1)
#    epc.move_till_hit(arm)

    raw_input('Hit ENTER to close')
    pr2_arms.close_gripper(arm)
    raw_input('Hit ENTER to search_and_hook')
    p1 = np.matrix([0.8, -0.22, -0.05]).T
    epc.search_and_hook(arm, p1)
    epc.pull_back_cartesian_control(arm, 0.3)

    d = {
            'f_list': epc.f_list, 'ee_list': epc.ee_list,
            'cep_list': epc.cep_list
        }

    ut.save_pickle(d, 'pr2_pull_'+ut.formatted_time()+'.pkl')


    
#    if False:
#        ea = [0, 0, 0, 0, 0, 0, 0]
#        ea = epc.robot.get_joint_angles(arm)
#        rospy.logout('Going to starting position')
#        epc.robot.set_jointangles(arm, ea, duration=4.0)
#        raw_input('Hit ENTER to pull')
#        epc.pull_back(arm, ea, tr.Rx(0), 0.2)
#
#    if False:
#        p = np.matrix([0.9, -0.3, -0.15]).T
#        rot = tr.Rx(0.)
#        rot = tr.Rx(math.radians(90.))
#
#        rospy.logout('Going to starting position')
#    #    epc.robot.open_gripper(arm)
#        epc.robot.set_cartesian(arm, p, rot)
#    #    raw_input('Hit ENTER to close the gripper')
#    #    epc.robot.close_gripper(arm)
#        raw_input('Hit ENTER to pull')
#        epc.pull_back_cartesian_control(arm, p, rot, 0.4)




