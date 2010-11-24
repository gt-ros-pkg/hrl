
#
# Temoprarily in this package. Advait needs to move it to a better
# location.
#

import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_pr2_lib')
import rospy


## Class defining the core EPC function and a few simple examples.
# More complex behaviors that use EPC should have their own ROS
# packages.
class EPC():
    def __init__(self, robot):
        self.robot = robot

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
                # self.robot.step() this should be within the rapid_call_func for the meka arms.
                t1 = rospy.get_time()

            stop, ea = equi_pt_generator(*arg_list)

            if stop == 'reset timing':
                stop = ''
                t_end = rospy.get_time()

        return stop, ea

    ## Pull back along a straight line (-ve x direction)
    # @param arm - 'right_arm' or 'left_arm'
    # @param ea - starting equilibrium angle.
    # @param rot_mat - rotation matrix defining end effector pose
    # @param distance - how far back to pull.
    def pull_back(self, arm, ea, rot_mat, distance):
        cep = self.robot.FK(arm, ea)
        self.dist_left = distance
        self.ea = ea

        def eq_gen_pull_back(robot, arm, rot_mat):
            if self.dist_left <= 0.:
                return 'done', None
            step_size = 0.01
            cep[0,0] -= step_size
            self.dist_left -= step_size
            ea = robot.IK(arm, cep, rot_mat, self.ea)
            self.ea = ea
            if ea == None:
                return 'IK fail', ea
            return '', [ea,]
        
        arg_list = [self.robot, arm, rot_mat]
        stop, ea = self.epc_motion(eq_gen_pull_back, 0.1, arm, arg_list,
                                   control_function = self.robot.set_jointangles)
        print stop, ea

    ## Pull back along a straight line (-ve x direction)
    # @param arm - 'right_arm' or 'left_arm'
    # @param ea - starting cep.
    # @param rot_mat - rotation matrix defining end effector pose
    # @param distance - how far back to pull.
    def pull_back_cartesian_control(self, arm, cep, rot_mat, distance):
        cep = cep
        self.dist_left = distance

        def eq_gen_pull_back(robot, arm, rot_mat):
            if self.dist_left <= 0.:
                return 'done', None
            step_size = 0.01
            cep[0,0] -= step_size
            self.dist_left -= step_size
            if cep[0,0] < 0.4:
                return 'very close to the body: %.3f'%cep[0,0], None

            return '', (cep, rot_mat)
        
        arg_list = [self.robot, arm, rot_mat]
        stop, ea = self.epc_motion(eq_gen_pull_back, 0.1, arm, arg_list,
                                   control_function = self.robot.set_cartesian)
        print stop, ea

    def move_till_hit(self, arm, vec=np.matrix([0.3,0.,0.]).T, force_threshold=3.0,
                      speed=0.1, bias_FT=True):
        unit_vec =  vec/np.linalg.norm(vec)
        time_step = 0.1
        step_size = speed * time_step
        cep_start = self.robot.get_cep_jtt(arm)
        cep = copy.copy(cep_start)
        def eq_gen(cep):
            force = self.robot.get_wrist_force(arm, base_frame = True)
            force_projection = force.T*unit_vec *-1 # projection in direction opposite to motion.
            if force_projection>force_threshold:
                return 'done', None
            elif np.linalg.norm(force)>45.:
                return 'large force', None
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
                               self.robot.set_cep_jtt)


if __name__ == '__main__':
    import pr2_arms as pa
    rospy.init_node('epc_pr2', anonymous = True)
    rospy.logout('epc_pr2: ready')

    pr2_arms = PR2Arms()
    epc = EPC(pr2)

    r_arm, l_arm = 0, 1
    arm = r_arm

    p1 = np.matrix([0.6, -0.22, -0.05]).T
    epc.robot.go_cep_jtt(arm, p1)
    epc.move_till_hit(arm)

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




