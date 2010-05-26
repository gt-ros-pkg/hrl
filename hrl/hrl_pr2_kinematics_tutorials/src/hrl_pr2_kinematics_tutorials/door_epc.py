
import roslib; roslib.load_manifest('hrl_pr2_kinematics')
import rospy

from equilibrium_point_control.msg import MechanismKinematicsRot
from equilibrium_point_control.msg import MechanismKinematicsJac

import epc
from threading import RLock


##
# compute the end effector rotation matrix.
# @param hook - hook angle. RADIANS(0, -90, 90) (hor, up, down)
# @param angle - angle between robot and surface normal.
# Angle about the Z axis through which the robot must turn to face
# the surface.
def rot_mat_from_angles(hook, surface):
    rot_mat = tr.Rz(hook) * tr.Rx(surface)
    return rot_mat



class Door_EPC(epc.EPC):
    def __init__(self, robot):
        epc.EPC.__init__(self, robot)

        self.mech_kinematics_lock = RLock()
        self.fit_circle_lock = RLock()

        rospy.Subscriber('mechanism_kinematics_rot',
                         MechanismKinematicsRot,
                         self.mechanism_kinematics_rot_cb)
        rospy.Subscriber('mechanism_kinematics_jac',
                         MechanismKinematicsJac,
                         self.mechanism_kinematics_jac_cb)
        self.mech_traj_pub = rospy.Publisher('mechanism_trajectory', Point32)
        self.eq_pt_not_moving_counter = 0

    ## log the joint angles, equi pt joint angles and forces.
    def log_state(self, arm):
        t_now = rospy.get_time()
        q_now = self.robot.get_joint_angles(arm)

        self.pull_trajectory.q_list.append(q_now)
        self.pull_trajectory.time_list.append(t_now)

        self.eq_pt_trajectory.q_list.append(self.q_guess) # see equi_pt_generator - q_guess is the config for the eq point.
        self.eq_pt_trajectory.time_list.append(t_now)
    
        wrist_force = self.robot.get_wrist_force(arm, base_frame=True)
        self.force_trajectory.f_list.append(wrist_force.A1.tolist())
        self.force_trajectory.time_list.append(t_now)
        return '' # log_state also used as a rapid_call_func

    ##
    # @param arm - 'right_arm' or 'left_arm'
    # @param motion vec is in tl frame.
    # @param step_size - distance (meters) through which CEP should move
    # @param rot_mat - rotation matrix for IK
    # @return JEP
    def update_eq_point(self, arm, motion_vec, step_size, rot_mat):
        self.eq_pt_cartesian = self.eq_pt_cartesian_ts
        next_pt = self.eq_pt_cartesian + step_size * motion_vec
        q_eq = self.robot.IK(arm, next_pt, rot_mat, self.q_guess)
        self.eq_pt_cartesian = next_pt
        self.eq_pt_cartesian_ts = self.eq_pt_cartesian
        self.q_guess = q_eq
        return q_eq

    def common_stopping_conditions(self):
        stop = ''
        if self.q_guess == None:
            stop = 'IK fail'

        wrist_force = self.robot.get_wrist_force('right_arm',base_frame=True)
        mag = np.linalg.norm(wrist_force)
        print 'force magnitude:', mag
        if mag > self.eq_force_threshold:
            stop = 'force exceed'

        if mag < 1.2 and self.hooked_location_moved:
            if (self.prev_force_mag - mag) > 30.:
                stop = 'slip: force step decrease and below thresold.'
                #stop = ''
            else:
                self.slip_count += 1
        else:
            self.slip_count = 0

        if self.slip_count == 10:
            stop = 'slip: force below threshold for too long.'
        return stop

    ## constantly update the estimate of the kinematics and move the
    # equilibrium point along the tangent of the estimated arc, and
    # try to keep the radial force constant.
    # @param h_force_possible - True (hook side) or False (hook up).
    # @param v_force_possible - False (hook side) or True (hook up).
    # Is maintaining a radial force possible or not (based on hook
    # geometry and orientation)
    # @param cep_vel - tangential velocity of the cep in m/s
    def eqpt_gen_control_radial_force(self, arm, rot_mat,
                           h_force_possible, v_force_possible, cep_vel):
        self.log_state(arm)
        step_size = 0.1 * cep_vel # 0.1 is the time interval between calls to the equi_generator function (see pull)
        q_eq = self.update_eq_point(arm, self.eq_motion_vec, step_size,
                                    rot_mat)
        stop = self.common_stopping_conditions()

        wrist_force = self.robot.get_wrist_force(arm, base_frame=True)
        mag = np.linalg.norm(wrist_force)

        curr_pos = self.robot.FK(arm,self.pull_trajectory.q_list[-1])
        if len(self.pull_trajectory.q_list)>1:
            start_pos = np.matrix(self.cartesian_pts_list[0]).T
        else:
            start_pos = curr_pos

        #mechanism kinematics.
        self.mech_traj_pub.publish(Point32(curr_pos[0,0],
                                        curr_pos[1,0], curr_pos[2,0]))

        if self.use_jacobian:
            self.mech_kinematics_lock.acquire()

            self.cartesian_pts_list.append(curr_pos.A1.tolist())
            tangential_vec_ts = self.tangential_vec_ts
            force_vec_ts = self.constraint_vec_1_ts + self.constraint_vec_2_ts

            # this is specifically for the right arm, and lots of
            # other assumptions about the hook etc. (Advait, Feb 28, 2010)
            if h_force_possible:
                force_vec_ts[2,0] = 0.
            if v_force_possible:
                force_vec_ts[1,0] = 0.
            force_vec_ts = force_vec_ts / np.linalg.norm(force_vec_ts)
            if force_vec_ts[2,0] < 0.: # only allowing upward force.
                force_vec_ts = -force_vec_ts
            if force_vec_ts[1,0] < 0.: # only allowing force to the left
                force_vec_ts = -force_vec_ts

            self.mech_kinematics_lock.release()

            force_vec = force_vec_ts
            tangential_vec = tangential_vec_ts

        if self.use_rotation_center:
            self.fit_circle_lock.acquire()
            self.cartesian_pts_list.append(curr_pos.A1.tolist())
            rad = self.rad
            cx_start, cy_start = self.cx_start, self.cy_start
            cz_start = self.cz_start
            self.fit_circle_lock.release()
            cx, cy = cx_start, cy_start
            cz = cz_start

            radial_vec = curr_pos_tl-np.matrix([cx,cy,cz]).T
            radial_vec = radial_vec/np.linalg.norm(radial_vec)
            if cy_start<start_pos[1,0]:
                tan_x,tan_y = -radial_vec[1,0],radial_vec[0,0]
            else:
                tan_x,tan_y = radial_vec[1,0],-radial_vec[0,0]
            
            if tan_x > 0. and (start_pos[0,0]-curr_pos[0,0]) < 0.09:
                tan_x = -tan_x
                tan_y = -tan_y

            if cy_start > start_pos[1,0]:
                radial_vec = -radial_vec # axis to the left, want force in
                                       # anti-radial direction.
            rv = radial_vec
            force_vec = np.matrix([rv[0,0], rv[1,0], 0.]).T
            tangential_vec = np.matrix([tan_x, tan_y, 0.]).T
            
            tangential_vec_ts = tangential_vec
            radial_vec_ts = radial_vec
            force_vec_ts = force_vec

        f_vec = -1*np.array([wrist_force[0,0], wrist_force[1,0],
                             wrist_force[2,0]])
        f_rad_mag = np.dot(f_vec, force_vec.A1)
        err = f_rad_mag-5.
        if err>0.:
            kp = -0.1
        else:
            kp = -0.2
        radial_motion_mag = kp * err # radial_motion_mag in cm (depends on eq_motion step size)

        if self.use_rotation_center:
            if h_force_possible == False and parallel_to_floor:
                radial_motion_mag = 0.
            if v_force_possible == False and parallel_to_floor == False:
                radial_motion_mag = 0.

        radial_motion_vec =  force_vec * radial_motion_mag
        self.eq_motion_vec = copy.copy(tangential_vec)
        self.eq_motion_vec += radial_motion_vec
        
        self.prev_force_mag = mag

        if self.init_tangent_vector == None:
            self.init_tangent_vector = copy.copy(tangential_vec_ts)
        c = np.dot(tangential_vec_ts.A1, self.init_tangent_vector.A1)
        ang = np.arccos(c)

        dist_moved = np.dot((curr_pos - start_pos).A1, self.tangential_vec_ts.A1)
        ftan = abs(np.dot(wrist_force.A1, tangential_vec.A1))
        if abs(dist_moved) > 0.09 and self.hooked_location_moved == False:
            # change the force threshold once the hook has started pulling.
            self.hooked_location_moved = True
            self.eq_force_threshold = ut.bound(mag+30.,20.,80.)
            self.ftan_threshold = self.ftan_threshold + max(10., 1.5*ftan)
        if self.hooked_location_moved:
            if abs(tangential_vec_ts[2,0]) < 0.2 and ftan > self.ftan_threshold:
                stop = 'ftan threshold exceed: %f'%ftan
        else:
            self.ftan_threshold = max(self.ftan_threshold, ftan)


        if self.hooked_location_moved and ang > math.radians(90.):
            print 'Angle:', math.degrees(ang)
            self.open_ang_exceed_count += 1
            if self.open_ang_exceed_count > 2:
                stop = 'opened mechanism through large angle: %.1f'%(math.degrees(ang))
        else:
            self.open_ang_exceed_count = 0

        self.mech_time_list.append(time.time())
        self.mech_x_list.append(ang)
        self.f_rad_list.append(f_rad_mag)
        self.f_tan_list.append(np.dot(f_vec, tangential_vec.A1))
        self.tan_vec_list.append(tangential_vec_ts.A1.tolist())
        self.rad_vec_list.append(force_vec_ts.A1.tolist())

        return stop, q_eq

    def pull(self, arm, hook_angle, force_threshold, ea,
             kinematics_estimation = 'rotation_center', pull_left = False):

        self.init_tangent_vector = None
        self.open_ang_exceed_count = 0.
        if kinematics_estimation == 'rotation_center':
            self.use_rotation_center = True
        else:
            self.use_rotation_center = False

        if kinematics_estimation == 'jacobian':
            self.use_jacobian = True
        else:
            self.use_jacobian = False

        #rot_mat = tr.Rz(hook_angle)*tr.Ry(math.radians(-90))
        rot_mat = rot_mat_from_angles(hook_angle, surface_angle)

        time_dict = {}
        time_dict['before_pull'] = time.time()

        self.pull_trajectory = at.JointTrajectory()
        self.eq_pt_trajectory = at.JointTrajectory()
        self.force_trajectory = at.ForceTrajectory()

        self.robot.step()
        start_config = self.robot.get_joint_angles(arm)

        self.eq_force_threshold = force_threshold
        self.ftan_threshold = 2.
        self.hooked_location_moved = False # flag to indicate when the hooking location started moving.
        self.prev_force_mag = np.linalg.norm(self.robot.get_wrist_force(arm))
        self.eq_motion_vec = np.matrix([-1.,0.,0.]).T # might want to change this to account for the surface_angle.
        self.slip_count = 0

        self.eq_pt_cartesian = self.robot.FK(arm, ea)
        self.eq_pt_cartesian_ts = self.robot.FK(arm, ea)
        self.start_pos = copy.copy(self.eq_pt_cartesian)
        self.q_guess = ea

        if not pull_left:
            self.tangential_vec_ts = np.matrix([-1.,0.,0.]).T
            self.constraint_vec_2_ts = np.matrix([0.,0.,1.]).T
            self.constraint_vec_1_ts = np.matrix([0.,1.,0.]).T
        else:
            self.tangential_vec_ts = np.matrix([0.,1.,0.]).T
            self.constraint_vec_2_ts = np.matrix([0.,0.,1.]).T
            self.constraint_vec_1_ts = np.matrix([1.,0.,0.]).T

        self.mech_time_list = []
        self.mech_x_list = []
        self.f_rad_list = []
        self.f_tan_list = []
        self.tan_vec_list = []
        self.rad_vec_list = []

        self.cartesian_pts_list = []
        ee_pos = self.robot.end_effector_pos(arm)

        if self.use_rotation_center:
        # this might have to change depending on left and right
        # arm? or maybe not since the right arm can open both
        # doors.
            self.cx_start = ee_pos[0,0]
            self.cy_start = ee_pos[1,0]-1.0
            self.cz_start = ee_pos[2,0]
            self.rad = 5.0

        h_force_possible = abs(hook_angle) < math.radians(30.)
        v_force_possible = abs(hook_angle) > math.radians(60.)
        arg_list = [arm, rot_mat, h_force_possible, v_force_possible, cep_vel]
        result, jep = self.epc_motion(self.eqpt_gen_control_radial_force,
                                            0.1, arm, arg_list, self.log_state)

        print 'EPC motion result:', result
        print 'Original force threshold:', force_threshold
        print 'Adapted force threshold:', self.eq_force_threshold
        print 'Adapted ftan threshold:', self.ftan_threshold

        time_dict['after_pull'] = time.time()

        d = {'actual': self.pull_trajectory, 'eq_pt': self.eq_pt_trajectory,
             'force': self.force_trajectory, 'torque': self.jt_torque_trajectory,
             'info': info_string, 'force_threshold': force_threshold,
             'eq_force_threshold': self.eq_force_threshold, 'hook_angle':hook_angle,
             'result':result, 'time_dict':time_dict,
             'cep_vel': cep_vel,
             'ftan_threshold': self.ftan_threshold}

        self.robot.step()

        ut.save_pickle(d,'pull_trajectories_'+d['info']+'_'+ut.formatted_time()+'.pkl')

        dd = {'mechanism_x': self.mech_x_list,
              'mechanism_time': self.mech_time_list,
              'force_rad_list': self.f_rad_list,
              'force_tan_list': self.f_tan_list,
              'tan_vec_list': self.tan_vec_list,
              'rad_vec_list': self.rad_vec_list
              }
        ut.save_pickle(dd,'mechanism_trajectories_robot_'+d['info']+'_'+ut.formatted_time()+'.pkl')

    ## behavior to search around the hook_loc to try and get a good
    # hooking grasp
    # @param arm - 'right_arm' or 'left_arm'
    # @param hook_angle - radians(0,-90,90) side,up,down
    # @param hook_loc - 3x1 np matrix
    # @param angle - angle between torso x axis and surface normal.
    # @return s, jep (stopping string and last commanded JEP)
    def search_and_hook(self, arm, hook_angle, hook_loc, angle,
                        hooking_force_threshold = 5.):

        rot_mat = rot_mat_from_angles(hook_angle, angle)
        if arm == 'right_arm':
            hook_dir = np.matrix([0.,1.,0.]).T # hook direc in home position
        elif arm == 'left_arm':
            hook_dir = np.matrix([0.,-1.,0.]).T # hook direc in home position
        else:
            raise RuntimeError('Unknown arm: %s', arm)
        start_loc = hook_loc + rot_mat.T * hook_dir * -0.03 # 3cm direc opposite to hook.

        # vector normal to surface and pointing into the surface.
        normal_tl = tr.Rz(-angle) * np.matrix([1.0,0.,0.]).T

        pt1 = start_loc - normal_tl * 0.1
        pt1[2,0] -= 0.02 # funny error in meka control code? or is it gravity comp?
        self.robot.go_cartesian(arm, pt1, rot_mat, speed=0.2)

        vec = normal_tl * 0.2
        s, jep = self.firenze.move_till_hit(arm, vec=vec, force_threshold=2.0,
                                            rot=rot_mat, speed=0.07)

        self.eq_pt_cartesian = self.firenze.FK(arm, jep)
        self.eq_pt_cartesian_ts = self.firenze.FK(arm, jep)
        self.start_pos = copy.copy(self.eq_pt_cartesian)
        self.q_guess = jep
        move_dir = rot_mat.T * hook_dir

        arg_list = [arm, move_dir, rot_mat, hooking_force_threshold]
        s, jep = self.compliant_motion(self.equi_generator_surface_follow, 0.05,
                                       arm, arg_list)
        return s, jep


if __name__ == '__main__':

    print 'Hello World'


