
import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('hrl_pr2_door_opening')
roslib.load_manifest('equilibrium_point_control')
import rospy

from equilibrium_point_control.msg import MechanismKinematicsRot
from equilibrium_point_control.msg import MechanismKinematicsJac
from equilibrium_point_control.msg import ForceTrajectory
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty

import epc
import hrl_lib.util as ut

class Door_EPC(epc.EPC):
    def __init__(self, robot):
        epc.EPC.__init__(self, robot)

        self.mech_kinematics_lock = RLock()
        self.fit_circle_lock = RLock()

        rospy.Subscriber('mechanism_kinematics_rot',
                         MechanismKinematicsRot,
                         self.mechanism_kinematics_rot_cb)
        rospy.Subscriber('epc/stop', Empty, self.stop_cb)
        # used in the ROS stop_cb and equi_pt_generator_control_radial_force
        self.force_traj_pub = rospy.Publisher('epc/force_test', ForceTrajectory)
        self.mech_traj_pub = rospy.Publisher('mechanism_trajectory', Point32)

    def init_log(self):
        self.f_list = []
        self.f_list_ati = []
        self.f_list_estimate = []
        self.f_list_torques = []
        self.cep_list = []
        self.ee_list = []
        self.ft = ForceTrajectory()
        if self.mechanism_type != '':
            self.ft.type = self.mechanism_type
        else:
            self.ft.type = 'rotary'

    def log_state(self, arm):
        # only logging the right arm.
        f = self.robot.get_wrist_force_ati(arm, base_frame=True)
        self.f_list_ati.append(f.A1.tolist())

        f = self.robot.get_wrist_force_estimate(arm, base_frame=True)
        self.f_list_estimate.append(f.A1.tolist())

        f = self.robot.get_force_from_torques(arm)
        self.f_list_torques.append(f.A1.tolist())

        f = self.robot.get_wrist_force(arm, base_frame=True)
        self.f_list.append(f.A1.tolist())

        cep, _ = self.robot.get_cep_jtt(arm, hook_tip=True)
        self.cep_list.append(cep.A1.tolist())
#        ee, _ = self.robot.get_ee_jtt(arm)
        ee, _ = self.robot.end_effector_pos(arm)
        self.ee_list.append(ee.A1.tolist())
        
        if self.started_pulling_on_handle == False:
            if f[0,0] > 10.:
                self.started_pulling_on_handle_count += 1
            else:
                self.started_pulling_on_handle_count = 0
                self.init_log() # reset logs until started pulling on the handle.
                self.init_tangent_vector = None

            if self.started_pulling_on_handle_count > 1:
                self.started_pulling_on_handle = True

        return ''

    ## ROS callback. Stop and maintain position.
    def stop_cb(self, cmd):
        self.stopping_string = 'stop_cb called.'

    def common_stopping_conditions(self):
        stop = ''
        # right arm only.
        wrist_force = self.robot.get_wrist_force(0, base_frame=True)
        mag = np.linalg.norm(wrist_force)
        if mag > self.eq_force_threshold:
            stop = 'force exceed'

        if mag < 1.2 and self.hooked_location_moved:
            if (self.prev_force_mag - mag) > 30.:
                stop = 'slip: force step decrease and below thresold.'
            else:
                self.slip_count += 1
        else:
            self.slip_count = 0

        if self.slip_count == 10:
            stop = 'slip: force below threshold for too long.'
        return stop

    def mechanism_kinematics_rot_cb(self, mk):
        self.fit_circle_lock.acquire()
        self.cx_start = mk.cx
        self.cy_start = mk.cy
        self.cz_start = mk.cz
        self.rad = mk.rad
        self.fit_circle_lock.release()

    ## constantly update the estimate of the kinematics and move the
    # equilibrium point along the tangent of the estimated arc, and
    # try to keep the radial force constant.
    # @param h_force_possible - True (hook side) or False (hook up).
    # @param v_force_possible - False (hook side) or True (hook up).
    # Is maintaining a radial force possible or not (based on hook
    # geometry and orientation)
    # @param cep_vel - tangential velocity of the cep in m/s
    def cep_gen_control_radial_force(self, arm, cep, cep_vel):
        self.log_state(arm)
        if self.started_pulling_on_handle == False:
            cep_vel = 0.02

        #step_size = 0.01 * cep_vel
        step_size = 0.1 * cep_vel # 0.1 is the time interval between calls to the equi_generator function (see pull)
        stop = self.common_stopping_conditions()
        wrist_force = self.robot.get_wrist_force(arm, base_frame=True)
        mag = np.linalg.norm(wrist_force)

        curr_pos, _ = self.robot.get_ee_jtt(arm)
        if len(self.ee_list)>1:
            start_pos = np.matrix(self.ee_list[0]).T
        else:
            start_pos = curr_pos

        #mechanism kinematics.
        if self.started_pulling_on_handle:
            self.mech_traj_pub.publish(Point32(curr_pos[0,0],
                                       curr_pos[1,0], curr_pos[2,0]))

        self.fit_circle_lock.acquire()
        rad = self.rad
        cx_start, cy_start = self.cx_start, self.cy_start
        cz_start = self.cz_start
        self.fit_circle_lock.release()
        cx, cy = cx_start, cy_start
        cz = cz_start
        print 'cx, cy, r:', cx, cy, rad

        radial_vec = curr_pos - np.matrix([cx,cy,cz]).T
        radial_vec = radial_vec/np.linalg.norm(radial_vec)
        if cy_start < start_pos[1,0]:
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

        if arm == 'right_arm' or arm == 0:
            if force_vec_ts[1,0] < 0.: # only allowing force to the left
                force_vec_ts = -force_vec_ts
        else:
            if force_vec_ts[1,0] > 0.: # only allowing force to the right
                force_vec_ts = -force_vec_ts

        f_vec = -1*np.array([wrist_force[0,0], wrist_force[1,0],
                             wrist_force[2,0]])
        f_rad_mag = np.dot(f_vec, force_vec.A1)
        err = f_rad_mag-4.
        if err>0.:
            kp = -0.1
        else:
            kp = -0.2
        radial_motion_mag = kp * err # radial_motion_mag in cm (depends on eq_motion step size)
        radial_motion_vec =  force_vec * radial_motion_mag
        print 'tangential_vec:', tangential_vec.A1
        eq_motion_vec = copy.copy(tangential_vec)
        eq_motion_vec += radial_motion_vec
        
        self.prev_force_mag = mag

        if self.init_tangent_vector == None or self.started_pulling_on_handle == False:
            self.init_tangent_vector = copy.copy(tangential_vec_ts)
        c = np.dot(tangential_vec_ts.A1, self.init_tangent_vector.A1)
        ang = np.arccos(c)
        if np.isnan(ang):
            ang = 0.

        tangential_vec = tangential_vec / np.linalg.norm(tangential_vec) # paranoia abot vectors not being unit vectors.
        dist_moved = np.dot((curr_pos - start_pos).A1, tangential_vec_ts.A1)
        ftan = abs(np.dot(wrist_force.A1, tangential_vec.A1))
        self.ft.tangential_force.append(ftan)
        self.ft.radial_force.append(f_rad_mag)

        if self.ft.type == 'rotary':
            self.ft.configuration.append(ang)
        else: # drawer
            print 'dist_moved:', dist_moved
            self.ft.configuration.append(dist_moved)

        if self.started_pulling_on_handle:
            self.force_traj_pub.publish(self.ft)

#        if self.started_pulling_on_handle == False:
#            ftan_pull_test = -np.dot(wrist_force.A1, tangential_vec.A1)
#            print 'ftan_pull_test:', ftan_pull_test
#            if ftan_pull_test > 5.:
#                self.started_pulling_on_handle_count += 1
#            else:
#                self.started_pulling_on_handle_count = 0
#                self.init_log() # reset logs until started pulling on the handle.
#                self.init_tangent_vector = None
#
#            if self.started_pulling_on_handle_count > 1:
#                self.started_pulling_on_handle = True

        if abs(dist_moved) > 0.09 and self.hooked_location_moved == False:
            # change the force threshold once the hook has started pulling.
            self.hooked_location_moved = True
            self.eq_force_threshold = ut.bound(mag+30.,20.,80.)
            self.ftan_threshold = 1.2 * self.ftan_threshold + 20.
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

        cep_t = cep + eq_motion_vec * step_size
        cep[0,0] = cep_t[0,0]
        cep[1,0] = cep_t[1,0]
        cep[2,0] = cep_t[2,0]
        
        print 'CEP:', cep.A1

        stop = stop + self.stopping_string
        return stop, (cep, None)

    def pull(self, arm, force_threshold, cep_vel, mechanism_type=''):
        self.mechanism_type = mechanism_type
        self.stopping_string = ''
        self.eq_pt_not_moving_counter = 0

        self.init_log()

        self.init_tangent_vector = None
        self.open_ang_exceed_count = 0.

        self.eq_force_threshold = force_threshold
        self.ftan_threshold = 2.
        self.hooked_location_moved = False # flag to indicate when the hooking location started moving.
        self.prev_force_mag = np.linalg.norm(self.robot.get_wrist_force(arm))
        self.slip_count = 0

        self.started_pulling_on_handle = False
        self.started_pulling_on_handle_count = 0

        ee_pos, _ = self.robot.get_ee_jtt(arm)

        self.cx_start = ee_pos[0,0]
        self.rad = 10.0
        self.cy_start = ee_pos[1,0]-self.rad
        self.cz_start = ee_pos[2,0]

        cep, _ = self.robot.get_cep_jtt(arm)
        arg_list = [arm, cep, cep_vel]
        result, _ = self.epc_motion(self.cep_gen_control_radial_force,
                                    0.1, arm, arg_list, self.log_state,
                                    #0.01, arm, arg_list,
                                    control_function = self.robot.set_cep_jtt)

        print 'EPC motion result:', result
        print 'Original force threshold:', force_threshold
        print 'Adapted force threshold:', self.eq_force_threshold
        print 'Adapted ftan threshold:', self.ftan_threshold

        d = {
                'f_list': self.f_list, 'ee_list': self.ee_list,
                'cep_list': self.cep_list, 'ftan_list': self.ft.tangential_force,
                'config_list': self.ft.configuration, 'frad_list': self.ft.radial_force,
                'f_list_ati': self.f_list_ati,
                'f_list_estimate': self.f_list_estimate,
                'f_list_torques': self.f_list_torques
            }
        ut.save_pickle(d,'pr2_pull_'+ut.formatted_time()+'.pkl')

    def search_and_hook(self, arm, hook_loc, hooking_force_threshold = 5.,
                        hit_threshold=15., hit_motions = 1,
                        hook_direction = 'left'):
        # this needs to be debugged. Hardcoded for now.
        #if arm == 'right_arm' or arm == 0:
        #    hook_dir = np.matrix([0., 1., 0.]).T # hook direc in home position
        #    offset = -0.03
        #elif arm == 'left_arm' or arm == 1:
        #    hook_dir = np.matrix([0., -1., 0.]).T # hook direc in home position
        #    offset = -0.03
        #else:
        #    raise RuntimeError('Unknown arm: %s', arm)
        #start_loc = hook_loc + rot_mat.T * hook_dir * offset

        if hook_direction == 'left':
            offset = np.matrix([0., -0.03, 0.]).T
            move_dir = np.matrix([0., 1., 0.]).T
        elif hook_direction == 'up':
            offset = np.matrix([0., 0., -0.03]).T
            move_dir = np.matrix([0., 0., 1.]).T
        start_loc = hook_loc + offset

        # vector normal to surface and pointing into the surface.
        normal_tl = np.matrix([1.0, 0., 0.]).T

        pt1 = start_loc - normal_tl * 0.1
        self.robot.go_cep_jtt(arm, pt1)

        raw_input('Hit ENTER to go')

        vec = normal_tl * 0.2
        rospy.sleep(1.)
        for i in range(hit_motions):
            s = self.move_till_hit(arm, vec=vec, force_threshold=hit_threshold, speed=0.07)

        cep_start, _ = self.robot.get_cep_jtt(arm)
        cep = copy.copy(cep_start)
        arg_list = [arm, move_dir, hooking_force_threshold, cep, cep_start]
        print 'Hi there.'
        s = self.epc_motion(self.cep_gen_surface_follow, 0.1, arm,
                arg_list, control_function = self.robot.set_cep_jtt)
        print 'result:', s
        return s


if __name__ == '__main__':
    import pr2_arms as pa
    rospy.init_node('epc_pr2', anonymous = True)
    rospy.logout('epc_pr2: ready')

    #pr2_arms = pa.PR2Arms(primary_ft_sensor='ati')
    pr2_arms = pa.PR2Arms(primary_ft_sensor='estimate')
    door_epc = Door_EPC(pr2_arms)

    r_arm, l_arm = 0, 1
    arm = r_arm

    tip = np.matrix([0.35, 0., 0.]).T
    pr2_arms.arms.set_tooltip(arm, tip)

    raw_input('Hit ENTER to close')
    pr2_arms.close_gripper(arm, effort=80)
    raw_input('Hit ENTER to start Door Opening')

    # for cabinets.
    #p1 = np.matrix([0.8, -0.40, -0.04]).T # pos 3
    #p1 = np.matrix([0.8, -0.10, -0.04]).T # pos 2
    p1 = np.matrix([0.8, -0.35, 0.1]).T # pos 1
    door_epc.search_and_hook(arm, p1, hook_direction='left')
    door_epc.pull(arm, force_threshold=40., cep_vel=0.05)

#    # hrl toolchest drawer.
#    p1 = np.matrix([0.8, -0.2, -0.17]).T
#    door_epc.search_and_hook(arm, p1, hook_direction='up')
#    door_epc.pull(arm, force_threshold=40., cep_vel=0.05)


