#
#
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)


import m3.toolbox as m3t

import mekabot.hrl_robot as hr
import mekabot.coord_frames as mcf
import util as uto
import camera

import segway_motion_calc as smc
import arm_trajectories as at

import roslib; roslib.load_manifest('2010_icra_epc_pull')
import rospy
from geometry_msgs.msg import Point32

# hrl ros imports
import hrl_hokuyo.hokuyo_scan as hs
import hrl_tilting_hokuyo.tilt_hokuyo_servo as ths
import hrl_lib.util as ut, hrl_lib.transforms as tr
import segway_vo.segway_command as sc
import vis_odometry_feat.vo_pose as vop
import zenither.zenither as zenither
import zenither.zenither_client as zc

from equilibrium_point_control.msg import MechanismKinematicsRot
from equilibrium_point_control.msg import MechanismKinematicsJac

import sys, time, os, optparse
import math, numpy as np
import copy

from opencv.cv import *
from opencv.highgui import *

from threading import RLock
import thread



##
# compute the end effector rotation matrix.
# @param hook - hook angle. RADIANS(0, -90, 90) (hor, up, down)
# @param angle - angle between robot and surface normal.
# Angle about the Z axis through which the robot must turn to face
# the surface.
def rot_mat_from_angles(hook, surface):
    rot_mat = tr.Rz(hook) * tr.Rx(surface) * tr.Ry(math.radians(-90))
    return rot_mat



class CompliantMotionGenerator():
    ''' a specific form of compliant motion.
        class name might be inappropriate.
    '''

    ##
    # @param move_segway - if True then move the segway while pulling.
    def __init__(self, move_segway=False, use_right_arm=True,
                 use_left_arm=True, set_wrist_theta_gc=False, end_effector_length=0.12818):

        self.mech_kinematics_lock = RLock()
        self.fit_circle_lock = RLock()

        if use_right_arm:
            # stiffness in Nm/rad: [20,50,15,25,2.5]
            #settings_r = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,0.75])
            # stiffness in Nm/rad: [20,50,20,25,2.5]
            settings_r = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.997,0.7272,0.75])
            #settings_r = hr.MekaArmSettings(stiffness_list=[0.7,0.7,0.9,0.9,0.7]) # for RIP project
        else:
            settings_r = None

        if use_left_arm:
            if set_wrist_theta_gc:
                settings_l = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,.8,.3,1.],
                        control_mode = 'wrist_theta_gc')
            else:
                settings_l = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,0.75])
            print 'left arm assigned'
            #settings_l = hr.MekaArmSettings(stiffness_list=[0.7,0.7,0.9,0.9,0.7]) # for RIP project
        else:
            settings_l = None
            print 'left arm NOT assigned'

        self.firenze = hr.M3HrlRobot(connect = True, left_arm_settings = settings_l,
                         right_arm_settings = settings_r, end_effector_length = end_effector_length)


        rospy.init_node('compliant_trajectory', anonymous = False)
        rospy.Subscriber('mechanism_kinematics_rot',
                         MechanismKinematicsRot,
                         self.mechanism_kinematics_rot_cb)
        rospy.Subscriber('mechanism_kinematics_jac',
                         MechanismKinematicsJac,
                         self.mechanism_kinematics_jac_cb)
        self.mech_traj_pub = rospy.Publisher('mechanism_trajectory', Point32)

        self.hok = hs.Hokuyo('utm', 0, flip = True, ros_init_node = False)
        self.thok = ths.tilt_hokuyo('/dev/robot/servo0',5,self.hok,l1=0.,l2=-0.055)

        self.cam = camera.Camera('mekabotUTM')
        self.set_camera_settings()

        self.z = zenither.Zenither(robot='HRL2', pose_read_thread=True)
        self.zenither_client = zc.ZenitherClient()
        self.zenither_list = []
        self.zenither_moving = False

        self.move_segway_flag = move_segway
        self.segway_trajectory = at.PlanarTajectory()
        self.move_segway_lock = RLock()
        self.segway_motion_tup = (0.0,0.0,0.0)
        self.new_segway_command = False

        self.dist_boundary = 0.05
        self.eq_pt_close_to_bndry = False # used in segway_motion_local

        self.workspace_dict = ut.load_pickle(os.environ['HRLBASEPATH']+'/src/projects/equilibrium_point_control/pkls/workspace_dict_2010Feb20_225427.pkl')

        if move_segway:
            self.segway_command_node = sc.SegwayCommand(vel_topic='/hrl/segway/command',
                                            pose_local_topic='/hrl/segway/pose_local',
                                            pose_global_topic='/hrl/segway/pose_global',
                                            stop_topic='/hrl/segway/stop',
                                            max_vel_topic='/hrl/segway/max_vel',
                                            ros_init_node=False)
            #self.segway_pose = vop.vo_pose('/hrl/segway/pose',ros_init_node=False)
            self.segway_pose = vop.vo_pose('pose',ros_init_node=False)

        self.eq_pt_not_moving_counter = 0

    def movement_posture(self):
        q = [math.radians(-45), 0, 0, math.radians(125.), 0, 0, 0]
        self.firenze.go_jointangles('right_arm',q)
        #self.firenze.go_jointangles('right_arm',q,arm2='left_arm',q2=q)

    def stop(self):
        self.run_fit_circle_thread = False
        self.run_move_segway_thread = False
        self.z.estop()
        self.z.broadcaster.stop()
        self.firenze.stop()

    ##
    # move the robot so that hooking location is at a better position
    # relative to the robot.
    # @param hook_location - 3x1 np matrix (in torso coord frame)
    # @param turn_angle - angle (in RADIANS) through which the segway
    # should rotate.
    # @param hook_angle - angle of the hook. (to determine good height)
    # @return new hooking location (3x1) in torso coord frame.
    def reposition_robot(self,hook_location,turn_angle,hook_angle,position_number=2):
        h = self.workspace_dict[hook_angle]['height']
        max_z_height = 1.3
        min_z_height = 0.5
        h_desired = self.z.get_position_meters()+hook_location[2,0]-h
        print 'h_desired:',h_desired
        print 'h:',h
        h_achieve = ut.bound(h_desired,max_z_height,min_z_height)
        h_err = h_desired-h_achieve
        h = h+h_err
        print 'h_achieve:',h_achieve

        wkd = self.workspace_dict[hook_angle]['pts']
        k = wkd.keys()
        k_idx = np.argmin(np.abs(np.array(k)-h))
        wrkspc_pts = wkd[k[k_idx]]

#        good_location = np.matrix([0.45,-0.35,h]).T
        good_location = wrkspc_pts.mean(1)
        good_location = np.matrix(good_location.A1.tolist()+[h]).T

        if position_number == 1:
            good_location[0,0] += 0.0
            good_location[1,0] -= 0.1
        elif position_number == 2:
            good_location[0,0] += 0.0
            good_location[1,0] -= 0.0
        elif position_number == 3:
            good_location[0,0] += 0.0
            good_location[1,0] += 0.1
        else:
            good_location[0,0] -= 0.1
            good_location[1,0] -= 0.0

        good_location = mcf.mecanumTglobal(mcf.globalTtorso(good_location))
        hook_location = mcf.mecanumTglobal(mcf.globalTtorso(hook_location))

        v = tr.Rz(-turn_angle)*good_location
        move_vector = hook_location - v

        pose1 = self.segway_pose.get_pose()
        self.segway_command_node.go_xya_pos_local(move_vector[0,0],move_vector[1,0],
                                                  turn_angle,blocking=False)

        self.z.torque_move_position(h_achieve)
        print 'before waiting for segway to stop moving.'
        self.segway_command_node.wait_for_tolerance_pos(0.03,0.03,math.radians(4))
        print 'after segway has stopped moving.'
        
        pose2 = self.segway_pose.get_pose()
        hl_new = vop.transform_pts_local(hook_location[0:2,:],pose1,pose2)
        h_err = h_desired-self.z.get_position_meters()

        g = np.matrix(hl_new.A1.tolist()+[good_location[2,0]+h_err]).T
        g = mcf.torsoTglobal(mcf.globalTmecanum(g))
        angle_turned = pose2[2]-pose1[2]
        angle_error = tr.angle_within_mod180(turn_angle-angle_turned)
        self.segway_pose.set_origin() # re-origining for the manipulation behaviors
        return g, angle_error

    def set_camera_settings(self):
        self.cam.set_frame_rate(30)
        self.cam.set_auto()
        self.cam.set_gamma(1)
        self.cam.set_whitebalance(r_val=512,b_val=544)

    ##
    # @param equi_pt_generator: function that returns stop, q  where q: list of 7 joint angles and  stop: string which is '' for compliant motion to continue
    # @param rapid_call_func: called in the time between calls to the equi_pt_generator can be used for logging, safety etc.  returns string which is '' for compliant motion to continue
    # @param time_step: time between successive calls to equi_pt_generator
    # @param arg_list - list of arguments to be passed to the equi_pt_generator
    # @return stop (the string which has the reason why the compliant
    # motion stopped.), JEP (last commanded JEP)
    def compliant_motion(self, equi_pt_generator, time_step, arm, arg_list, record=False,
                         rapid_call_func=None):

#        self.init_wrist_torque = self.firenze.get_wrist_torque(arm)
#        self.firenze.step()
        stop, q = equi_pt_generator(*arg_list)


        threshold = 100.
        ratio_j4 = 50.
        ratio_j5 = 100.

        t_end = time.time()
        while stop == '':
#            wrist_torque = self.firenze.get_wrist_torque(arm,base_frame=True)
#            self.firenze.step()
#            if follow_contours:
#                if abs(wrist_torque[0,0]) > threshold:     #If surface is applying a positive torque about roll axis...
#                    q[4] = q[4] + math.radians(wrist_torque[0,0]/ratio_j4) #roll the wrist in the same direction as the torque to get zero net torque.
#                    print 'Roll change: j4=', q[4]

#                if abs(wrist_torque[1,0]) > threshold:     #If surface is applying a positive torque about pitch axis...
#                    q[5] = q[5] - math.radians(wrist_torque[1,0]/ratio_j5) #pitch the wrist in the same direction as the torque to get zero net torque.
#                    print 'Pitch change: j5=', q[5]

            t_end += time_step
            self.firenze.set_joint_angles(arm, q)
            self.firenze.step()
            t1 = time.time()

			if record:
	            f = self.firenze.get_wrist_force(arm)
            	t = self.firenze.get_wrist_torque(arm,base_frame=True)
	            p = self.firenze.FK(arm, q)
	            print >> self.file, t1,f[0,0],f[1,0],f[2,0],t[0,0],t[1,0],t[2,0],p[0,0],p[1,0],p[2,0],q[0],q[1],q[2],q[3],q[4],q[5],q[6]

            if stop != '':
                break

            while t1<t_end:
                if rapid_call_func != None:
                    stop = rapid_call_func(arm)
                    if stop != '':
                        break
                self.firenze.step()
                t1 = time.time()

#            print 'Commanded JEP:', np.degrees(q)
#            print 'Just called equi_pt_generator once. Hit a key to loop again'
#            k=m3t.get_keystroke()

            stop,q = equi_pt_generator(*arg_list)

            if stop == 'reset timing':
                stop = ''
                t_end = time.time()

        return stop, q

    ## log the joint angles, equi pt joint angles and forces.
    def log_state(self, arm):
        t_now = time.time()
        q_now = self.firenze.get_joint_angles(arm)
        qdot_now = self.firenze.get_joint_velocities(arm)
        qdotdot_now = self.firenze.get_joint_accelerations(arm)

        tau_now = self.firenze.get_joint_torques(arm)
        self.jt_torque_trajectory.q_list.append(tau_now)
        self.jt_torque_trajectory.time_list.append(t_now)

        self.pull_trajectory.q_list.append(q_now)
        self.pull_trajectory.qdot_list.append(qdot_now)
        self.pull_trajectory.qdotdot_list.append(qdotdot_now)
        self.pull_trajectory.time_list.append(t_now)

        self.eq_pt_trajectory.q_list.append(self.q_guess) # see equi_pt_generator - q_guess is the config for the eq point.
        self.eq_pt_trajectory.time_list.append(t_now)
    
        wrist_force = self.firenze.get_wrist_force(arm, base_frame=True)
        self.force_trajectory.f_list.append(wrist_force.A1.tolist())
        self.force_trajectory.time_list.append(t_now)

        wrist_torque = self.firenze.get_wrist_torque(arm)
        self.torque_trajectory.f_list.append(wrist_torque.A1.tolist())
        self.torque_trajectory.time_list.append(t_now)

        if self.move_segway_flag:
            x,y,a = self.segway_pose.get_pose()
        else:
            x,y,a = 0.,0.,0.

        self.segway_trajectory.time_list.append(t_now)
        self.segway_trajectory.x_list.append(x)
        self.segway_trajectory.y_list.append(y)
        self.segway_trajectory.a_list.append(a)

        self.zenither_list.append(self.zenither_client.height())

        return '' # log_state also used as a rapid_call_func

    def common_stopping_conditions(self):
        stop = ''
        if self.q_guess == None:
            stop = 'IK fail'

        wrist_force = self.firenze.get_wrist_force('right_arm',base_frame=True)
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
            print stop
            #stop = ''

        curr_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[-1])
        if curr_pos[0,0] < 0.29 and curr_pos[1,0] > -0.2 and \
           curr_pos[2,0] > -0.28:
            print 'curr_pos:', curr_pos.A1.tolist()
            stop = 'danger of self collision'

        if self.firenze.is_motor_power_on() == False:
            stop = 'estop pressed'

        if self.eq_pt_not_moving_counter > 20:
            stop = 'unable to move equilibrium point away from bndry'

        return stop

    ##
    # @param arm - 'right_arm' or 'left_arm'
    # @param motion vec is in tl frame.
    # @param step_size - distance (meters) through which CEP should move
    # @param rot_mat - rotation matrix for IK
    # @return JEP
    def update_eq_point(self, arm, motion_vec, step_size, rot_mat):
        x,y,a,dx,dy,da = 0.,0.,0.,0.,0.,0.
        st = self.segway_trajectory
        if len(st.x_list)>0:
            x,y,a = st.x_list[-1],st.y_list[-1],st.a_list[-1]
        if len(st.x_list)>1:
            dx = x-st.x_list[-2]
            dy = y-st.y_list[-2]
            da = tr.angle_within_mod180(x-st.a_list[-2])

        self.eq_pt_cartesian = smc.tlTts(self.eq_pt_cartesian_ts,x,y,a)
        if len(self.zenither_list) > 2:
            h = self.zenither_list[-1] - self.zenither_list[-2]
            self.eq_pt_cartesian[2,0] -= h

        next_pt = self.eq_pt_cartesian + step_size * motion_vec

        if self.q_guess != None:
            if arm == 'right_arm':
                self.q_guess[1] = ut.bound(self.q_guess[1],math.radians(20),math.radians(5))

        q_eq = self.firenze.IK(arm,next_pt,rot_mat,self.q_guess)
        self.eq_pt_cartesian = next_pt
        self.eq_pt_cartesian_ts = smc.tsTtl(self.eq_pt_cartesian,x,y,a)
        self.q_guess = q_eq
        return q_eq

    def segway_mover(self):
        self.run_move_segway_thread = True
        print 'Starting the segway moving thread.'

        send_command = False
        while self.run_move_segway_thread:
            self.move_segway_lock.acquire()
            if self.new_segway_command == True:
                send_command = True
                self.new_segway_command = False
                t = self.segway_motion_tup
            self.move_segway_lock.release()

            if send_command:
                send_command = False
                if t == (0.,0.,0.):
                    self.segway_command_node.stop()
                else:
                    #self.segway_command_node.go_xya_pos_local(t[0],t[1],t[2],blocking=False)
                    self.segway_command_node.set_velocity(t[0],t[1],t[2])
                time.sleep(0.05)
            time.sleep(0.005)

        self.segway_command_node.stop()
        time.sleep(0.1)
        print 'Ended the segway moving thread.'

    def mechanism_kinematics_rot_cb(self, mk):
        self.fit_circle_lock.acquire()
        self.cx_start = mk.cx
        self.cy_start = mk.cy
        self.cz_start = mk.cz
        self.rad = mk.rad
        self.fit_circle_lock.release()

    # sa - segway pose from VO.
    def segway_motion_local(self, curr_pos_tl, sa):
        k = self.wkd['pts'].keys()
        z = min(self.eq_pt_cartesian[2,0], curr_pos_tl[2,0])
        k_idx = np.argmin(np.abs(np.array(k)-z))
        wrkspc_pts = self.wkd['pts'][k[k_idx]]
        bndry = self.wkd['bndry'][k[k_idx]]

        self.eq_pt_close_to_bndry = False
        if z < -0.4:
            # don't open the mechanism
            self.eq_motion_vec = np.matrix([0.,0.,0.]).T
            self.eq_pt_close_to_bndry = True

        if self.zenither_client.height() > 0.6:
            if z < -0.35 and self.zenither_moving == False:
                #self.z.nadir(1)
                self.z.nadir(0)
                self.zenither_moving = True
            if z > -0.25 and self.zenither_moving == True:
                self.z.estop()
                self.zenither_moving = False
        else:
            if self.zenither_moving:
                self.z.estop()
                self.zenither_moving = False

        ht = smc.segway_motion_repulse(curr_pos_tl, self.eq_pt_cartesian, bndry,
                                       wrkspc_pts)

        self.vec_bndry = smc.vec_from_boundary(self.eq_pt_cartesian, bndry)
        self.dist_boundary = smc.dist_from_boundary(self.eq_pt_cartesian, bndry,
                                                    wrkspc_pts)
        dist_bndry_ee = smc.dist_from_boundary(curr_pos_tl,bndry,wrkspc_pts)
        self.vec_bndry = self.vec_bndry/self.dist_boundary
        vec_bndry = self.vec_bndry
        dist_boundary = self.dist_boundary

        avel = -sa * 0.05 # maintaining the orientation of the segway.

        eq_mec = mcf.mecanumTglobal(mcf.globalTtorso(self.eq_pt_cartesian))
        svel_rot_y = -avel*eq_mec[0,0] # segway vel in y direction due to rotation
        svel_rot_x = avel*eq_mec[1,0] # segway vel in x direction due to rotation

        if self.zenither_moving:
            sp = 0.075
        else:
            sp = 0.075
        st = ht*sp + np.matrix([svel_rot_x,svel_rot_y]).T
        st[0,0] = min(st[0,0],0.) # disallowing +ve translation.

        self.move_segway_lock.acquire()
        self.segway_motion_tup = (st[0,0],st[1,0],avel)
        self.new_segway_command = True
        self.move_segway_lock.release()

        proj = (self.eq_motion_vec[0:2,0].T*vec_bndry)[0,0]
        proj_threshold = math.cos(math.radians(100))
        if (dist_boundary<= 0.04 and proj<proj_threshold) or \
            dist_boundary<0.03:
            #dist_boundary<0.01 or dist_bndry_ee < 0.04:
            self.eq_motion_vec = np.matrix([0.,0.,0.]).T
            self.eq_pt_close_to_bndry = True

        if self.eq_pt_close_to_bndry:
            if not self.zenither_moving:
                self.eq_pt_not_moving_counter += 1
            else:
                self.eq_pt_not_moving_counter = 0
        else:
            self.eq_pt_not_moving_counter = 0


        print '-------------------------------------'
        print 'segway_translation:',st[0,0],st[1,0]
        print 'avel:',math.degrees(avel)
        print 'dist_boundary:', dist_boundary
        print 'proj,proj_threshold:',proj,proj_threshold
        print 'self.eq_pt_cartesian:',self.eq_pt_cartesian.A1.tolist()
        print 'self.eq_pt_close_to_bndry?',self.eq_pt_close_to_bndry
        print 'dist_bndry_ee:',dist_bndry_ee

    ## constantly update the estimate of the kinematics and move the
    # equilibrium point along the tangent of the estimated arc, and
    # try to keep the radial force constant.
    # @param h_force_possible - True (hook side) or False (hook up).
    # @param v_force_possible - False (hook side) or True (hook up).
    # Is maintaining a radial force possible or not (based on hook
    # geometry and orientation)
    # @param cep_vel - tangential velocity of the cep in m/s
    def equi_pt_generator_control_radial_force(self, arm, rot_mat,
                           h_force_possible, v_force_possible, cep_vel):
        self.log_state(arm)
        step_size = 0.1 * cep_vel # 0.1 is the time interval between calls to the equi_generator function (see pull)
        q_eq = self.update_eq_point(arm, self.eq_motion_vec, step_size,
                                    rot_mat)

        stop = self.common_stopping_conditions()

        wrist_force = self.firenze.get_wrist_force(arm, base_frame=True)
        mag = np.linalg.norm(wrist_force)

        curr_pos_tl = self.firenze.FK(arm,self.pull_trajectory.q_list[-1])
        st = self.segway_trajectory
        x,y,a = st.x_list[-1],st.y_list[-1],st.a_list[-1]
        curr_pos_ts = smc.tsTtl(curr_pos_tl,x,y,a)
        curr_pos = curr_pos_ts
        if len(self.zenither_list) > 1:
            h = self.zenither_list[-1] - self.zenither_list[0]
            curr_pos[2,0] += h

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

            force_vec = smc.tlRts(force_vec_ts, a)
            tangential_vec = smc.tlRts(tangential_vec_ts, a)
            print 'tangential vec right at the transformation:', tangential_vec.A1
            print 'tangential vec ts right at the transformation:', tangential_vec_ts.A1
            print 'a:', a

        if self.use_rotation_center:
            self.fit_circle_lock.acquire()
            self.cartesian_pts_list.append(curr_pos.A1.tolist())
            rad = self.rad
            cx_start, cy_start = self.cx_start, self.cy_start
            cz_start = self.cz_start
            self.fit_circle_lock.release()
            c_ts = np.matrix([cx_start,cy_start,0.]).T
            c_tl = smc.tlTts(c_ts,x,y,a)
            cx,cy = c_tl[0,0],c_tl[1,0]
            cz = cz_start

            z_start = start_pos[2,0]
            parallel_to_floor = abs(z_start - cz) < 0.1
            print 'abs(z_start - cz)', abs(z_start - cz)
            if parallel_to_floor:
                print 'Mechanism is in the XY plane.'
                # trajectory is parallel to the ground.
                # find tangential direction.
                radial_vec_tl = curr_pos_tl-np.matrix([cx,cy,cz]).T
                radial_vec = radial_vec_tl
                radial_vec = radial_vec/np.linalg.norm(radial_vec)
                if cy_start<start_pos[1,0]:
            #        if cy<0.:
                    tan_x,tan_y = -radial_vec[1,0],radial_vec[0,0]
                else:
                    tan_x,tan_y = radial_vec[1,0],-radial_vec[0,0]
                
                if tan_x > 0. and (start_pos[0,0]-curr_pos[0,0]) < 0.09:
                    tan_x = -tan_x
                    tan_y = -tan_y

                if cy_start>start_pos[1,0]:
                    radial_vec = -radial_vec # axis to the left, want force in
                                           # anti-radial direction.
                rv = radial_vec
                force_vec = np.matrix([rv[0,0], rv[1,0], 0.]).T
                tangential_vec = np.matrix([tan_x, tan_y, 0.]).T
            else:
                print 'Mechanism is in the XZ plane.'
                # here the mechanism does not move in a plane parallel
                # to the ground.
                radial_vec_tl = curr_pos_tl-np.matrix([cx,cy,cz]).T
                radial_vec = radial_vec_tl
                radial_vec = radial_vec/np.linalg.norm(radial_vec)
                tan_x, tan_z = -radial_vec[2,0], radial_vec[0,0]
                
                if tan_x > 0. and (start_pos[0,0]-curr_pos[0,0]) < 0.09:
                    tan_x = -tan_x
                    tan_z = -tan_z

                rv = radial_vec
                force_vec = np.matrix([rv[0,0], 0., rv[2,0]]).T
                tangential_vec = np.matrix([tan_x, 0., tan_z]).T
            
            if abs(curr_pos[2,0] - z_start) > 0.03 and parallel_to_floor:
                force_vec += np.matrix([0.,0.,1]).T
                parallel_to_floor = False

            tangential_vec_ts = smc.tsRtl(tangential_vec, a)
            radial_vec_ts = smc.tsRtl(radial_vec, a)
            force_vec_ts = smc.tsRtl(force_vec, a)

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
        
        if self.move_segway_flag:
            self.segway_motion_local(curr_pos_tl, a)

        self.prev_force_mag = mag

        if self.init_tangent_vector == None:
            self.init_tangent_vector = copy.copy(tangential_vec_ts)
        c = np.dot(tangential_vec_ts.A1, self.init_tangent_vector.A1)
        ang = np.arccos(c)

        dist_moved = np.dot((curr_pos - start_pos).A1, self.tangential_vec_ts.A1)
        ftan = abs(np.dot(wrist_force.A1, tangential_vec.A1))
#        print 'wrist force:', wrist_force.A1
#        print 'tangential_vec:', tangential_vec.A1
#        print 'tangential_vec_ts:', tangential_vec_ts.A1
#        print 'ftan:', ftan
#        print 'force magnitude:', mag
        if abs(dist_moved) > 0.09 and self.hooked_location_moved == False:
            # change the force threshold once the hook has started pulling.
            self.hooked_location_moved = True
            self.eq_force_threshold = ut.bound(mag+30.,20.,80.)
            self.ftan_threshold = self.ftan_threshold + max(10., 1.5*ftan)
        if self.hooked_location_moved:
            if abs(tangential_vec_ts[2,0]) < 0.2 and ftan > self.ftan_threshold:
#                print 'ftan threshold exceed.'
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

        self.move_segway_lock.acquire()
        if stop != '' and stop != 'reset timing':
            self.segway_motion_tup = (0.0,0.0,0.0)
            self.new_segway_command = True
        self.move_segway_lock.release()

        self.mech_time_list.append(time.time())
        self.mech_x_list.append(ang)
        self.f_rad_list.append(f_rad_mag)
        self.f_tan_list.append(np.dot(f_vec, tangential_vec.A1))
        self.tan_vec_list.append(tangential_vec_ts.A1.tolist())
        self.rad_vec_list.append(force_vec_ts.A1.tolist())

        return stop, q_eq

    ## behavior to search around the hook_loc to try and get a good
    # hooking grasp
    # @param arm - 'right_arm' or 'left_arm'
    # @param hook_angle - radians(0,-90,90) side,up,down
    # @param hook_loc - 3x1 np matrix
    # @param angle - angle between torso x axis and surface normal.
    # @return s, jep (stopping string and last commanded JEP)
    def search_and_hook(self, arm, hook_angle, hook_loc, angle,
                        hooking_force_threshold = 5.):
        #rot_mat = tr.Rz(hook_angle) * tr.Rx(angle) * tr.Ry(math.radians(-90))
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
        self.firenze.go_cartesian(arm, pt1, rot_mat, speed=0.2)

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

    
    ## move along a vertical surface while maintaining a small for on it.
    # NOTE: stopping conditions assume we are trying to hook onto a
    # handle.
    # @param arm - 'left_arm' or 'right_arm'
    # @param move_dir - direction along which the end effector moves. (3x1 np matrix in tl frame)
    # @param rot_mat - matrix that defines the end effector
    # orientation. It transforms points in torso frame to end effector
    # frame.
    def equi_generator_surface_follow(self, arm, move_dir, rot_mat,
                                      force_threshold):
        wrist_force = self.firenze.get_wrist_force(arm, base_frame=True)
        if wrist_force[0,0] < -3.:
            self.eq_pt_cartesian_ts[0,0] -= 0.002
        if wrist_force[0,0] > -1.:
            self.eq_pt_cartesian_ts[0,0] += 0.003

        if self.eq_pt_cartesian_ts[0,0] > (self.start_pos[0,0]+0.05):
            self.eq_pt_cartesian_ts[0,0] = self.start_pos[0,0]+0.05

        q_eq = self.update_eq_point(arm, move_dir, 0.002, rot_mat)

        v = self.eq_pt_cartesian-self.start_pos
        if (wrist_force.T * move_dir)[0,0] < -force_threshold:
            stop = 'got a hook'
        elif np.linalg.norm(wrist_force) > 30.:
            stop = 'force is large %f'%(np.linalg.norm(wrist_force))
        elif (v.T * move_dir)[0,0] > 0.20:
            stop = 'moved a lot without a hook'
        else:
            stop = ''
        return stop, q_eq

    ##
    # assumes: 1) already hooked onto mechanism.
    # @param arm - 'left_arm' or 'right_arm'
    # @param hook_angle - RADIANS(0, -90, 90) (hor, up, down)
    # @param surface_angle - see rot_mat_from_angles
    # @param force_threshold - max force at which to stop pulling.
    # @param jep - last commanded JEP.
    # @param use_utm - to take 3D scans or not.
    # @param use_camera - to take pictures from the camera or not.
    # @param strategy - 'line_neg_x': move CEP along -x axis.
    #                   'control_radial_force': try and keep the radial force constant
    # @param info_string - string saved with key 'info' in the pkl.
    # @param cep_vel - tangential velocity of the cep in m/s
    # @param kinematics_estimation - 'rotation_center' or 'jacobian'
    # @param pull_left - UBER ADHOC. trying to specify initial motion direction (Advait, Feb 28. 2:50am)
    def pull(self, arm, hook_angle, surface_angle, force_threshold, jep, use_utm=False,
             use_camera=False, strategy='line_neg_x', info_string='',
             cep_vel = 0.1, kinematics_estimation='rotation_center',
             pull_left = False):

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

        if use_utm:
            self.firenze.step()
            armconfig1 = self.firenze.get_joint_angles('right_arm')
            plist1,slist1 = self.scan_3d()

        if use_camera:
            cam_plist1, cam_imlist1 = self.image_region()
        else:
            cam_plist1,cam_imlist1 = None,None

        #rot_mat = tr.Rz(hook_angle)*tr.Ry(math.radians(-90))
        rot_mat = rot_mat_from_angles(hook_angle, surface_angle)

        if self.move_segway_flag:
            self.segway_pose.set_origin()
            self.segway_command_node.set_max_velocity(0.15,0.1,math.radians(15))

        time_dict = {}
        time_dict['before_pull'] = time.time()

        stiffness_scale = self.firenze.arm_settings[arm].stiffness_scale

        self.pull_trajectory = at.JointTrajectory()
        self.jt_torque_trajectory = at.JointTrajectory()
        self.eq_pt_trajectory = at.JointTrajectory()
        self.force_trajectory = at.ForceTrajectory()
        self.torque_trajectory = at.ForceTrajectory()

        self.firenze.step()
        start_config = self.firenze.get_joint_angles(arm)

        self.eq_force_threshold = force_threshold
        self.ftan_threshold = 2.
        self.hooked_location_moved = False # flag to indicate when the hooking location started moving.
        self.prev_force_mag = np.linalg.norm(self.firenze.get_wrist_force(arm))
        self.eq_motion_vec = np.matrix([-1.,0.,0.]).T # might want to change this to account for the surface_angle.
        self.slip_count = 0

        self.eq_pt_cartesian = self.firenze.FK(arm, jep)
        self.eq_pt_cartesian_ts = self.firenze.FK(arm, jep)
        self.start_pos = copy.copy(self.eq_pt_cartesian)
        self.q_guess = jep

        if strategy == 'control_radial_force':
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
            ee_pos = self.firenze.end_effector_pos(arm)

            if self.use_rotation_center:
            # this might have to change depending on left and right
            # arm? or maybe not since the right arm can open both
            # doors.
                self.cx_start = ee_pos[0,0]
                self.cy_start = ee_pos[1,0]-1.0
                self.cz_start = ee_pos[2,0]
                self.rad = 5.0

            z = ee_pos[2,0]
            print 'ee_pos[2,0]:',z
            self.wkd = self.workspace_dict[hook_angle]
            k = self.wkd['pts'].keys()
            k_idx = np.argmin(np.abs(np.array(k)-z))
            self.wrkspc_pts = self.wkd['pts'][k[k_idx]]
            self.bndry = self.wkd['bndry'][k[k_idx]]
            #self.bndry = smc.compute_boundary(self.wrkspc_pts)

#            thread.start_new_thread(self.circle_estimator,())

            if self.move_segway_flag:
                thread.start_new_thread(self.segway_mover,())
                self.segway_command_node.set_max_velocity(0.1,0.1,math.radians(2.5))
            
            h_force_possible = abs(hook_angle) < math.radians(30.)
            v_force_possible = abs(hook_angle) > math.radians(60.)
            arg_list = [arm, rot_mat, h_force_possible, v_force_possible, cep_vel]
            result, jep = self.compliant_motion(self.equi_pt_generator_control_radial_force,
                                                0.1, arm, arg_list, self.log_state)
            if self.move_segway_flag:
                self.segway_command_node.stop()
            self.z.estop()
        else:
            raise RuntimeError('unknown pull strategy: ', strategy)

        if result == 'slip: force step decrease' or result == 'danger of self collision':
            self.firenze.motors_off()
            print 'powered off the motors.'

        print 'Compliant motion result:', result
        print 'Original force threshold:', force_threshold
        print 'Adapted force threshold:', self.eq_force_threshold
        print 'Adapted ftan threshold:', self.ftan_threshold

        time_dict['after_pull'] = time.time()

        d = {'actual': self.pull_trajectory, 'eq_pt': self.eq_pt_trajectory,
             'force': self.force_trajectory, 'torque': self.jt_torque_trajectory,
             'torque_ft': self.torque_trajectory,
             'stiffness': self.firenze.arm_settings[arm],
             'info': info_string, 'force_threshold': force_threshold,
             'eq_force_threshold': self.eq_force_threshold, 'hook_angle':hook_angle,
             'result':result, 'strategy':strategy, 'time_dict':time_dict,
             'segway':self.segway_trajectory, 'wrkspc':self.wrkspc_pts,
             'bndry':self.bndry, 'cep_vel': cep_vel,
             'zenither_list': self.zenither_list, 'ftan_threshold': self.ftan_threshold}

        self.firenze.step()
        if use_utm:
            armconfig2 = self.firenze.get_joint_angles(arm)
            plist2,slist2 = self.scan_3d()
            d['start_config'] = start_config
            d['armconfig1'] = armconfig1
            d['armconfig2'] = armconfig2
            d['l1'],d['l2'] = 0., -0.055
            d['scanlist1'], d['poslist1'] = slist1, plist1
            d['scanlist2'], d['poslist2'] = slist2, plist2

        d['cam_plist1'] = cam_plist1
        d['cam_imlist1'] = cam_imlist1

        ut.save_pickle(d,'pull_trajectories_'+d['info']+'_'+ut.formatted_time()+'.pkl')

        dd = {'mechanism_x': self.mech_x_list,
              'mechanism_time': self.mech_time_list,
              'force_rad_list': self.f_rad_list,
              'force_tan_list': self.f_tan_list,
              'tan_vec_list': self.tan_vec_list,
              'rad_vec_list': self.rad_vec_list
              }
        ut.save_pickle(dd,'mechanism_trajectories_robot_'+d['info']+'_'+ut.formatted_time()+'.pkl')

    def scan_3d(self):
        tilt_angles = (math.radians(20),math.radians(70))
        pos_list,scan_list = self.thok.scan(tilt_angles,speed=math.radians(10),save_scan=False)
        return pos_list,scan_list

    def save_frame(self):
        cvim = self.cam.get_frame()
        cvSaveImage('im_'+ut.formatted_time()+'.png',cvim)

    def image_region(self):
        ''' takes images from the UTM camera at different angles.
            returns list of servo angles, list of images.
            images are numpy images. so that they can be pickled.
        '''
        im_list = []
        p_list = []

        for cmd_ang in [0,30,45]:
            self.thok.servo.move_angle(math.radians(cmd_ang))
            cvim = self.cam.get_frame()
            cvim = self.cam.get_frame()
            im_list.append(uto.cv2np(cvim,format='BGR'))
            p_list.append(self.thok.servo.read_angle())

        self.thok.servo.move_angle(math.radians(0))
        return p_list,im_list

def test_IK(arm, rot_mat):
    ''' try out the IK at a number of different cartesian
        points in the workspace, with the given rotation
        matrix for the end effector.
    '''
    print 'press ENTER to start.'
    k=m3t.get_keystroke()
    while k=='\r':
        p = firenze.end_effector_pos(arm)
        firenze.go_cartesian(arm,p,rot_mat,speed=0.1)
        firenze.step()
        print 'press ENTER to save joint angles.'
        k=m3t.get_keystroke()
        if k == '\r':
            firenze.step()
            q = firenze.get_joint_angles(arm)
            ut.save_pickle(q,'arm_config_'+ut.formatted_time()+'.pkl')
        print 'press ENTER for next IK test. something else to exit.'
        k=m3t.get_keystroke()

def test_elbow_angle():
    firenze = hr.M3HrlRobot(connect=False)
    rot_mat = tr.Rz(math.radians(-90.))*tr.Ry(math.radians(-90))

    x_list = [0.55,0.45,0.35]
    y = -0.2
    z = -0.23
    for x in x_list:
        p0 = np.matrix([x,y,z]).T
        q = firenze.IK('right_arm',p0,rot_mat)
    #        q[1] = math.radians(15)
    #        q = firenze.IK('right_arm',p0,rot_mat,q_guess = q)
        elbow_angle = firenze.elbow_plane_angle('right_arm',q)
        print '---------------------------------------'
        print 'ee position:', p0.A1
    #        print 'joint angles:', q
        print 'elbow_angle:', math.degrees(elbow_angle)


if __name__=='__main__':
    p = optparse.OptionParser()
    p.add_option('--ik_single_pos', action='store_true', dest='ik_single_pos',
                 help='test IK at a single position.')
    p.add_option('--ik_test', action='store_true', dest='ik_test',
                 help='test IK in a loop.')
    p.add_option('--la', action='store_true', dest='la',
                 help='use the left arm.')
    p.add_option('--pull', action='store_true', dest='pull',
                 help='pull.')
    p.add_option('--search_hook', action='store_true', dest='search_hook',
                 help='search for a good hooking grasp.')
    p.add_option('--scan', action='store_true', dest='scan',
                 help='take and save 3D scans. specify --pull also.')
    p.add_option('--camera', action='store_true', dest='camera',
                 help='take and save images from UTM camera. specify --pull also.')
    p.add_option('--ha', action='store', dest='ha',type='float',
                 default=None,help='hook angle (degrees).')
    p.add_option('--ft', action='store', dest='ft',type='float',
                 default=None,help='force threshold (Newtons).')
    p.add_option('--info', action='store', type='string', dest='info_string',
                 help='string to save in the pkl log.', default='')
    p.add_option('--eaf', action='store_true', dest='eaf',
                 help='test elbow angle finding with the horizontal plane.')
    p.add_option('--move_segway', action='store_true', dest='ms',
                 help='move the segway while pulling')
    p.add_option('--test_reposition', action='store_true', dest='rf',
                 help='test repositioning the robot. Requires --move_segway')

    opt, args = p.parse_args()

    scan_flag = opt.scan
    camera_flag = opt.camera
    move_segway_flag  = opt.ms

    ha = opt.ha
    ft = opt.ft
    info_string = opt.info_string

    elbow_angle_flag = opt.eaf
    reposition_flag = opt.rf
    ik_single_pos_flag = opt.ik_single_pos
    test_ik_flag = opt.ik_test
    pull_flag = opt.pull
    search_hook_flag = opt.search_hook or pull_flag

    try:
        if pull_flag or reposition_flag or search_hook_flag:
            if ha == None:
                print 'please specify hook angle (--ha)'
                print 'Exiting...'
                sys.exit()

            if ft == None and pull_flag:
                print 'please specify force threshold (--ft) along with --pull'
                print 'Exiting...'
                sys.exit()

            if opt.la:
                use_left_arm = True
                use_right_arm = False
                arm = 'left_arm'
            else:
                use_left_arm = False
                use_right_arm = True
                arm = 'right_arm'

            cmg = CompliantMotionGenerator(move_segway_flag, use_right_arm,
                                           use_left_arm)

            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            cmg.firenze.power_on()

            if reposition_flag:
                rot_mat = tr.Rz(hook_angle)*tr.Ry(math.radians(-90))
                cmg.firenze.pose_robot(arm,rot_mat)
                hook_location = cmg.firenze.end_effector_pos(arm)
                g = cmg.reposition_robot(hook_location)
                cmg.firenze.go_cartesian(arm,g,rot_mat,speed=0.1)

            if search_hook_flag:
                hook_angle = math.radians(ha)
                surface_angle = math.radians(0.)
                p = np.matrix([0.45,-0.2,-0.23]).T
                if arm == 'left_arm':
                    p[1,0] = p[1,0] * -1
                print 'hit a key to search and hook.'
                k=m3t.get_keystroke()
                res, jep = cmg.search_and_hook(arm, hook_angle, p,
                                               surface_angle)
                print 'Search and Hook result:', res

            if pull_flag:
                cmg.pull(arm, hook_angle, surface_angle, ft, jep, scan_flag,
                         camera_flag, 'control_radial_force', info_string)

            
            print 'hit  a key to end everything'
            k=m3t.get_keystroke()
            cmg.stop()


        #----------- non-class functions test --------------------
        if elbow_angle_flag:
            test_elbow_angle()

        if ik_single_pos_flag or test_ik_flag:
            if ha == None:
                raise RuntimeError('You need to specify a hooking angle (--ha)')

            #p = np.matrix([0.45,-0.2,-0.23]).T
            p = np.matrix([0.55,-0.2,-0.23]).T
            if opt.la:
                arm = 'left_arm'
                p[1,0] = p[1,0] * -1
                settings_l = hr.MekaArmSettings(stiffness_list=[0.15,0.7,0.8,0.8,0.8])
                settings_r = None
            else:
                arm = 'right_arm'
                settings_l = None
                settings_r = hr.MekaArmSettings(stiffness_list=[0.15,0.7,0.8,0.8,0.8])

            firenze = hr.M3HrlRobot(connect = True, right_arm_settings = settings_r,
                                    left_arm_settings = settings_l)
            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            firenze.power_on()

            print 'hit a key to test IK'
            k=m3t.get_keystroke()


            rot_mat = tr.Rz(math.radians(ha))*tr.Ry(math.radians(-90))
            firenze.go_cartesian(arm, p, rot_mat, speed=0.1)

            if test_ik_flag:
                test_IK(arm, p, rot_mat)

            print 'hit  a key to end everything'
            k=m3t.get_keystroke()
            firenze.stop()

    except (KeyboardInterrupt, SystemExit):
        cmg.stop()




