#
# Copyright (c) 2009, Georgia Tech Research Corporation
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

# Author: Advait Jain

import m3.toolbox as m3t

import hokuyo.hokuyo_scan as hs
import tilting_hokuyo.tilt_hokuyo_servo as ths
import mekabot.hrl_robot as hr
import hrl_lib.util as ut, hrl_lib.transforms as tr
import util as uto
import camera

import sys, time, os, optparse
import math, numpy as np
import copy

import arm_trajectories as at

from opencv.cv import *
from opencv.highgui import *


from threading import RLock
import threading

hook_3dprint_angle = math.radians(20-2.54)

class CompliantMotionGenerator(threading.Thread):
    ''' a specific form of compliant motion.
        class name might be inappropriate.
    '''
    def __init__(self):
        # stiffness in Nm/rad: [20,50,15,25]
        self.settings_r = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.748,0.7272,0.8])
#        self.settings_r = hr.MekaArmSettings(stiffness_list=[0.2,1.0,1.0,0.4,0.8])

        self.settings_stiff = hr.MekaArmSettings(stiffness_list=[0.8,1.0,1.0,1.0,0.8])
        self.firenze = hr.M3HrlRobot(connect=True,right_arm_settings=self.settings_stiff)

        self.hok = hs.Hokuyo('utm',0,flip=True)
        self.thok = ths.tilt_hokuyo('/dev/robot/servo0',5,self.hok,l1=0.,l2=-0.055)

        self.cam = camera.Camera('mekabotUTM')
        self.set_camera_settings()

        self.fit_circle_lock = RLock()
        threading.Thread.__init__(self)

    def run(self):
        self.circle_estimator()

    def stop(self):
        self.run_fit_circle_thread = False
        self.firenze.stop()

    ## pose the arm by moving the end effector to the hookable location.
    # @param hook_angle - RADIANS(0, -90, 90 etc.)
    # 0 - horizontal, -pi/2 hook points up, +pi/2 hook points down
    def pose_arm(self,hook_angle):
        print 'press ENTER to pose the robot.'
        k=m3t.get_keystroke()

        if k!='\r':
            print 'You did not press ENTER.'
            return

#        settings_r_orig = copy.copy(self.firenze.arm_settings['right_arm'])
        settings_torque_gc = hr.MekaArmSettings(stiffness_list=[0.,0.,0.,0.,0.],control_mode='torque_gc')
        self.firenze.set_arm_settings(settings_torque_gc,None)
        self.firenze.step()
        print 'hit ENTER to end posing, something else to exit'
        k=m3t.get_keystroke()

        p = self.firenze.end_effector_pos('right_arm')

        q = self.firenze.get_joint_angles('right_arm')
#        self.firenze.set_arm_settings(settings_r_orig,None)
        self.firenze.set_arm_settings(self.settings_stiff,None)
        self.firenze.set_joint_angles('right_arm',q)
        self.firenze.step()
        self.firenze.set_joint_angles('right_arm',q)
        self.firenze.step()

        rot_mat = tr.Rz(hook_angle-hook_3dprint_angle)*tr.Ry(math.radians(-90))
        self.firenze.go_cartesian('right_arm',p,rot_mat,speed=0.1)

        print 'hit ENTER after making finer adjustment, something else to exit'
        k=m3t.get_keystroke()
        p = self.firenze.end_effector_pos('right_arm')
        q = self.firenze.get_joint_angles('right_arm')
        self.firenze.set_joint_angles('right_arm',q)
        self.firenze.step()

    def set_camera_settings(self):
        self.cam.set_frame_rate(30)
        self.cam.set_auto()
        self.cam.set_gamma(1)
        self.cam.set_whitebalance(r_val=512,b_val=544)

    def compliant_motion(self,equi_pt_generator,time_step,rapid_call_func=None):
        ''' equi_pt_generator: function that returns stop,q
                                 q: list of 7 joint angles
                                 stop: string which is '' for compliant motion to continue
            rapid_call_func: called in the time between calls to the equi_pt_generator
                             can be used for logging, safety etc.
                             returns stop
                                 stop: string which is '' for compliant motion to continue
            time_step: time between successive calls to equi_pt_generator

            returns stop (the string which has the reason why the compliant motion stopped.)
        '''
        
        stop,q = equi_pt_generator()
        while stop == '':
            self.firenze.set_joint_angles('right_arm',q)
            t1 = time.time()
            t_end = t1+time_step
            while t1<t_end:
                self.firenze.step()
                if rapid_call_func != None:
                    stop = rapid_call_func()
                    if stop != '':
                        break
                t1 = time.time()

            if stop != '':
                break
            stop,q = equi_pt_generator()
        return stop

    ## log the joint angles, equi pt joint angles and forces.
    def log_state(self):
        t_now = time.time()
        q_now = self.firenze.get_joint_angles('right_arm')
        qdot_now = self.firenze.get_joint_velocities('right_arm')

        tau_now = self.firenze.get_joint_torques('right_arm')
        self.jt_torque_trajectory.q_list.append(tau_now)
        self.jt_torque_trajectory.time_list.append(t_now)

        self.pull_trajectory.q_list.append(q_now)
        self.pull_trajectory.qdot_list.append(qdot_now)
        self.pull_trajectory.time_list.append(t_now)

        #self.eq_pt_trajectory.p_list.append(self.eq_pt_cartesian.A1.tolist())
        self.eq_pt_trajectory.q_list.append(self.q_guess) # see equi_pt_generator - q_guess is the config for the eq point.
        self.eq_pt_trajectory.time_list.append(t_now)
    
        wrist_force = self.firenze.get_wrist_force('right_arm',base_frame=True)
        self.force_trajectory.f_list.append(wrist_force.A1.tolist())
        self.force_trajectory.time_list.append(t_now)

    def common_stopping_conditions(self):
        stop = ''
        if self.q_guess == None:
            stop = 'IK fail'

        wrist_force = self.firenze.get_wrist_force('right_arm',base_frame=True)
        mag = np.linalg.norm(wrist_force)
        if mag > self.eq_force_threshold:
            stop = 'force exceed'

        if mag < 1.2 and self.hooked_location_moved:
            if (self.prev_force_mag - mag) > 10.:
                stop = 'slip: force step decrease and below thresold.'
            else:
                self.slip_count += 1
        else:
            self.slip_count = 0

        if self.slip_count == 4:
            stop = 'slip: force below threshold for too long.'

        curr_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[-1])
        if curr_pos[0,0]<0.27 and curr_pos[1,0]>-0.2:
            stop = 'danger of self collision'

        return stop

    def update_eq_point(self,motion_vec,step_size):
        next_pt = self.eq_pt_cartesian + step_size * motion_vec
        rot_mat = self.eq_IK_rot_mat
#        self.q_guess[1] += math.radians(1)
        q_eq = self.firenze.IK('right_arm',next_pt,rot_mat,self.q_guess)
        self.eq_pt_cartesian = next_pt
        self.q_guess = q_eq
        return q_eq

    def circle_estimator(self):
        self.run_fit_circle_thread = True
        print 'Starting the circle estimating thread.'

        while self.run_fit_circle_thread:
            self.fit_circle_lock.acquire()
            if len(self.cartesian_pts_list)==0:
                self.fit_circle_lock.release()
                continue
            pts_2d = (np.matrix(self.cartesian_pts_list).T)[0:2,:]
            self.fit_circle_lock.release()

            rad = self.rad_guess
            start_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[0])
            rad,cx,cy = at.fit_circle(rad,start_pos[0,0],start_pos[1,0]-rad,pts_2d,method='fmin_bfgs',verbose=False)
            rad = ut.bound(rad,3.0,0.1)

            self.fit_circle_lock.acquire()
            self.cx = cx
            self.cy = cy
    #        self.rad = rad
            self.fit_circle_lock.release()

        print 'Ended the circle estimating thread.'


    ## constantly update the estimate of the kinematics and move the
    # equilibrium point along the tangent of the estimated arc, and
    # try to keep the radial force constant.
    def equi_pt_generator_control_radial_force(self):
        self.log_state()
        q_eq = self.update_eq_point(self.eq_motion_vec,0.01)
        stop = self.common_stopping_conditions()

        wrist_force = self.firenze.get_wrist_force('right_arm',base_frame=True)
        mag = np.linalg.norm(wrist_force)

        start_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[0])
        curr_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[-1])
        if (start_pos[0,0]-curr_pos[0,0])>0.09 and self.hooked_location_moved==False:
            # change the force threshold once the hook has started pulling.
            self.hooked_location_moved = True
            self.eq_force_threshold = ut.bound(mag+30.,20.,80.)
            self.piecewise_force_threshold = ut.bound(mag+5.,0.,80.)
            
        self.fit_circle_lock.acquire()
        self.cartesian_pts_list.append(curr_pos.A1.tolist())
        self.fit_circle_lock.release()

        # find tangential direction.
        radial_vec = curr_pos[0:2]-np.matrix([self.cx,self.cy]).T
        radial_vec = radial_vec/np.linalg.norm(radial_vec)
        tan_x,tan_y = -radial_vec[1,0],radial_vec[0,0]
        
        if tan_x >0.:
            tan_x = -tan_x
            tan_y = -tan_y

        self.eq_motion_vec = np.matrix([tan_x,tan_y,0.]).T

        f_vec = -1*np.array([wrist_force[0,0],wrist_force[1,0]])
        f_rad_mag = np.dot(f_vec,radial_vec.A1)
        #if f_rad_mag>10.:
        if f_rad_mag>5.:
            self.eq_motion_vec[0:2] -= radial_vec/2. * self.hook_maintain_dist_plane/0.05
        else:
            self.eq_motion_vec[0:2] += radial_vec/2. * self.hook_maintain_dist_plane/0.05

        self.prev_force_mag = mag
        return stop,q_eq

    ## moves eq point along the -x axis.
    def equi_pt_generator_line(self):
        self.log_state()
        #q_eq = self.update_eq_point(self.eq_motion_vec,0.005)
        q_eq = self.update_eq_point(self.eq_motion_vec,0.010)
        stop = self.common_stopping_conditions()

        wrist_force = self.firenze.get_wrist_force('right_arm',base_frame=True)
        mag = np.linalg.norm(wrist_force)

        start_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[0])
        curr_pos = self.firenze.FK('right_arm',self.pull_trajectory.q_list[-1])
        if (start_pos[0,0]-curr_pos[0,0])>0.09 and self.hooked_location_moved==False:
            # change the force threshold once the hook has started pulling.
            self.hooked_location_moved = True
            self.eq_force_threshold = ut.bound(mag+15.,20.,80.)
            
        self.prev_force_mag = mag
        return stop,q_eq

    ## move the end effector to properly hook onto the world
    # direction of motion governed by the hook angle.
    # @param hook_angle - angle of hook in RADIANS (see pose_arm or pull for details.)
    def get_firm_hook(self, hook_angle):
        rot_mat = tr.Rz(hook_angle-hook_3dprint_angle)*tr.Ry(math.radians(-90))

        # move in the +x until contact.
        vec = np.matrix([0.08,0.,0.]).T
        self.firenze.move_till_hit('right_arm',vec=vec,force_threshold=2.0,rot=rot_mat,
                                   speed=0.05)

        # now move in direction of hook.
        vec = tr.rotX(-hook_angle) * np.matrix([0.,0.05,0.]).T
        self.firenze.move_till_hit('right_arm',vec=vec,force_threshold=5.0,rot=rot_mat,
                                   speed=0.05,bias_FT=False)
        self.firenze.set_arm_settings(self.settings_r,None)
        self.firenze.step()

    def pull(self,hook_angle,force_threshold,use_utm=False,use_camera=False,strategy='line_neg_x',
             pull_loc=None, info_string=''):
        ''' force_threshold - max force at which to stop pulling.
            hook_angle - radians(0, -90, 90 etc.)
                         0 - horizontal, -pi/2 hook points up, +pi/2 hook points down
            use_utm - to take 3D scans or not.
            use_camera - to take pictures from the camera or not.
            strategy - 'line_neg_x': move eq point along -x axis.
                       'piecewise_linear': try and estimate circle and move along it.
                       'control_radial_force': try and keep the radial force constant
                       'control_radial_dist'
            pull_loc - 3x1 np matrix of location for pulling. If None then arm will go into
                       gravity comp and user can show the location.
            info_string - string saved with key 'info' in the pkl.
        '''
        if use_utm:
            self.firenze.step()
            armconfig1 = self.firenze.get_joint_angles('right_arm')
            plist1,slist1 = self.scan_3d()

        if use_camera:
            cam_plist1, cam_imlist1 = self.image_region()
        else:
            cam_plist1,cam_imlist1 = None,None

        rot_mat = tr.Rz(hook_angle-hook_3dprint_angle)*tr.Ry(math.radians(-90))

        if pull_loc == None:
            self.pose_arm(hook_angle)
            pull_loc = self.firenze.end_effector_pos('right_arm')
            ut.save_pickle(pull_loc,'pull_loc_'+info_string+'_'+ut.formatted_time()+'.pkl')
        else:
            pt1 = copy.copy(pull_loc)
            pt1[0,0] = pt1[0,0]-0.1
            print 'pt1:', pt1.A1
            print 'pull_loc:', pull_loc.A1
            self.firenze.go_cartesian('right_arm',pt1,rot_mat,speed=0.2)
            self.firenze.go_cartesian('right_arm',pull_loc,rot_mat,speed=0.07)


        print 'press ENTER to pull'
        k=m3t.get_keystroke()
        if k != '\r':
            return

        time_dict = {}
        time_dict['before_hook'] = time.time()
        print 'first getting a good hook'
        self.get_firm_hook(hook_angle)
        time.sleep(0.5)
        time_dict['before_pull'] = time.time()

        print 'pull begins'
        stiffness_scale = self.settings_r.stiffness_scale
        vec = tr.rotX(-hook_angle) * np.matrix([0.,0.05/stiffness_scale,0.]).T
        self.keep_hook_vec = vec
        self.hook_maintain_dist_plane = np.dot(vec.A1,np.array([0.,1.,0.]))
        self.eq_pt_cartesian = self.firenze.end_effector_pos('right_arm') + vec
        q_eq = self.firenze.IK('right_arm',self.eq_pt_cartesian,rot_mat)
        self.firenze.go_jointangles('right_arm',q_eq,speed=math.radians(30))
        self.q_guess = q_eq

#        self.q_guess = self.firenze.get_joint_angles('right_arm')

        self.pull_trajectory = at.JointTrajectory()
        self.jt_torque_trajectory = at.JointTrajectory()
        self.eq_pt_trajectory = at.JointTrajectory()
        self.force_trajectory = at.ForceTrajectory()

        self.firenze.step()
        start_config = self.firenze.get_joint_angles('right_arm')

        self.eq_IK_rot_mat = rot_mat # for equi pt generators.
        self.eq_force_threshold = force_threshold
        self.hooked_location_moved = False # flag to indicate when the hooking location started moving.
        self.prev_force_mag = np.linalg.norm(self.firenze.get_wrist_force('right_arm'))
        self.eq_motion_vec = np.matrix([-1.,0.,0.]).T
        self.slip_count = 0
        if strategy == 'line_neg_x':
            result = self.compliant_motion(self.equi_pt_generator_line,0.025)
        elif strategy == 'control_radial_force':
            self.cartesian_pts_list = []
            self.piecewise_force_threshold = force_threshold
            self.rad_guess = 1.0
            self.cx = 0.6
            self.cy = -self.rad_guess
            self.start() # start the circle estimation thread
            result = self.compliant_motion(self.equi_pt_generator_control_radial_force,0.025)
        else:
            raise RuntimeError('unknown pull strategy: ', strategy)

        if result == 'slip: force step decrease' or result == 'danger of self collision':
            self.firenze.motors_off()
            print 'powered off the motors.'

        print 'Compliant motion result:', result
        print 'Original force threshold:', force_threshold
        print 'Adapted force threshold:', self.eq_force_threshold

        time_dict['after_pull'] = time.time()

        d = {'actual': self.pull_trajectory, 'eq_pt': self.eq_pt_trajectory,
             'force': self.force_trajectory, 'torque': self.jt_torque_trajectory,
             'stiffness': self.firenze.arm_settings['right_arm'],
             'info': info_string, 'force_threshold': force_threshold,
             'eq_force_threshold': self.eq_force_threshold, 'hook_angle':hook_angle,
             'result':result,'strategy':strategy,'time_dict':time_dict}

        self.firenze.step()
        armconfig2 = self.firenze.get_joint_angles('right_arm')
        if use_utm:
            plist2,slist2 = self.scan_3d()
            d['start_config']=start_config
            d['armconfig1']=armconfig1
            d['armconfig2']=armconfig2
            d['l1'],d['l2']=0.,-0.055
            d['scanlist1'],d['poslist1']=slist1,plist1
            d['scanlist2'],d['poslist2']=slist2,plist2

        d['cam_plist1']=cam_plist1
        d['cam_imlist1']=cam_imlist1

        ut.save_pickle(d,'pull_trajectories_'+d['info']+'_'+ut.formatted_time()+'.pkl')

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

def test_IK(rot_mat):
    ''' try out the IK at a number of different cartesian
        points in the workspace, with the given rotation
        matrix for the end effector.
    '''
    print 'press ENTER to start.'
    k=m3t.get_keystroke()
    while k=='\r':
        p = firenze.end_effector_pos('right_arm')
        firenze.go_cartesian('right_arm',p,rot_mat,speed=0.1)
        firenze.step()
        print 'press ENTER to save joint angles.'
        k=m3t.get_keystroke()
        if k == '\r':
            firenze.step()
            q = firenze.get_joint_angles('right_arm')
            ut.save_pickle(q,'arm_config_'+ut.formatted_time()+'.pkl')
        print 'press ENTER for next IK test. something else to exit.'
        k=m3t.get_keystroke()

def test_elbow_angle():
    firenze = hr.M3HrlRobot(connect=False)
    hook_3dprint_angle = math.radians(20-2.54)
    rot_mat = tr.Rz(math.radians(-90.)-hook_3dprint_angle)*tr.Ry(math.radians(-90))

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
    p.add_option('--pull', action='store_true', dest='pull',
                 help='pull with hook up (name will be changed later).')
    p.add_option('--pull_pos', action='store', type='string', dest='pull_pos_pkl',
                 help='pkl file with 3D coord of point to start pulling at.', default='')
    p.add_option('--firm_hook', action='store_true', dest='firm_hook',
                 help='test getting a firm hook on things.')
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
    p.add_option('--ve', action='store_true', dest='ve',
                 help='vary experiment. (vary stiffness settings and repeatedly pull)')
    p.add_option('--eaf', action='store_true', dest='eaf',
                 help='test elbow angle finding with the horizontal plane.')

    opt, args = p.parse_args()
    ik_single_pos_flag = opt.ik_single_pos
    test_ik_flag = opt.ik_test
    pull_flag = opt.pull
    pull_pos_pkl = opt.pull_pos_pkl
    firm_hook_flag = opt.firm_hook
    scan_flag = opt.scan
    camera_flag = opt.camera
    ha = opt.ha
    ft = opt.ft
    info_string = opt.info_string
    vary_expt_flag = opt.ve
    elbow_angle_flag = opt.eaf

    try:
        if vary_expt_flag:
            stiff_scale_list = [1.0,1.2,0.8]
            if pull_pos_pkl != '':
                pull_loc = ut.load_pickle(pull_pos_pkl)
            else:
                raise RuntimeError('Need to specify a pull_pos with vary_expt')

            cmg = CompliantMotionGenerator()
            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            cmg.firenze.power_on()
                
            #for strategy in ['line_neg_x','control_radial_force']:
            for strategy in ['line_neg_x','control_radial_dist','control_radial_force']:
            #for strategy in ['line_neg_x']:
            #for strategy in ['piecewise_linear']:
                for s_scale in stiff_scale_list:
                    cmg.settings_r.stiffness_scale = s_scale
                    cmg.pull(math.radians(ha), ft,use_utm=scan_flag,use_camera=camera_flag,
                             strategy=strategy,pull_loc=pull_loc,info_string=info_string)
                    cmg.firenze.maintain_configuration()
                    cmg.firenze.motors_on()
                    cmg.firenze.set_arm_settings(cmg.settings_stiff,None)
                    time.sleep(0.5)

            print 'hit  a key to end everything'
            k=m3t.get_keystroke()
            cmg.firenze.stop()

            sys.exit()

        if pull_flag or firm_hook_flag:
            if ha == None:
                print 'please specify hook angle (--ha)'
                print 'Exiting...'
                sys.exit()

            if ft == None and pull_flag:
                print 'please specify force threshold (--ft) along with --pull'
                print 'Exiting...'
                sys.exit()

            cmg = CompliantMotionGenerator()

            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            cmg.firenze.power_on()

            if pull_flag:
                if pull_pos_pkl != '':
                    pull_loc = ut.load_pickle(pull_pos_pkl)
                else:
                    pull_loc = None

    #            cmg.pull(math.radians(ha), ft,use_utm=scan_flag,use_camera=camera_flag,
    #                     strategy = 'control_radial_dist',pull_loc=pull_loc,info_string=info_string)
    #            cmg.pull(math.radians(ha), ft,use_utm=scan_flag,use_camera=camera_flag,
    #                     strategy = 'piecewise_linear',pull_loc=pull_loc,info_string=info_string)
                cmg.pull(math.radians(ha), ft,use_utm=scan_flag,use_camera=camera_flag,
                         strategy = 'control_radial_force',pull_loc=pull_loc,info_string=info_string)
    #            cmg.pull(math.radians(ha), ft,use_utm=scan_flag,use_camera=camera_flag,
    #                     strategy = 'line_neg_x',pull_loc=pull_loc,info_string=info_string)

            if firm_hook_flag:
                hook_angle = math.radians(ha)
                p = np.matrix([0.3,-0.25,-0.2]).T
                rot_mat = tr.Rz(hook_angle-hook_3dprint_angle)*tr.Ry(math.radians(-90))
                cmg.firenze.go_cartesian('right_arm',p,rot_mat,speed=0.1)
                print 'hit a key to get a firm hook.'
                k=m3t.get_keystroke()
                cmg.get_firm_hook(hook_angle)

            print 'hit  a key to end everything'
            k=m3t.get_keystroke()
            cmg.stop()

    #        cmg = CompliantMotionGenerator()
    #        print 'hit a key to test IK'
    #        k=m3t.get_keystroke()
    #        cmg.get_firm_hook(ha)

        #----------- non-class functions test --------------------
        if elbow_angle_flag:
            test_elbow_angle()

        if ik_single_pos_flag or test_ik_flag:
            if ha == None:
                raise RuntimeError('You need to specify a hooking angle (--ha)')

            settings_r = hr.MekaArmSettings(stiffness_list=[0.15,0.7,0.8,0.8,0.8])
            firenze = hr.M3HrlRobot(connect=True,right_arm_settings=settings_r)
            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            firenze.power_on()

            print 'hit a key to test IK'
            k=m3t.get_keystroke()
            #p = np.matrix([0.26,-0.25,-0.25]).T
            p = np.matrix([0.45,-0.2,-0.23]).T
            rot_mat = tr.Rz(math.radians(ha)-hook_3dprint_angle)*tr.Ry(math.radians(-90))
            #rot_mat = tr.Rz(math.radians(0))*tr.Ry(math.radians(-90))
            firenze.go_cartesian('right_arm',p,rot_mat,speed=0.1)


            if test_ik_flag:
                rot_mat = tr.Rz(math.radians(-110))*tr.Ry(math.radians(-90))
                #rot_mat = tr.Rz(math.radians(0))*tr.Ry(math.radians(-90))
                test_IK(rot_mat)

            print 'hit  a key to end everything'
            k=m3t.get_keystroke()
            firenze.stop()

    except (KeyboardInterrupt, SystemExit):
        cmg.stop()



