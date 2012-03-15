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


import sys, time, os, optparse
import math, numpy as np
import copy

import mekabot.coord_frames as mcf
import compliant_trajectories as ct
import m3.toolbox as m3t

import roslib; roslib.load_manifest('2010_icra_epc_pull')
import hrl_lib.util as ut
import hrl_lib.transforms as tr

if __name__=='__main__':
    p = optparse.OptionParser()
    p.add_option('--ha', action='store', dest='ha',type='float',
                 default=None,help='hook angle (degrees).')
    p.add_option('--ft', action='store', dest='ft',type='float',
                 default=80.,help='force threshold (Newtons). [default 80.]')
    p.add_option('--info', action='store', type='string', dest='info_string',
                 help='string to save in the pkl log.', default='')
    p.add_option('--pull_fixed', action='store_true', dest='pull_fixed',
                 help='pull with the segway stationary')
    p.add_option('--lead', action='store_true', dest='lead',
                 help='move the segway while pulling')
    p.add_option('--lpi', action='store_true', dest='lpi',
                 help='use the laser pointer interface to designate hooking location')
    p.add_option('-p', action='store', dest='p',type='int',
                 default=2,help='position number')
    p.add_option('-z', action='store', dest='z',type='float',
                 default=1.0,help='zenither height')
    p.add_option('--sa', action='store', dest='sa',type='float',
                 default=0.0,help='servo angle at which to take camera image (DEGREES)')
    p.add_option('--sliding_left', action='store_true',
                 dest='sliding_left',
                 help='defining the initial motion of the hook.')

    p.add_option('--use_jacobian', action='store_true',
                 dest='use_jacobian',
                 help='assume that kinematics estimation gives a jacobian')

    opt, args = p.parse_args()
    ha = opt.ha
    z = opt.z
    sa = opt.sa
    ft = opt.ft
    info_string = opt.info_string
    lead_flag = opt.lead
    lpi_flag = opt.lpi
    pull_fixed_flag = opt.pull_fixed
    move_segway_flag  = not pull_fixed_flag
    pnum = opt.p

    arm = 'right_arm'

    try:
        if ha == None:
            print 'please specify hook angle (--ha)'
            print 'Exiting...'
            sys.exit()

        cmg = ct.CompliantMotionGenerator(move_segway = move_segway_flag,
                                          use_right_arm = True,
                                          use_left_arm = False)

        if lead_flag:
            sys.path.append(os.environ['HRLBASEPATH']+'/src/projects/lead')
            import lead

            print 'hit a key to start leading.'
            k=m3t.get_keystroke()
            cmg.firenze.power_on()

            if move_segway_flag:
                mec = cmg.segway_command_node
            else:
                import segway_omni.segway as segway
                mec = segway.Mecanum()

            zed = cmg.z
            import mekabot.hrl_robot as hr
            settings_lead = hr.MekaArmSettings(stiffness_list=[0.2,0.3,0.3,0.5,0.8])
            cmg.firenze.set_arm_settings(settings_lead,None)

            follower = lead.Lead(cmg.firenze,'right_arm',mec,zed,
                                 max_allowable_height=zed.calib['max_height'],
                                 init_height = zed.get_position_meters())
            qr = [0, 0, 0, math.pi/2, -(math.radians(ha)-ct.hook_3dprint_angle), 0, 0]
            follower.start_lead_thread(qr=qr)

            print 'hit a key to start hook and pull.'
            k=m3t.get_keystroke()
            follower.stop()

            cmg.firenze.set_arm_settings(cmg.settings_stiff,None)
            cmg.firenze.step()
            cmg.firenze.pose_robot('right_arm')

            print 'Hit ENTER to reposition the robot'
            k=m3t.get_keystroke()
            if k!='\r':
                print 'You did not press ENTER.'
                print 'Exiting ...'
                sys.exit()

            hook_location = cmg.firenze.end_effector_pos('right_arm')
            pull_loc = cmg.reposition_robot(hook_location)

        elif lpi_flag:
            import lpi
#            import point_cloud_features.pointcloud_features as ppf
            import hrl_tilting_hokuyo.processing_3d as p3d
#            pc_feat = ppf.PointcloudFeatureExtractor(ros_init_node=False)

            cmg.z.torque_move_position(1.0)
            cmg.firenze.power_on()
            cmg.movement_posture()
            if z<0.5:
                print 'desired zenither height of %.2f is unsafe.'%(z)
                print 'Exiting...'
                sys.exit()

            hook_location = None
            #z_list = [1.0,0.5]
            z_list = [opt.z]
            i = 0
            while hook_location == None:
                if i == len(z_list):
                    print 'Did not get a click. Exiting...'
                    sys.exit()
                z = z_list[i]
                cmg.z.torque_move_position(z)
                hook_location = lpi.select_location(cmg.cam,cmg.thok,math.radians(sa))
                i += 1

            hl_thok0 = mcf.thok0Tglobal(hook_location)
            hl_torso = mcf.torsoTglobal(hook_location)

            t_begin = time.time()
            angle = 0.
            pull_loc,residual_angle = cmg.reposition_robot(hl_torso,angle,math.radians(ha),
                                                           position_number=pnum)
            print 'pull_loc:',pull_loc.A1.tolist()

            starting_location = copy.copy(hl_torso)
            starting_angle = -angle
            pose_dict = {}
            pose_dict['loc'] = starting_location
            pose_dict['angle'] = angle
            pose_dict['residual_angle'] = residual_angle
            pose_dict['position_number'] = pnum

            if opt.sliding_left:
                thresh = 2.
            else:
                thresh = 5.
            res, jep = cmg.search_and_hook(arm, math.radians(ha),
                                    pull_loc, residual_angle, thresh)
            print 'result of search and hook:', res
            if res != 'got a hook':
                print 'Did not get a hook.'
                print 'Exiting...'
                sys.exit()

        elif pull_fixed_flag:
            print 'hit a key to power up the arms.'
            k=m3t.get_keystroke()
            cmg.firenze.power_on()

            t_begin = time.time()
            pull_loc = np.matrix([0.55, -0.2, -.23]).T
            if opt.sliding_left:
                thresh = 2.
            else:
                thresh = 5.
            res, jep = cmg.search_and_hook(arm, math.radians(ha),
                                           pull_loc, 0., thresh)
            print 'result of search and hook:', res
            if res != 'got a hook':
                print 'Did not get a hook.'
                print 'Exiting...'
                sys.exit()
            residual_angle = 0.
            pose_dict = {}

        else:
            raise RuntimeError('Unsupported. Advait Jan 02, 2010.')

        if opt.use_jacobian:
            kinematics_estimation = 'jacobian'
        else:
            kinematics_estimation = 'rotation_center'

        t_pull = time.time()
        cmg.pull(arm, math.radians(ha), residual_angle, ft, jep,
                 strategy = 'control_radial_force',
                 info_string=info_string, cep_vel = 0.10,
                 kinematics_estimation=kinematics_estimation,
                 pull_left = opt.sliding_left)
        t_end = time.time()

        pose_dict['t0'] = t_begin
        pose_dict['t1'] = t_pull
        pose_dict['t2'] = t_end
        ut.save_pickle(pose_dict,'pose_dict_'+ut.formatted_time()+'.pkl')

        print 'hit  a key to end everything'
        k=m3t.get_keystroke()
        cmg.stop()

    except: # catch all exceptions, stop and re-raise them
        print 'Hello, Mr. Exception'
        cmg.stop()
        raise


