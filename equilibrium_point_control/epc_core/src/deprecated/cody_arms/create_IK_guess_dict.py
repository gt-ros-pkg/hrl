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

import arms as ar
import math, numpy as np
import sys, optparse

import roslib; roslib.load_manifest('epc_core')
import hrl_lib.transforms as tr
import hrl_lib.util as ut

def find_good_config(pt,dict):
    ''' finds a good configuration for the 3x1 pt.
    '''
    m = dict['cart_pts_mat']
    min_idx = np.argmin(ut.norm(m-pt))
    c = dict['good_configs_list'][min_idx]
#    print 'good configuration:', [math.degrees(qi) for qi in c]
    return c

def test_dict(fname):
    dict = ut.load_pickle(fname)
    firenze = ar.M3HrlRobot(connect=False)

    rot = tr.rotY(math.radians(-90))
    p = np.matrix([0.4,-0.42,-0.2]).T

    c = find_good_config(p,dict)
    res = firenze.IK(p,rot,q_guess=c)
    print 'IK soln: ', [math.degrees(qi) for qi in res]

def create_dict(fname):
    firenze = ar.M3HrlRobot(connect=False)
    good_configs_list = ut.load_pickle(fname)
    cartesian_points_list = []
    for gc in good_configs_list:
        cartesian_points_list.append(firenze.FK('right_arm',gc).A1.tolist())

    m = np.matrix(cartesian_points_list).T
    print 'm.shape:', m.shape
    dict = {'cart_pts_mat':m, 'good_configs_list':good_configs_list}
    ut.save_pickle(dict,ut.formatted_time()+'_goodconf_dict.pkl')


def record_good_configs(use_left_arm):
    import m3.toolbox as m3t
    settings_arm = ar.MekaArmSettings(stiffness_list=[0.,0.,0.,0.,0.],control_mode='torque_gc')

    if use_left_arm:
        firenze = ar.M3HrlRobot(connect=True,left_arm_settings=settings_arm)
        arm = 'left_arm'
    else:
        firenze = ar.M3HrlRobot(connect=True,right_arm_settings=settings_arm)
        arm = 'right_arm'

    print 'hit ENTER to start the recording process'
    k=m3t.get_keystroke()
    firenze.power_on()

    good_configs_list = []

    while k == '\r':
        print 'hit ENTER to record configuration, something else to exit'
        k=m3t.get_keystroke()
        firenze.proxy.step()
        q = firenze.get_joint_angles(arm)
        good_configs_list.append(np.matrix(q).A1.tolist())

    firenze.stop()
    ut.save_pickle(good_configs_list,ut.formatted_time()+'_good_configs_list.pkl')



if __name__=='__main__':
    p = optparse.OptionParser()
    p.add_option('-r', action='store_true', dest='record',
                 help='put robot in GC and record good configurations.')
    p.add_option('-c', action='store_true', dest='create',
                 help='create table to map points to good configs. (needs a good_configs pkl)')
    p.add_option('-t', action='store_true', dest='test',
                 help='find a good config for a cartesian point. (needs a dict pkl)')
    p.add_option('-f', action='store', type='string', dest='fname',
                 help='pkl file to use.', default='')
    p.add_option('--ra', action='store_true', dest='right',
                 help='choose the right arm')
    p.add_option('--la', action='store_true', dest='left',
                 help='choose the left arm')

    opt, args = p.parse_args()
    record = opt.record
    create = opt.create
    test = opt.test
    fname = opt.fname
    use_left_arm = opt.left
    use_right_arm = opt.right
    
    if test:
        if fname == '':
            print 'Specify a file name.'
            sys.exit()
        test_dict(fname)
    elif create:
        if fname == '':
            print 'Specify a file name.'
            sys.exit()
        create_dict(fname)
    elif record:
        if use_right_arm == None and use_left_arm == None:
            print 'Please specify either left or right arm. (--ra or --la)'
            print 'Exiting...'
            sys.exit()

        record_good_configs(use_left_arm)
    else:
        print 'Please specify either record or create.'


