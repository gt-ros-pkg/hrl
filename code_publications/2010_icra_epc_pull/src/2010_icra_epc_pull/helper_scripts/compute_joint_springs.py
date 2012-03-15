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
import time
import math, numpy as np
import hrl_lib.util as ut, hrl_lib.transforms as tr

def record_initial(firenze):
    equilibrium_pos_list = []
    for i in range(50):
        equilibrium_pos_list.append(firenze.end_effector_pos('right_arm'))

    eq_pos = np.column_stack(equilibrium_pos_list).mean(1)
    ut.save_pickle(eq_pos,'eq_pos_'+ut.formatted_time()+'.pkl')
    firenze.bias_wrist_ft('right_arm')



def record_joint_displacements():
    print 'hit ENTER to start the recording process'
    k=m3t.get_keystroke()

    pos_list = []
    force_list = []

    while k == '\r':
        print 'hit ENTER to record configuration, something else to exit'
        k=m3t.get_keystroke()
        firenze.proxy.step()
        pos_list.append(firenze.end_effector_pos('right_arm'))
        force_list.append(firenze.get_wrist_force('right_arm', base_frame=True))

    ut.save_pickle({'pos_list':pos_list,'force_list':force_list},'stiffness_'+ut.formatted_time()+'.pkl')
    firenze.stop()



if __name__ == '__main__':

    settings_r = hr.MekaArmSettings(stiffness_list=[0.1939,0.6713,0.997,0.7272,0.75])
    firenze = hr.M3HrlRobot(connect = True, right_arm_settings = settings_r)

    print 'hit a key to power up the arms.'
    k = m3t.get_keystroke()
    firenze.power_on()

    print 'hit a key to test IK'
    k = m3t.get_keystroke()

    rot = tr.Ry(math.radians(-90))
    p = np.matrix([0.3,-0.40,-0.2]).T

    firenze.motors_on()
    #firenze.go_cartesian('right_arm', p, rot)

    # jep from springloaded door, trial 15
    jep = [-0.30365041761032346, 0.3490658503988659, 0.59866827092412689, 1.7924513637028943, 0.4580617747379146, -0.13602429148726047, -0.48610218950666179] 
    firenze.go_jointangles('right_arm', jep)


    print 'hit  a key to record equilibrium position'
    k = m3t.get_keystroke()
    record_initial(firenze)

    record_joint_displacements()

    print 'hit  a key to end everything'
    k = m3t.get_keystroke()
    firenze.stop()


