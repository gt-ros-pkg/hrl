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


import math, numpy as np
import m3.rt_proxy as m3p
import m3.component_factory as m3f
import m3.toolbox as m3t
import m3.arm

THETA_GC = 5
THETA = 3

def safeop_things(proxy):
    robot_name = 'm3humanoid_bimanual_mr1'
    chain_names = ['m3arm_ma1', 'm3arm_ma2']
    dynamatics_nms = ['m3dynamatics_ma1', 'm3dynamatics_ma2']

    proxy.make_safe_operational(robot_name)
    for c in chain_names:
        proxy.make_safe_operational(c)
    for d in dynamatics_nms:
        proxy.make_safe_operational(d)


proxy = m3p.M3RtProxy()
proxy.start()

pwr_nm = 'm3pwr_pwr003'
pwr = m3f.create_component(pwr_nm)
proxy.publish_command(pwr)

joint_names = ['m3joint_ma1_j0', 'm3joint_ma1_j1', 'm3joint_ma1_j2', 'm3joint_ma1_j3', 'm3joint_ma1_j4', 'm3joint_ma1_j5', 'm3joint_ma1_j6']

comp_list = []
stiff_list = [0.2, 0.67, 1., 0.7, 0.75, 0.5, 0.5]
for i, c in enumerate(joint_names):
    comp = m3f.create_component(c)
    comp_list.append(comp)
    proxy.publish_command(comp)
    if i < 5:
        comp.set_control_mode(THETA_GC)
    else:
        comp.set_control_mode(THETA)
    comp.set_stiffness(stiff_list[i])
    comp.set_slew_rate_proportion(1.)

# safeop_things must be after make_operational_all.
proxy.make_operational_all()
safeop_things(proxy)

#ma1 = m3.arm.M3Arm('m3arm_ma1')
#proxy.subscribe_status(ma1)

proxy.step()
proxy.step()

raw_input('Hit ENTER to power on')
pwr.set_motor_power_on()

proxy.step()
proxy.step()

raw_input('Hit ENTER to move the joint')
q = [0., 0., 0., 90., 0., 0., 0.]
q = np.radians(q)

for i, c in enumerate(comp_list):
    c.set_theta_rad(q[i])

proxy.step()
proxy.step()

raw_input('Hit ENTER to stop')
proxy.stop()









#-------------- older code ---------------

##Force safe-op of robot, etc are present
#types=['m3humanoid','m3hand','m3gripper']
#for t in types:
#    cc = proxy.get_available_components(t)
#    for ccc in cc:
#        print 'ccc:', ccc
#        proxy.make_safe_operational(ccc)
#
#
##Force safe-op of chain so that gravity terms are computed
#chain=None
#if len(joint_names)>0:
#    for j in joint_names:
#        chain_name=m3t.get_joint_chain_name(j)
#        print 'chain_name:', chain_name
#        if chain_name!="":
#            proxy.make_safe_operational(chain_name)
#
#            #Force safe-op of chain so that gravity terms are computed
#            dynamatics_name = m3t.get_chain_dynamatics_component_name(chain_name)
#            print 'dynamatics_name:', dynamatics_name
#            if dynamatics_name != "":
#                proxy.make_safe_operational(dynamatics_name)
#
#
#
##Force safe-op of robot so that gravity terms are computed
#robot_name = m3t.get_robot_name()
#print 'robot_name:', robot_name
#if robot_name != "":
#    proxy.make_safe_operational(robot_name)




