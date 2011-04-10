
import m3.rt_proxy as m3p
import m3.component_factory as m3f
import m3.toolbox as m3t
import m3.arm

THETA_GC = 5

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

joint_names = ['m3joint_ma1_j0']

c = joint_names[0]
comp = m3f.create_component(c)

pwr_nm = 'm3pwr_pwr003'
pwr = m3f.create_component(pwr_nm)

proxy.publish_command(pwr)
proxy.publish_command(comp)

# safeop_things must be after make_operational_all.
proxy.make_operational_all()
safeop_things(proxy)


ma1 = m3.arm.M3Arm('m3arm_ma1')
proxy.subscribe_status(ma1)


comp.set_control_mode(THETA_GC)
comp.set_stiffness(0.5)
comp.set_theta_deg(0.)
#comp.set_thetadot_deg(0.)



proxy.step()
proxy.step()

raw_input('Hit ENTER to power on')
pwr.set_motor_power_on()

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




