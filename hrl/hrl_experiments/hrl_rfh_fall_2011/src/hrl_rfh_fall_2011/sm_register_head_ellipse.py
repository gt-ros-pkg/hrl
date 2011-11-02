#! /usr/bin/python

import sys
import numpy as np
import scipy.io

import roslib
roslib.load_manifest('hrl_phri_2011')
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('pixel_2_3d')
roslib.load_manifest('hrl_rfh_fall_2011')
roslib.load_manifest('smach_ros')
import rospy
import tf

import smach
import tf.transformations as tf_trans
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from hrl_phri_2011.msg import EllipsoidParams
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from hrl_generic_arms.pose_converter import PoseConverter

#from pixel_2_3d.sm_click_monitor import ClickMonitor
from hrl_rfh_fall_2011.srv import PCSwitch, PCSwitchRequest
from hrl_rfh_fall_2011.sm_topic_monitor import TopicMonitor

class SMEllipsoidRegistration(object):
    def __init__(self):
        self.ep_pub = rospy.Publisher("/ell_params_cmd", EllipsoidParams)
        self.cheek_pub = rospy.Publisher("/cheek_pose", PoseStamped)
        self.ell_center_pub = rospy.Publisher("/ell_pose", PoseStamped)
        self.tf_list = tf.TransformListener()

    def get_pub_head_registration(self):
        
        @smach.cb_interface(input_keys=['cheek_pose'],
                            output_keys=[],
                            outcomes=['succeeded'])
        def pub_head_registration(ud):
            cheek_pose_base_link = self.tf_list.transformPose("/base_link", ud.cheek_pose)
            # find the center of the ellipse given a cheek click
            cheek_transformation = np.mat(tf_trans.euler_matrix(2.6 * np.pi/6, 0, 0, 'szyx'))
            cheek_transformation[0:3, 3] = np.mat([-0.08, -0.04, 0]).T
            cheek_pose = PoseConverter.to_homo_mat(cheek_pose_base_link)
            #b_B_c[0:3,0:3] = np.eye(3)
            norm_xy = cheek_pose[0:2, 2] / np.linalg.norm(cheek_pose[0:2, 2])
            head_rot = np.arctan2(norm_xy[1], norm_xy[0])
            cheek_pose[0:3,0:3] = tf_trans.euler_matrix(0, 0, head_rot, 'sxyz')[0:3,0:3]
            self.cheek_pub.publish(PoseConverter.to_pose_stamped_msg("/base_link", cheek_pose))
            ell_center = cheek_pose * cheek_transformation
            self.ell_center_pub.publish(PoseConverter.to_pose_stamped_msg("/base_link", ell_center))

            # create an ellipsoid msg and command it 
            ep = EllipsoidParams()
            ep.e_frame.transform = PoseConverter.to_tf_msg(ell_center)
            ep.height = 0.924
            ep.E = 0.086
            self.ep_pub.publish(ep)

            return 'succeeded'

        return smach.CBState(pub_head_registration)

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
        
        with sm:

            # turn the switch on the pc topic on 
            def pc_swtich_resp_cb(us, resp):
                return 'succeeded'
            smach.StateMachine.add('UNFREEZE_PC',
                                   ServiceState('pc_switch', PCSwitch, 
                                                request=PCSwitchRequest(True),
                                                response_cb=pc_swtich_resp_cb),
                                   transitions={'succeeded' : 'HEAD_REGISTRATION',
                                                'aborted' : 'aborted'})

            # create clicking registration loop
            def child_term_cb(outcome_map):
                print 'outcome_map', outcome_map
                if outcome_map['REGISTRATION_CONFIRM'] == 'succeeded':
                    return True
                return False
            sm_reg = smach.Concurrence(outcomes=['succeeded', 'aborted'],
                                       default_outcome='succeeded',
                                       child_termination_cb=child_term_cb)

            with sm_reg:
                
                # wait for this topic to be called to exit the loop
                def reg_confirm_cb(ud, req):
                    return 'succeeded'
                smach.Concurrence.add('REGISTRATION_CONFIRM', 
                        TopicMonitor('reg_confirm', Bool))

                sm_reg_loop = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
                with sm_reg_loop:
                    # receive a message from pixel_2_3d
                    smach.StateMachine.add(
                        'INPUT_CHEEK_CLICK',
                        TopicMonitor('/init_head', PoseStamped),
                        transitions={'succeeded' : 'REGISTER_HEAD'},
                        remapping={'output' : 'cheek_pose'})

                    # update the ellipsoid frame location
                    smach.StateMachine.add(
                        'REGISTER_HEAD',
                        self.get_pub_head_registration(),
                        transitions={'succeeded' : 'INPUT_CHEEK_CLICK'},
                        remapping={'cheek_pose' : 'cheek_pose'})

                smach.Concurrence.add('HEAD_REGISTRATION_LOOP', sm_reg_loop)

            smach.StateMachine.add('HEAD_REGISTRATION', sm_reg,
                                   transitions={'succeeded' : 'FREEZE_PC'})

            # turn the switch on the pc topic off so that the user can register on a still PC
            def pc_swtich_resp_cb(us, resp):
                return 'succeeded'
            smach.StateMachine.add('FREEZE_PC',
                                   ServiceState('pc_switch', PCSwitch, 
                                                request=PCSwitchRequest(False),
                                                response_cb=pc_swtich_resp_cb),
                                   transitions={'aborted' : 'aborted'})

        return sm

def main():
    rospy.init_node('register_head_ellipse')
    smer = SMEllipsoidRegistration()
    sm = smer.get_sm()
    rospy.sleep(1)

    sis = IntrospectionServer('register_head', sm, '/UNFREEZE_PC')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

if __name__ == '__main__':
    main()
