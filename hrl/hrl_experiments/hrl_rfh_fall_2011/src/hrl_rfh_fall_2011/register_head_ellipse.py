#! /usr/bin/python

import sys
import numpy as np
import scipy.io

import roslib
roslib.load_manifest('hrl_phri_2011')
roslib.load_manifest('hrl_generic_arms')
roslib.load_manifest('smach_ros')
import rospy

import smach
from smach_ros import SimpleActionState, ServiceState, IntrospectionServer
from hrl_phri_2011.msg import EllipsoidParams
from geometry_msgs.msg import PoseStamped
from hrl_generic_arms.pose_converter import PoseConverter

class ClickMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['click', 'shutdown'],
                                   output_keys=['click_pose'])
        self.cur_msg = None
        rospy.Subscriber('/pixel3d', PoseStamped, self.click_cb)

    def click_cb(self, msg):
        self.cur_msg = msg

    def execute(self, userdata):
        self.cur_msg = None
        while not rospy.is_shutdown():
            if self.cur_msg is not None:
                userdata.click_pose = self.cur_msg
                return 'click'
            rospy.sleep(0.01)
        return 'shutdown'

class SMEllipsoidRegistration(object):
    def __init__(self):
        self.ep_pub = rospy.Publisher("/ell_params_cmd", EllipsoidParams)

    def get_pub_head_registration(self):
        
        @smach.cb_interface(input_keys=['cheek_pose'],
                            output_keys=[],
                            outcomes=['succeeded'])
        def pub_head_registration(ud):
            # find the center of the ellipse given a cheek click
            # TODO find this tranformation
            cheek_transformation = np.mat([[1,      0,      0,      0],
                                           [0,      1,      0,      0],
                                           [0,      0,      1,      0],
                                           [0,      0,      0,      1]])
            b_B_c = PoseConverter.to_homo_mat(ud.cheek_pose)
            b_B_c[0:3,0:3] = np.eye(3)
            ell_center = b_B_c * cheek_transformation

            # create an ellipsoid msg and command it 
            ep = EllipsoidParams()
            ep.e_frame.transform = PoseConverter.to_tf_msg(ell_center)
            ep.height = 0.924
            ep.E = 0.086
            self.ep_pub.publish(ep)

            return 'succeeded'

        return smach.CBState(pub_head_registration)

    def get_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded','preempted','shutdown'])
        
        with sm:
            smach.StateMachine.add(
                'INPUT_CHEEK_CLICK',
                ClickMonitor(),
                transitions={'click' : 'REGISTER_HEAD',
                             'shutdown' : 'shutdown'},
                remapping={'click_pose' : 'cheek_pose'}) # output (PoseStamped)

            smach.StateMachine.add(
                'REGISTER_HEAD',
                self.get_pub_head_registration(),
                transitions={'succeeded' : 'INPUT_CHEEK_CLICK'},
                remapping={'cheek_pose' : 'cheek_pose'})

        return sm

def main():
    rospy.init_node('register_head_ellipse')
    smer = SMEllipsoidRegistration()
    sm = smer.get_sm()
    rospy.sleep(1)

    sis = IntrospectionServer('register_head', sm, '/INPUT_CHEEK_CLICK')
    sis.start()

    outcome = sm.execute()
    
    sis.stop()

if __name__ == '__main__':
    main()
