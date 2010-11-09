#! /usr/bin/python
import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import planning_environment_msgs.srv as psrv
import sensor_msgs.msg as sm
import numpy as np
import time
import hai_sandbox.msg as hmsg

class CollisionMonitor:
    def __init__(self, arm):
        rospy.init_node('collision_monitor_' + arm)
        self.name_dict = None
        link_names = ['_shoulder_pan_joint',   '_shoulder_lift_joint',
                      '_upper_arm_roll_joint', '_elbow_flex_joint',
                      '_forearm_roll_joint',   '_wrist_flex_joint', '_wrist_roll_joint']
        self.arm_name = [arm + l for l in link_names]

        if arm == 'l':
            service_name = 'environment_server_left_arm/get_state_validity'
        else:
            service_name = 'environment_server_right_arm/get_state_validity'


        rospy.loginfo('waiting for ' + service_name)
        rospy.wait_for_service(service_name)
        self.check_state_validity_client = rospy.ServiceProxy(service_name, \
                psrv.GetStateValidity, persistent=True)
        rospy.Subscriber('joint_states', sm.JointState, self.joint_state_cb, \
                queue_size=5, tcp_nodelay=True)
        self.contact_pub = rospy.Publisher('contacts_' + arm, hmsg.OnlineContact)

    def joint_state_cb(self, msg):
        if self.name_dict == None:
            self.name_dict = {}
            for i, n in enumerate(msg.name):
                self.name_dict[n] = i  

        arm_indices = [self.name_dict[n] for n in self.arm_name]
        arm_list = np.array(msg.position)[arm_indices].tolist()

        req = psrv.GetStateValidityRequest()    
        req.robot_state.joint_state.name = self.arm_name
        req.robot_state.joint_state.position     = arm_list
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.check_collisions = True
        res = self.check_state_validity_client(req)
        if not (res.error_code.val == res.error_code.SUCCESS):
            #contact_with_points = False
            #for c in res.contacts:
            #    if c.contact_body_1 == 'points' or c.contact_body_2 == 'points':
            #        contact_with_points = True
            #    else:
            #        print 'contact between', c.contact_body_1, c.contact_body_2

            m = hmsg.OnlineContact()
            m.contacts = res.contacts
            self.contact_pub.publish(m)
            #if not contact_with_points:
            #    rospy.loginfo('state is in COLLISION')
        
            
def call_collision_monitor(arm):
    a = CollisionMonitor(arm)
    rospy.loginfo('ready')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()

def call_get_state_validity():
    rospy.init_node('test_get_state_validity')
    service_name = 'environment_server_left_arm/get_state_validity'
    rospy.loginfo('waiting for ' + service_name)
    rospy.wait_for_service(service_name)
    check_state_validity_client = rospy.ServiceProxy('environment_server_left_arm/get_state_validity', \
            psrv.GetStateValidity, persistent=True)
    req = psrv.GetStateValidityRequest()    
    req.robot_state.joint_state.name = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint',
                                        'l_upper_arm_roll_joint', 'l_elbow_flex_joint',
                                        'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
    req.robot_state.joint_state.position     = 7 * [0.0]

    #while not rospy.is_shutdown():
    t = time.time()
    for i in range(1000):
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.check_collisions = True
        res = check_state_validity_client(req)
    diff = time.time() - t
    time_per_call = diff / 1000
    print time_per_call, 'rate', 1 / time_per_call
        #if res.error_code.val == res.error_code.SUCCESS:
        #    rospy.loginfo('state is NOT in collision')
        #else:
        #    rospy.loginfo('state is in collision')


if __name__ == '__main__':
    import sys
    call_collision_monitor(sys.argv[1])
    #call_get_state_validity()

    





























































