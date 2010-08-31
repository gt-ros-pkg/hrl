import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import kinematics_msgs.srv as ks
import hrl_lib.tf_utils as tfu
import tf
import hai_sandbox.pr2_kinematics as pr2k
import sensor_msgs.msg as sm
import hrl_lib.rutils as ru
import numpy as np
import hrl_lib.util as ut

def script():
    rospy.init_node('forward_kin')
    tflistener = tf.TransformListener()
    print 'waiting for transform'
    tflistener.waitForTransform('r_gripper_tool_frame', 'r_wrist_roll_link', rospy.Time(), rospy.Duration(10))
    print 'waiting for services'
    rospy.wait_for_service('pr2_right_arm_kinematics/get_fk_solver_info')#, ks.GetKinematicSolverInfo )
    rospy.wait_for_service('pr2_right_arm_kinematics/get_fk')#,             ks.GetPositionFK)
    print 'done init'
    
    r_fk_info = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk_solver_info', ks.GetKinematicSolverInfo )
    r_fk      = rospy.ServiceProxy('pr2_right_arm_kinematics/get_fk',             ks.GetPositionFK)
    
    resp = r_fk_info()
    print 'get_fk_solver_info returned', resp.kinematic_solver_info.joint_names
    print 'get_fk_solver_info returned', resp.kinematic_solver_info.limits
    print 'get_fk_solver_info returned', resp.kinematic_solver_info.link_names
    
    fk_req = ks.GetPositionFKRequest()
    fk_req.header.frame_id = 'torso_lift_link'
    fk_req.fk_link_names = ['r_wrist_roll_link']
    fk_req.robot_state.joint_state.name = resp.kinematic_solver_info.joint_names
    fk_req.robot_state.joint_state.position = [.5 for i in range(len(resp.kinematic_solver_info.joint_names))]
    fk_resp = r_fk(fk_req)
    
    rtip_T_wr = tfu.transform('r_gripper_tool_frame', 'r_wrist_roll_link', tflistener)
    right_wr = tfu.pose_as_matrix(fk_resp.pose_stamped[0].pose)
    rtip_pose = rtip_T_wr * right_wr
    print tfu.matrix_as_tf(rtip_pose)

class TestForwardKin:
    def __init__(self):
        self.name_dict = None
        self.msgs = []
        self.joint_idx = {}

        rospy.init_node('forward_kin')
        self.tflistener = tf.TransformListener()
        self.fkright = pr2k.PR2ArmKinematics('right', self.tflistener)
        self.point_cloud_pub = rospy.Publisher('right_fk', sm.PointCloud)

        rospy.Subscriber('/joint_states', sm.JointState, self.joint_state_cb)
        print 'done init'


    def joint_state_cb(self, msg):
        if self.name_dict == None:
            self.name_dict = {}
            for i, n in enumerate(msg.name):
                self.name_dict[n] = i 
            self.joint_groups = ut.load_pickle('link_names.pkl')
            for group in self.joint_groups.keys():
                self.joint_idx[group] = [self.name_dict[name] for name in self.joint_groups[group]]

        dmat = np.matrix(msg.position).T
        joint_angles = dmat[self.joint_idx['rarm'], 0].A1.tolist()
        #print len(joint_angles)
        #print dmat.shape, self.joint_idx['rarm']
        rtip_pose = self.fkright.fk('base_footprint', 'r_wrist_roll_link', 'r_gripper_tool_frame', joint_angles)
        position  = rtip_pose[0:3,3]
        #print position.T

        pc = ru.np_to_pointcloud(position, 'base_footprint')
        pc.header.stamp = rospy.get_rostime()
        self.point_cloud_pub.publish(pc)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


def kin_class():
    tflistener = tf.TransformListener()
    right = pr2k.PR2ArmKinematics('right', tflistener)
    rtip_pose = right.fk('torso_lift_link', 'r_wrist_roll_link', 'r_gripper_tool_frame',  [.5 for i in range(len(right.joint_names))])
    print tfu.matrix_as_tf(rtip_pose)

    left = pr2k.PR2ArmKinematics('left', tflistener)
    ltip_pose = left.fk('torso_lift_link', 'l_wrist_roll_link', 'l_gripper_tool_frame',  [.5 for i in range(len(left.joint_names))])
    print tfu.matrix_as_tf(ltip_pose)

if __name__ == '__main__':
    #kin_class()
    t = TestForwardKin()
    t.run()





















