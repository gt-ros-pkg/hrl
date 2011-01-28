import PyKDL as kdl

import numpy as np, math

import roslib; roslib.load_manifest('pr2_arms_kdl')
import rospy
import hrl_lib.kdl_utils as ku


class PR2_arm_kdl():

    def __init__(self):
        self.right_chain = self.create_right_chain()
        fk, ik_v, ik_p = self.create_solvers(self.right_chain)
        self.right_fk = fk
        self.right_ik_v = ik_v
        self.right_ik_p = ik_p

    def create_right_chain(self):
        ch = kdl.Chain()
        self.right_arm_base_offset_from_torso_lift_link = np.matrix([0., -0.188, 0.]).T
        # shoulder pan
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotZ),kdl.Frame(kdl.Vector(0.1,0.,0.))))
        # shoulder lift
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        # upper arm roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.4,0.,0.))))
        # elbox flex
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.0,0.,0.))))
        # forearm roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.321,0.,0.))))
        # wrist flex
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0.,0.,0.))))
        # wrist roll
        ch.addSegment(kdl.Segment(kdl.Joint(kdl.Joint.RotX),kdl.Frame(kdl.Vector(0.,0.,0.))))
        return ch

    def create_solvers(self, ch):
         fk = kdl.ChainFkSolverPos_recursive(ch)
         ik_v = kdl.ChainIkSolverVel_pinv(ch)
         ik_p = kdl.ChainIkSolverPos_NR(ch, fk, ik_v)
         return fk, ik_v, ik_p

    def FK_kdl(self, arm, q, link_number):
        if arm == 'right_arm':
            fk = self.right_fk
            endeffec_frame = kdl.Frame()
            kinematics_status = fk.JntToCart(q, endeffec_frame,
                                             link_number)
            if kinematics_status >= 0:
                return endeffec_frame
            else:
                rospy.loginfo('Could not compute forward kinematics.')
                return None
        else:
            msg = '%s arm not supported.'%(arm)
            rospy.logerr(msg)
            raise RuntimeError(msg)

    ## returns point in torso lift link.
    def FK_all(self, arm, q, link_number = 7):
        q = self.pr2_to_kdl(q)
        frame = self.FK_kdl(arm, q, link_number)
        pos = frame.p
        pos = ku.kdl_vec_to_np(pos)
        pos = pos + self.right_arm_base_offset_from_torso_lift_link
        m = frame.M
        rot = ku.kdl_rot_to_np(m)
        return pos, rot

    def kdl_to_pr2(self, q):
        if q == None:
            return None

        q_pr2 = [0] * 7
        q_pr2[0] = q[0]
        q_pr2[1] = q[1]
        q_pr2[2] = q[2]
        q_pr2[3] = q[3]
        q_pr2[4] = q[4]
        q_pr2[5] = q[5]
        q_pr2[6] = q[6]
        return q_pr2

    def pr2_to_kdl(self, q):
        if q == None:
            return None
        n = len(q)
        q_kdl = kdl.JntArray(n)
        for i in range(n):
            q_kdl[i] = q[i]
        return q_kdl


if __name__ == '__main__':
    roslib.load_manifest('hrl_pr2_lib')
    import hrl_pr2_lib.pr2_arms as pa
    import hrl_lib.viz as hv
    from visualization_msgs.msg import Marker
    
    rospy.init_node('kdl_pr2_test')
    marker_pub = rospy.Publisher('/kdl_pr2_arms/viz_marker', Marker)
    pr2_arms = pa.PR2Arms(gripper_point=(0.,0.,0.))
    pr2_kdl = PR2_arm_kdl()

    r_arm, l_arm = 0, 1

    while not rospy.is_shutdown():
        q = pr2_arms.get_joint_angles(r_arm)
        p, r = pr2_kdl.FK_all('right_arm', q, 7)
        m = hv.create_frame_marker(p, r, 0.3, 'torso_lift_link')
        time_stamp = rospy.Time.now()
        m.header.stamp = time_stamp
        marker_pub.publish(m)
        rospy.sleep(0.1)



