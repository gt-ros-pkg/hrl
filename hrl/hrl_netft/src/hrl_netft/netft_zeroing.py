#! /usr/bin/python

import cPickle as pickle
import sys
import copy
import numpy as np

import roslib
roslib.load_manifest("tf")
roslib.load_manifest("rosparam")
roslib.load_manifest("visualization_msgs")
import rospy
import rosparam
import tf
import tf.transformations as tf_trans
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3, Point
from std_msgs.msg import Float64, Bool, ColorRGBA
from visualization_msgs.msg import Marker

g = 9.81

class NetFTListener:

    def __init__(self, netft_topic):
        self.cur_wrench = WrenchStamped()
        rospy.Subscriber(netft_topic, WrenchStamped, self.record_wrench)

    def record_wrench(self, msg):
        self.cur_wrench = [msg.wrench.force.x, 
                           msg.wrench.force.y, 
                           msg.wrench.force.z, 
                           msg.wrench.torque.x, 
                           msg.wrench.torque.y, 
                           msg.wrench.torque.z]

def collect_data_pr2(n_4, n_5, n_6):
    roslib.load_manifest("hrl_pr2_arms")
    from hrl_pr2_arms.pr2_arm import PR2ArmJointTrajectory, create_pr2_arm
    netft_topic = rospy.get_param("~netft_topic")
    gravity_frame = rospy.get_param("~gravity_frame")
    netft_frame = rospy.get_param("~netft_frame")
    jnt_arm = create_pr2_arm('l', arm_type=PR2ArmJointTrajectory)
    kin = jnt_arm.kinematics
    netft_list = NetFTListener(netft_topic)
    tf_list = tf.TransformListener()

    q_setup = [0.4, 0.0, 1.0, -1.2, 0, -0.1, -3.14]
    q_4_vals = np.linspace(0, 4.7, n_4)
    q_5_vals = np.linspace(-0.1, -1.8, n_5)
    q_6_vals = np.linspace(-3.14, -0.2, n_6)

    jnt_arm.set_ep(q_setup, 8)
    rospy.sleep(8.)

    data = []

    q_cur = q_setup
    for q_4 in q_4_vals:
        for q_5 in q_5_vals:
            for q_6 in q_6_vals:
                q_cur[4] = q_4
                q_cur[5] = q_5
                q_cur[6] = q_6
                jnt_arm.set_ep(q_cur, 2.)
                rospy.sleep(4.)
                (trans, rot) = tf_list.lookupTransform(gravity_frame, netft_frame, rospy.Time(0))
                data.append((copy.copy(netft_list.cur_wrench), rot))
                if rospy.is_shutdown():
                    return
    print "Wrench, orientation data", data
    return data

def collect_data_tool():
    netft_topic = rospy.get_param("~netft_topic")
    gravity_frame = rospy.get_param("~gravity_frame")
    netft_frame = rospy.get_param("~netft_frame")
    netft_list = NetFTListener(netft_topic)
    tf_list = tf.TransformListener()
    data = []
    while not rospy.is_shutdown():
        done_query = raw_input("Enter 'd' when done")
        if done_query == 'd':
            break
        (trans, rot) = tf_list.lookupTransform(gravity_frame, netft_frame, rospy.Time(0))
        data.append((copy.copy(netft_list.cur_wrench), rot))
    print "Wrench, orientation data", data
    return data

def process_data(data, is_pr2):
    wf_chain = []
    grav_chain = []
    if is_pr2:
        # account for the fact that the ft sensor is sensing forces
        # backwards and is measuring reaction forces as opposed to the appied forces
        react_mult = -1.
    else:
        react_mult = 1.
    for w, quat in data:
        wf_chain.extend(w[:3])
        rot_mat = np.mat(tf_trans.quaternion_matrix(quat))[:3,:3]
        z_grav = react_mult * rot_mat.T * np.mat([0, 0, -1.]).T
        z_x, z_y, z_z = z_grav.T.A[0]
        grav_chain.append([g * z_x, 1, 0, 0])
        grav_chain.append([g * z_y, 0, 1, 0])
        grav_chain.append([g * z_z, 0, 0, 1])

    b_w = np.mat(wf_chain).T
    A_g = np.mat(grav_chain)
    x_p, res_p, _, _ = np.linalg.lstsq(A_g, b_w)

    mass = x_p[0,0]

    wt_chain = []
    torque_chain = []
    for w, quat in data:
        wt_chain.extend(w[3:])
        rot_mat = np.mat(tf_trans.quaternion_matrix(quat))[:3,:3]
        z_grav = react_mult * rot_mat.T * np.mat([0, 0, -1.]).T
        force_grav = mass * g * z_grav
        f_x, f_y, f_z = force_grav.T.A[0]
        torque_chain.append([0, f_z, -f_y, 1, 0, 0])
        torque_chain.append([-f_z, 0, f_x, 0, 1, 0])
        torque_chain.append([f_y, -f_x, 0, 0, 0, 1])
    b_t = np.mat(wt_chain).T
    A_t = np.mat(torque_chain)
    x_t, res_t, _, _ = np.linalg.lstsq(A_t, b_t)

    print "Force residues:", res_p
    print "Torque residues:", res_t

    return x_p.T.A[0].tolist() + x_t.T.A[0].tolist()

def save_params(p, filename):
    ft_params = { "mass" : p[0],
                  "force_zero_x" : p[1],
                  "force_zero_y" : p[2],
                  "force_zero_z" : p[3],
                  "com_pos_x" : p[4],
                  "com_pos_y" : p[5],
                  "com_pos_z" : p[6],
                  "torque_zero_x" : p[7],
                  "torque_zero_y" : p[8],
                  "torque_zero_z" : p[9] }
    rosparam.upload_params(rospy.get_name(), ft_params)
    rospy.sleep(0.5)
    rosparam.dump_params(filename, rospy.get_name())

class NetFTZeroer:
    def __init__(self, start_zero=True, is_pr2=True):

        netft_topic = rospy.get_param("~netft_topic")
        self.gravity_frame = rospy.get_param("~gravity_frame")
        self.netft_frame = rospy.get_param("~netft_frame")
        self.tf_list = tf.TransformListener()
        rospy.Subscriber("~rezero_wrench", Bool, self.rezero_wrench)
        self.zero_pub = rospy.Publisher("~wrench_zeroed", WrenchStamped)
        self.vis_pub = rospy.Publisher("~wrench_markers", Marker)

        self.mass = rospy.get_param("~mass")
        self.wrench_zero = np.mat(np.zeros((6, 1)))
        self.wrench_zero[0, 0] = rospy.get_param("~force_zero_x")
        self.wrench_zero[1, 0] = rospy.get_param("~force_zero_y")
        self.wrench_zero[2, 0] = rospy.get_param("~force_zero_z")
        self.wrench_zero[3, 0] = rospy.get_param("~torque_zero_x")
        self.wrench_zero[4, 0] = rospy.get_param("~torque_zero_y")
        self.wrench_zero[5, 0] = rospy.get_param("~torque_zero_z")
        self.com_pos = np.mat(np.zeros((3, 1)))
        self.com_pos[0, 0] = rospy.get_param("~com_pos_x")
        self.com_pos[1, 0] = rospy.get_param("~com_pos_y")
        self.com_pos[2, 0] = rospy.get_param("~com_pos_z")

        if start_zero:
            self.got_zero = False
        else:
            self.got_zero = True

        if is_pr2:
            self.react_mult = -1.
        else:
            self.react_mult = 1.

        self.wrench_location_frame = rospy.get_param("~wrench_location_frame")
        self.wrench_base_frame = rospy.get_param("~wrench_base_frame")

        self.colors = [ColorRGBA(1., 0., 0., 1.), ColorRGBA(0., 1., 0., 1.)]

        rospy.sleep(0.1)
        rospy.Subscriber(netft_topic, WrenchStamped, self.process_wrench)

    def process_wrench(self, msg):
        cur_wrench = np.mat([msg.wrench.force.x, 
                             msg.wrench.force.y, 
                             msg.wrench.force.z, 
                             msg.wrench.torque.x, 
                             msg.wrench.torque.y, 
                             msg.wrench.torque.z]).T
        try:
            (ft_pos, ft_quat) = self.tf_list.lookupTransform(self.gravity_frame, 
                                                             self.netft_frame, rospy.Time(0))
        except:
            return
        rot_mat = np.mat(tf_trans.quaternion_matrix(ft_quat))[:3,:3]
        z_grav = self.react_mult * rot_mat.T * np.mat([0, 0, -1.]).T
        force_grav = np.mat(np.zeros((6, 1)))
        force_grav[:3, 0] = self.mass * g * z_grav
        torque_grav = np.mat(np.zeros((6, 1)))
        torque_grav[3:, 0] = np.mat(np.cross(self.com_pos.T.A[0], force_grav[:3, 0].T.A[0])).T
        zeroing_wrench = force_grav + torque_grav + self.wrench_zero
        zeroed_wrench = self.react_mult * (cur_wrench - zeroing_wrench)
        
        if not self.got_zero:
            self.wrench_zero = (cur_wrench - (force_grav + torque_grav))
            self.got_zero = True

        tf_zeroed_wrench = self.transform_wrench(zeroed_wrench)
        if tf_zeroed_wrench is None:
            return
        zero_msg = WrenchStamped(msg.header, 
                                 Wrench(Vector3(*tf_zeroed_wrench[:3,0]), Vector3(*tf_zeroed_wrench[3:,0])))
        self.zero_pub.publish(zero_msg)
        self.visualize_wrench(tf_zeroed_wrench)
        

    def transform_wrench(self, wrench):
        try:
            ft_pos, ft_quat = self.tf_list.lookupTransform(self.gravity_frame, 
                                                           self.netft_frame, rospy.Time(0))
            loc_pos, loc_quat = self.tf_list.lookupTransform(self.gravity_frame, 
                                                             self.wrench_location_frame, rospy.Time(0))
            base_pos, base_quat = self.tf_list.lookupTransform(self.wrench_base_frame,
                                                               self.netft_frame, rospy.Time(0))
        except:
            return None
        pos_diff = np.array(ft_pos) - np.array(loc_pos)
        loc_tf_wrench = wrench.copy()
        loc_tf_wrench[3:, 0] += np.mat(np.cross(pos_diff, loc_tf_wrench[:3, 0].T.A[0])).T
        base_rot_mat = np.mat(tf_trans.quaternion_matrix(base_quat))[:3,:3]
        zs = np.mat(np.zeros((3,3)))
        base_tf_wrench = np.bmat([[base_rot_mat, zs], [zs, base_rot_mat]]) * loc_tf_wrench
        return base_tf_wrench

    def rezero_wrench(self, msg):
        self.got_zero = False

    def visualize_wrench(self, wrench):
        try:
            loc_pos, loc_quat = self.tf_list.lookupTransform(self.wrench_base_frame, 
                                                             self.wrench_location_frame, rospy.Time(0))
        except:
            return
        self.publish_vector(self.wrench_base_frame, np.array(loc_pos), 0.05 * wrench[:3,0].A.T[0], 0)
        self.publish_vector(self.wrench_base_frame, np.array(loc_pos), 0.2 * wrench[3:,0].A.T[0], 1)

    def publish_vector(self, frame, loc, v, m_id):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = rospy.Time.now()
        m.ns = "netft_zeroing"
        m.id = m_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points.append(Point(*loc))
        m.points.append(Point(*(loc + v)))
        m.scale = Vector3(0.01, 0.02, 0.01)
        m.color = self.colors[m_id]
        self.vis_pub.publish(m)


def main():
    rospy.init_node("netft_gravity_zeroing")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-f', '--file', dest="filename", default="",
                 help="YAML file to save parameters in.")
    p.add_option('-l', '--load', dest="is_load",
                 action="store_true", default=False,
                 help="Load parameters from file.")
    p.add_option('-r', '--run', dest="is_run",
                 action="store_true", default=False,
                 help="Publish a zeroed wrench.")
    p.add_option('-t', '--train', dest="is_train",
                 action="store_true", default=False,
                 help="Train by moving the gripper to different poses and measuring the wrenches.")
    p.add_option('-p', '--pr2', dest="is_pr2",
                 action="store_true", default=False,
                 help="Will run automated data collection if this is the PR2.")
    p.add_option('-z', '--start_zero', dest="start_zero",
                 action="store_true", default=False,
                 help="Use the first value in to zero the sensor.")
    p.add_option('-n', '--ntrials', dest="n_trials",
                 default="6,6,4",
                 help="Number of trials for each of the last 3 joint angles to move through. (default: 6,6,4)")
    (opts, args) = p.parse_args()

    try:
        n_4, n_5, n_6 = [int(n_str) for n_str in opts.n_trials.split(',')]
    except:
        print "Bad --ntrials parameter (format: --ntrials 6,6,4)"
        p.print_help()
        return

    if opts.is_load:
        params = rosparam.load_file(opts.filename, rospy.get_name())[0][0]
        rosparam.upload_params(rospy.get_name(), params)

    if opts.is_run:
        rospy.sleep(0.1)
        nft_z = NetFTZeroer(start_zero=opts.start_zero, is_pr2=opts.is_pr2)
        rospy.spin()
        return

    if opts.is_train:
        if opts.is_pr2:
            data = collect_data_pr2(n_4, n_5, n_6)
            param_vector = process_data(data, True)
            save_params(param_vector, opts.filename)
            return
        else:
            data = collect_data_tool()
            param_vector = process_data(data, False)
            save_params(param_vector, opts.filename)
            return

if __name__ == "__main__":
    main()
