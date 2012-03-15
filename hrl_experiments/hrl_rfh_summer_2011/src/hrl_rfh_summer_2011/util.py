import numpy as np

import roslib; roslib.load_manifest('hrl_rfh_summer_2011')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf.transformations as tf_trans

def extract_pose_stamped(ps):
    seq = ps.header.seq; stamp = ps.header.stamp; frame_id = ps.header.frame_id
    px = ps.pose.position.x; py = ps.pose.position.y; pz = ps.pose.position.z
    ox = ps.pose.orientation.x; oy = ps.pose.orientation.y
    oz = ps.pose.orientation.z; ow = ps.pose.orientation.w
    return [seq, stamp, frame_id], [px, py, pz], [ox, oy, oz, ow]

def pose_msg_to_mat(ps):
    header, pos, quat = extract_pose_stamped(ps)
    return pose_pq_to_mat(pos, quat)

def pose_pq_to_mat(pos, quat):
    B = np.mat(tf_trans.quaternion_matrix(quat))
    B[0:3,3] = np.mat([pos]).T
    return B

def pose_mat_to_pq(mat):
    return mat[:3,3].T.tolist()[0], tf_trans.quaternion_from_matrix(mat)

def pose_mat_to_msg(B):
    pos = B[0:3,3].T.tolist()
    quat = tf_trans.quaternion_from_matrix(B)
    return Pose(Point(*pos[0]), Quaternion(*quat))

def pose_mat_to_stamped_msg(frame_id, B, time=None):
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    if time is None:
        ps.header.stamp = rospy.Time.now()
    else:
        ps.header.stamp = time
    ps.pose = pose_mat_to_msg(B)
    return ps

def quaternion_dist(B_a, B_b):
    quat_a = tf_trans.quaternion_from_matrix(B_a)
    quat_b = tf_trans.quaternion_from_matrix(B_b)
    quat_a_norm = quat_a / np.linalg.norm(quat_a)
    quat_b_norm = quat_b / np.linalg.norm(quat_b)
    dot = np.dot(quat_a_norm, quat_b_norm)
    if dot > 0.99999:
        dist = 0
    else:
        dist = np.arccos(dot)
    return dist

def interpolate_cartesian(start_pose, end_pose, num_steps):
    diff_pos = end_pose[:3,3] - start_pose[:3,3]
    start_quat = tf_trans.quaternion_from_matrix(start_pose)
    end_quat = tf_trans.quaternion_from_matrix(end_pose)
    mat_list = []
    for fraction in np.linspace(0, 1, num_steps):
        cur_quat = tf_trans.quaternion_slerp(start_quat, end_quat, fraction)
        cur_mat = np.mat(tf_trans.quaternion_matrix(cur_quat))
        cur_mat[:3,3] = start_pose[:3,3] + fraction * diff_pos
        mat_list.append(cur_mat)
    return mat_list
