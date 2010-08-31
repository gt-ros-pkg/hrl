import roslib
roslib.load_manifest('hrl_lib')

import rospy
import tf.transformations as tr
import numpy as np

def tf_as_matrix(tup):
    return np.matrix(tr.translation_matrix(tup[0])) * np.matrix(tr.quaternion_matrix(tup[1])) 

def matrix_as_tf(mat):
    return (tr.translation_from_matrix(mat), tr.quaternion_from_matrix(mat))

def transform(to_frame, from_frame, tflistener, t=0):
    return tf_as_matrix(tflistener.lookupTransform(to_frame, from_frame, rospy.Time(t)))

def transform_points(T, points):
    return (T * np.row_stack((points, 1+np.zeros((1, points.shape[1])))))[0:3,:]

def rotate(to_frame, from_frame, tflistener, t=0):
    t, q = tflistener.lookupTransform(to_frame, from_frame, rospy.Time(t))
    return np.matrix(tr.quaternion_matrix(q)) 

def quaternion_matrix(quat):
    return np.matrix(tr.quaternion_matrix(quat))

def translation_matrix(trans):
    return np.matrix(tr.translation_matrix(trans))

def pose_as_matrix(p):
    t = [p.position.x, p.position.y, p.position.z]
    o = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    return tf_as_matrix((t, o))

def posestamped_as_matrix(ps):
    p = ps.pose
    return pose_as_matrix(p), ps.header.frame_id
