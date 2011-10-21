import numpy as np

import roslib; roslib.load_manifest('hrl_generic_arms')
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
import tf.transformations as tf_trans

class PoseConverter:
    @staticmethod
    def _make_generic(args):
        if type(args[0]) == str:
            frame_id = args[0]
            header, homo_mat, rot_quat = PoseConverter._make_generic(args[1:])
            if header is None:
                header = [0, rospy.Time.now(), '']
            header[2] = frame_id
            return header, homo_mat, rot_quat

        if len(args) == 1:
            if type(args[0]) == Pose:
                homo_mat, rot_quat = PoseConverter._extract_pose_msg(args[0])
                return None, homo_mat, rot_quat

            elif type(args[0]) == PoseStamped:
                homo_mat, rot_quat = PoseConverter._extract_pose_msg(args[0].pose)
                seq = args[0].header.seq
                stamp = args[0].header.stamp
                frame_id = args[0].header.frame_id
                return [seq, stamp, frame_id], homo_mat, rot_quat

            elif type(args[0]) is np.matrix and np.shape(args[0]) == (4, 4):
                return None, args[0], tf_trans.quaternion_from_matrix(args[0]).tolist()

            elif (type(args[0]) == tuple or type(args[0]) == list) and len(args[0]) == 2:
                header, homo_mat, rot_quat = PoseConverter._make_generic_pos_rot(args[0][0], args[0][1])
                if homo_mat is not None:
                    return header, homo_mat, rot_quat

        elif len(args) == 2:
            header, homo_mat, rot_quat = PoseConverter._make_generic_pos_rot(args[0], args[1])
            if homo_mat is not None:
                return header, homo_mat, rot_quat

            elif ((type(args[0]) == list or type(args[0]) == tuple or 
                   type(args[0]) is np.matrix or type(args[0]) == np.ndarray) and 
                  len(args[0]) == 3 and
                  (type(args[1]) == list or type(args[1]) == tuple or
                   type(args[1]) is np.matrix or type(args[1]) == np.ndarray) and 
                  len(args[1]) == 4):
                if len(np.shape(args[0])) > 1 and np.shape(args[0])[1] == 1:
                    pos = args[0]
                else:
                    pos = np.mat(args[0]).T
                homo_mat = np.mat(tf_trans.quaternion_matrix(args[1]))
                homo_mat[:3,3] = pos
                return None, homo_mat, args[1]

        assert False, "Unknown pose type"

    @staticmethod
    def _make_generic_pos_rot(pos, rot):
        if (type(pos) is np.matrix and type(rot) is np.matrix and
            np.shape(pos) == (3, 1) and np.shape(rot) == (3, 3)):
            homo_mat = np.mat(np.eye(4))
            homo_mat[:3,3] = pos
            homo_mat[:3,:3] = rot
            return None, homo_mat, tf_trans.quaternion_from_matrix(homo_mat).tolist()
        return None, None, None

    @staticmethod
    def _extract_pose_msg(pose):
        px = pose.position.x; py = pose.position.y; pz = pose.position.z
        ox = pose.orientation.x; oy = pose.orientation.y
        oz = pose.orientation.z; ow = pose.orientation.w
        quat = [ox, oy, oz, ow]
        homo_mat = np.mat(tf_trans.quaternion_matrix(quat))
        homo_mat[:3,3] = np.mat([[px, py, pz]]).T
        return homo_mat, quat

    ##
    # @return geometry_msgs.Pose
    @staticmethod
    def to_pose_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        return Pose(Point(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))

    ##
    # @return geometry_msgs.PoseStamped
    @staticmethod
    def to_pose_stamped_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        ps = PoseStamped()
        if header is None:
            ps.header.stamp = rospy.Time.now()
        else:
            ps.header.seq = header[0]
            ps.header.stamp = header[1]
            ps.header.frame_id = header[2]
        ps.pose = Pose(Point(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))
        return ps

    ##
    # @return geometry_msgs.Transform
    @staticmethod
    def to_tf_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        return Transform(Vector3(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))

    ##
    # @return geometry_msgs.TransformStamped
    @staticmethod
    def to_tf_stamped_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        tf_stamped = TransformStamped()
        if header is None:
            tf_stamped.header.stamp = rospy.Time.now()
        else:
            tf_stamped.header.seq = header[0]
            tf_stamped.header.stamp = header[1]
            tf_stamped.header.frame_id = header[2]
        tf_stamped.transform = Transform(Vector3(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))
        return tf_stamped

    ##
    # @return (3x1 numpy mat, 3x3 numpy mat)
    @staticmethod
    def to_pos_rot(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        return homo_mat[:3,3], homo_mat[:3,:3]

    ##
    # @return 4x4 numpy mat
    @staticmethod
    def to_homo_mat(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        return homo_mat

    ##
    # @return (3 list, 4 list)
    @staticmethod
    def to_pos_quat(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        return list(homo_mat[:3,3].T.A[0]), quat_rot
