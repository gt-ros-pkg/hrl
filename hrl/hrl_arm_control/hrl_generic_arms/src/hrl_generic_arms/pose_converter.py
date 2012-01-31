import numpy as np
import copy

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
            header, homo_mat, rot_quat, rot_euler = PoseConverter._make_generic(args[1:])
            if header is None:
                header = [0, rospy.Time.now(), '']
            header[2] = frame_id
            return header, homo_mat, rot_quat, rot_euler

        if len(args) == 1:
            if type(args[0]) is Pose:
                homo_mat, rot_quat, rot_euler = PoseConverter._extract_pose_msg(args[0])
                return None, homo_mat, rot_quat, rot_euler

            elif type(args[0]) is PoseStamped:
                homo_mat, rot_quat, rot_euler = PoseConverter._extract_pose_msg(args[0].pose)
                seq = args[0].header.seq
                stamp = args[0].header.stamp
                frame_id = args[0].header.frame_id
                return [seq, stamp, frame_id], homo_mat, rot_quat, rot_euler

            elif type(args[0]) is Transform:
                homo_mat, rot_quat, rot_euler = PoseConverter._extract_tf_msg(args[0])
                return None, homo_mat, rot_quat, rot_euler

            elif type(args[0]) is TransformStamped:
                homo_mat, rot_quat, rot_euler = PoseConverter._extract_tf_msg(args[0].transform)
                seq = args[0].header.seq
                stamp = args[0].header.stamp
                frame_id = args[0].header.frame_id
                return [seq, stamp, frame_id], homo_mat, rot_quat, rot_euler
                

            elif isinstance(args[0], (np.matrix, np.ndarray)) and np.shape(args[0]) == (4, 4):
                return (None, np.mat(args[0]), tf_trans.quaternion_from_matrix(args[0]).tolist(),
                        tf_trans.euler_from_matrix(args[0]).tolist())

            elif isinstance(args[0], (tuple, list)) and len(args[0]) == 2:
                pos_arg = np.mat(args[0][0])
                rot_arg = np.mat(args[0][1])
                if pos_arg.shape == (1, 3):
                    # matrix is row, convert to column
                    pos = pos_arg.T
                elif pos_arg.shape == (3, 1):
                    pos = pos_arg
                else:
                    return None, None, None, None

                if rot_arg.shape == (3, 3):
                    # rotation matrix
                    homo_mat = np.mat(np.eye(4))
                    homo_mat[:3,:3] = rot_arg
                    quat = tf_trans.quaternion_from_matrix(homo_mat)
                    rot_euler = tf_trans.euler_from_matrix(homo_mat)
                else:
                    if rot_arg.shape[1] == 1:
                        # make into row matrix
                        rot_arg = rot_arg.T
                    else:
                        rot_arg = rot_arg.tolist()[0]
                        if len(rot_arg) == 3:
                            # Euler angles rotation
                            homo_mat = np.mat(tf_trans.euler_matrix(*rot_arg))
                            quat = tf_trans.quaternion_from_euler(*rot_arg)
                            rot_euler = rot_arg
                        elif len(rot_arg) == 4:
                            # quaternion rotation
                            homo_mat = np.mat(tf_trans.quaternion_matrix(rot_arg))
                            quat = rot_arg
                            rot_euler = tf_trans.euler_from_quaternion(quat)
                        else:
                            return None, None, None, None

                homo_mat[:3, 3] = pos
                return None, homo_mat, np.array(quat), rot_euler

        elif len(args) == 2:
            header, homo_mat, rot_quat, rot_euler = PoseConverter._make_generic(
                                                                  ((args[0], args[1]),))
            if homo_mat is not None:
                return header, homo_mat, rot_quat, rot_euler

        return None, None, None, None

    @staticmethod
    def _extract_pose_msg(pose):
        px = pose.position.x; py = pose.position.y; pz = pose.position.z
        ox = pose.orientation.x; oy = pose.orientation.y
        oz = pose.orientation.z; ow = pose.orientation.w
        quat = [ox, oy, oz, ow]
        rot_euler = tf_trans.euler_from_quaternion(quat)
        homo_mat = np.mat(tf_trans.quaternion_matrix(quat))
        homo_mat[:3,3] = np.mat([[px, py, pz]]).T
        return homo_mat, quat, rot_euler

    @staticmethod
    def _extract_tf_msg(tf_msg):
        px = tf_msg.translation.x; py = tf_msg.translation.y; pz = tf_msg.translation.z 
        ox = tf_msg.rotation.x; oy = tf_msg.rotation.y
        oz = tf_msg.rotation.z; ow = tf_msg.rotation.w
        quat = [ox, oy, oz, ow]
        rot_euler = tf_trans.euler_from_quaternion(quat)
        homo_mat = np.mat(tf_trans.quaternion_matrix(quat))
        homo_mat[:3,3] = np.mat([[px, py, pz]]).T
        return homo_mat, quat, rot_euler

    ##
    # @return geometry_msgs.Pose
    @staticmethod
    def to_pose_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return Pose(Point(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))

    ##
    # @return geometry_msgs.PoseStamped
    @staticmethod
    def to_pose_stamped_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
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
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return Transform(Vector3(*homo_mat[:3,3].T.A[0]), Quaternion(*quat_rot))

    ##
    # @return geometry_msgs.TransformStamped
    @staticmethod
    def to_tf_stamped_msg(*args):
        header, homo_mat, quat_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
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
    # @return 4x4 numpy mat
    @staticmethod
    def to_homo_mat(*args):
        header, homo_mat, quat_rot, euler_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return homo_mat.copy()

    ##
    # @return (3x1 numpy mat, 3x3 numpy mat)
    @staticmethod
    def to_pos_rot(*args):
        header, homo_mat, quat_rot, euler_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return homo_mat[:3,3].copy(), homo_mat[:3,:3].copy()

    ##
    # @return (3 list, 4 list)
    @staticmethod
    def to_pos_quat(*args):
        header, homo_mat, quat_rot, euler_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return copy.copy(list(homo_mat[:3,3].T.A[0])), copy.copy(quat_rot)

    ##
    # @return (3 list, 3 list)
    @staticmethod
    def to_pos_euler(*args):
        header, homo_mat, quat_rot, euler_rot = PoseConverter._make_generic(args)
        if homo_mat is None:
            rospy.logwarn("[pose_converter] Unknown pose type.")
            return None, None, None, None
        else:
            return copy.copy(list(homo_mat[:3,3].T.A[0])), copy.copy(euler_rot)
