
import numpy as np

import tf.transformations as tf_trans

from hrl_generic_arms.pose_converter import PoseConverter

class EllipsoidSpace:
    def __init__(self, a, center=np.mat(np.zeros((3,1))), rot=np.mat(np.eye(3))):
        self.A = A
        self.B = B
        self.E = np.sqrt(np.fabs(A**2 + B**2)) / A
        self.a = self.A * self.E
        self.center = center
        self.rot = rot
    def ellipsoidal_to_cart(self, lat, lon, height):
        #assert height > 0 and lat >= 0 and lat <= np.pi and lon >= 0 and lon < 2 * np.pi
        x = self.a * np.sinh(height) * np.sin(lat) * np.cos(lon)
        y = self.a * np.sinh(height) * np.sin(lat) * np.sin(lon)
        z = self.a * np.cosh(height) * np.cos(lat)
        pos_local = np.mat([x, y, z]).T
        return self.center + self.rot * pos_local
    def partial_u(self, lat, lon, height):
        #assert height > 0 and lat >= 0 and lat <= np.pi and lon >= 0 and lon < 2 * np.pi
        x = self.a * np.cosh(height) * np.sin(lat) * np.cos(lon)
        y = self.a * np.cosh(height) * np.sin(lat) * np.sin(lon)
        z = self.a * np.sinh(height) * np.cos(lat)
        return self.rot * np.mat([x, y, z]).T
    def partial_v(self, lat, lon, height):
        #assert height > 0 and lat >= 0 and lat <= np.pi and lon >= 0 and lon < 2 * np.pi
        x = self.a * np.sinh(height) * np.cos(lat) * np.cos(lon)
        y = self.a * np.sinh(height) * np.cos(lat) * np.sin(lon)
        z = self.a * np.cosh(height) * -np.sin(lat)
        return self.rot * np.mat([x, y, z]).T
    def partial_p(self, lat, lon, height):
        #assert height > 0 and lat >= 0 and lat <= np.pi and lon >= 0 and lon < 2 * np.pi
        x = self.a * np.sinh(height) * np.sin(lat) * -np.sin(lon)
        y = self.a * np.sinh(height) * np.sin(lat) * np.cos(lon)
        z = 0.
        return self.rot * np.mat([x, y, z]).T
    def ellipsoidal_to_pose(self, lat, lon, height, rot_gripper=0.):
        pos = self.ellipsoidal_to_cart(lat, lon, height)
        df_du = self.partial_u(lat, lon, height)
        nx, ny, nz = df_du.T.A[0] / np.linalg.norm(df_du)
        j = np.sqrt(1./(1.+ny*ny/(nz*nz)))
        k = -ny*j/nz
        norm_rot = np.mat([[-nx,  ny*k - nz*j,  0],      
                           [-ny,  -nx*k,        j],      
                           [-nz,  nx*j,         k]])
        _, norm_quat = PoseConverter.to_pos_quat(np.mat([0, 0, 0]).T, norm_rot)
        rot_angle = np.arctan(-norm_rot[2,1] / norm_rot[2,2])
        print norm_rot
        quat_ortho_rot = tf_trans.quaternion_from_euler(rot_angle + np.pi + rot_gripper, 0.0, 0.0)
        norm_quat_ortho = tf_trans.quaternion_multiply(norm_quat, quat_ortho_rot)
        norm_rot_ortho = np.mat(tf_trans.quaternion_matrix(norm_quat_ortho)[:3,:3])
        if norm_rot_ortho[1, 1] > 0:
            flip_axis_ang = 0
        else:
            flip_axis_ang = np.pi
        quat_flip = tf_trans.quaternion_from_euler(flip_axis_ang, 0.0, 0.0)
        norm_quat_ortho_flipped = tf_trans.quaternion_multiply(norm_quat_ortho, quat_flip)
        return PoseConverter.to_pos_rot(pos, norm_quat_ortho_flipped)
        
