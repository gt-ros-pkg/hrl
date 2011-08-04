
import numpy as np

import tf.transformations as tf_trans

from hrl_generic_arms.pose_converter import PoseConverter

class SpheroidSpace:
    def __init__(self, a, center=np.mat(np.zeros((3,1))), rot=np.mat(np.eye(3))):
        self.a = a
        self.center = center
        self.rot = rot
    def spheroidal_to_cart(self, uvp):
        u, v, p = uvp
        assert u > 0 and v >= 0 and v <= np.pi and p >= 0 and p < 2 * np.pi
        x = self.a * np.sinh(u) * np.sin(v) * np.cos(p)
        y = self.a * np.sinh(u) * np.sin(v) * np.sin(p)
        z = self.a * np.cosh(u) * np.cos(v)
        pos_local = np.mat([x, y, z]).T
        return self.center + self.rot * pos_local
    def partial_u(self, uvp):
        u, v, p = uvp
        assert u > 0 and v >= 0 and v <= np.pi and p >= 0 and p < 2 * np.pi
        x = self.a * np.cosh(u) * np.sin(v) * np.cos(p)
        y = self.a * np.cosh(u) * np.sin(v) * np.sin(p)
        z = self.a * np.sinh(u) * np.cos(v)
        return self.rot * np.mat([x, y, z]).T
    def partial_v(self, uvp):
        u, v, p = uvp
        assert u > 0 and v >= 0 and v <= np.pi and p >= 0 and p < 2 * np.pi
        x = self.a * np.sinh(u) * np.cos(v) * np.cos(p)
        y = self.a * np.sinh(u) * np.cos(v) * np.sin(p)
        z = self.a * np.cosh(u) * -np.sin(v)
        return self.rot * np.mat([x, y, z]).T
    def partial_p(self, uvp):
        u, v, p = uvp
        assert u > 0 and v >= 0 and v <= np.pi and p >= 0 and p < 2 * np.pi
        x = self.a * np.sinh(u) * np.sin(v) * -np.sin(p)
        y = self.a * np.sinh(u) * np.sin(v) * np.cos(p)
        z = 0.
        return self.rot * np.mat([x, y, z]).T
    def spheroidal_to_pose(self, uvp, rot_gripper=0.):
        pos = self.spheroidal_to_cart(uvp)
        df_du = self.partial_u(uvp)
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
        
