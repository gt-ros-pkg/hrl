#! /usr/bin/python
        
import sys

import roslib
roslib.load_manifest('hrl_ellipsoidal_control')

from std_msgs.msg import Int8, String

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController
from hrl_ellipsoidal_control.ellipsoidal_parameters import *

quat_gripper_rot = tf_trans.quaternion_from_euler(np.pi, 0, 0)

class ForceCollisionMonitor(object):
    def __init__(self):
        rospy.Subscriber('/netft_gravity_zeroing/wrench_zeroed', WrenchStamped, self.force_cb)
        self.last_contact_time = 0.
        self.lock = Lock()

    def force_cb(self, msg):
        with self.lock:
            force_mag = np.linalg.norm([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            if force_mag > DANGEROUS_FORCE_THRESH:
                self.dangerous_cb()
            if force_mag > CONTACT_FORCE_THRESH:
                self.contact_cb()
            if force_mag > ACTIVITY_FORCE_THRESH:
                self.last_contact_time = rospy.get_time()
            if rospy.get_time() - self.last_contact_time > TIMEOUT_TIME:
                self.timeout_cb()

    def register_contact_cb(self, cb=lambda:None):
        self.contact_cb = cb

    def register_dangerous_cb(self, cb=lambda:None):
        self.dangerous_cb = cb

    def register_timeout_cb(self, cb=lambda:None):
        self.timeout_cb = cb


class FaceADLsManager(object):
    def __init__(self):
        # load global_poses from file
        poses_path = roslib.substitution_args.resolve_args(
                        '$(find hrl_rfh_fall_2011)/data/bilateral_poses.pkl')
        f = file(path, 'r')
        pose_dict = pkl.load(f)
        self.pose_names = sorted(pose_dict.keys())
        self.global_poses = [pose_dict[pose_name] for pose_name in self.pose_names]
        f.close()

        self.is_forced_retreat = False
        self.force_monitor = ForceCollisionMonitor()
        self.ell_ctrl = EllipsoidController()

        # registering force monitor callbacks
        def dangerous_cb():
            if not self.ell_ctrl.is_moving():
                ell_ep = self.ell_ctrl.get_ell_ep()
                if ell_ep[2] < SAFETY_RETREAT_HEIGHT * 0.9:
                    self.retreat_move(SAFETY_RETREAT_HEIGHT, SAFETY_RETREAT_VELOCITY)
        self.force_monitor.register_dangerous_cb(dangerous_cb)
        def timeout_cb():
            if not self.ell_ctrl.is_moving():
                ell_ep = self.ell_ctrl.get_ell_ep()
                if ell_ep[2] < RETREAT_HEIGHT * 0.9:
                    self.retreat_move(RETREAT_HEIGHT, LOCAL_VELOCITY)
        self.force_monitor.register_timeout_cb(timeout_cb)
        def contact_cb():
            if self.ell_ctrl.is_moving() and not self.is_forced_retreat:
                self.ell_ctrl.stop_moving(True)
                rospy.loginfo("Arm stopped due to making contact.")
        self.force_monitor.register_contact_cb(contact_cb)

        self.global_input_sub = rospy.Subscriber("/wt_shave_location", Int8, self.global_input_cb)
        self.local_input_sub = rospy.Subscriber("/wt_shave_step", String, self.local_input_cb)

    def enable_controller(self):
        cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                                  controller_name='%s_cart_jt_task', 
                                  end_link="%s_gripper_shaver45_frame", timeout=5)
        self.ell_ctrl.set_arm(cart_arm)

    def retreat_move(self, height, velocity):
        
        self.ell_ctrl.execute_ell_move(((0, 0, height), (0, 0, 0)), ((0, 0, 1), 0), 
                                       quat_gripper_rot, velocity, blocking=False)
        rospy.loginfo("Retreating from current location.")
        self.ell_ctrl.wait_until_stopped()
        rospy.loginfo("Finished retreat.")

    def stop_move(self):
        if not self.ell_ctrl.is_moving():
            return
        self.ell_ctrl.stop_moving(True)
        rospy.loginfo("Stopped controller.")

    def global_input_cb(self, msg):
        if self.ell_ctrl.is_moving():
            return
        goal_pose = self.global_poses[msg.data]
        goal_pose_name = self.global_names[msg.data]
        rospy.loginfo("Starting global ellipsoid movement.")
        if not self.ell_ctrl.execute_ell_move(((0, 0, RETREAT_HEIGHT), (0, 0, 0)), ((0, 0, 1), 0), 
                                              quat_gripper_rot, APPROACH_VELOCITY, blocking=True)
            rospy.loginfo("Ellipsoid global movement to pose %s preempted." % goal_pose_name)
            return
        if not self.ell_ctrl.execute_ell_move(((goal_pose[0], goal_pose[1], RETREAT_HEIGHT), 
                                              (0, 0, 0)), 
                                              ((1, 1, 1), 0), 
                                              quat_gripper_rot, GLOBAL_VELOCITY, blocking=True)
            rospy.loginfo("Ellipsoid global movement to pose %s preempted." % goal_pose_name)
            return
        if not self.ell_ctrl.execute_ell_move((goal_pose, (0, 0, 0)), ((1, 1, 1), 0), 
                                              quat_gripper_rot, GLOBAL_VELOCITY, blocking=True)
            rospy.loginfo("Ellipsoid global movement to pose %s preempted." % goal_pose_name)
            return

        rospy.loginfo("Ellipsoid global movement to pose %s successful." % goal_pose_name)

    def local_input_cb(self, msg):
        if self.ell_ctrl.is_moving():
            return
        button_press = msg.data 
        if button_press in ell_trans_params:
            rospy.loginfo("Starting local ellipsoid linear movement")
            change_trans_ep = ell_trans_params[button_press]
            success = self.ell_ctrl.execute_ell_move((change_trans_ep, (0, 0, 0)), ((0, 0, 0), 0), 
                                                    quat_gripper_rot, ELL_LOCAL_VEL, blocking=True)
        elif button_press in ell_rot_params:
            rospy.loginfo("Starting local ellipsoid rotation movement")
            change_rot_ep = ell_rot_params[button_press]
            success = self.ell_ctrl.execute_ell_move(((0, 0, 0), change_rot_ep), ((0, 0, 0), 0), 
                                                    quat_gripper_rot, ELL_ROT_VEL, blocking=True)
        elif button_press == "reset_rotation":
            rospy.loginfo("Starting local ellipsoid reset movement")
            success = self.ell_ctrl.execute_ell_move(((0, 0, 0), np.mat(np.eye(3))), ((0, 0, 0), 1), 
                                                    quat_gripper_rot, ELL_ROT_VEL, blocking=True)

        if success:
            rospy.loginfo("Finished local ellipsoid movement.")
        else:
            rospy.loginfo("Local ellipsoid movement preempted.")

def main():
    rospy.init_node("face_adls_manager")
    fam = FaceADLsManager()
    fam.enable_controller()
    rospy.spin()

if __name__ == "__main__":
    main()
