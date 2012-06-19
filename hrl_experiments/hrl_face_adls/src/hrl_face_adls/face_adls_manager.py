#! /usr/bin/python
        
import numpy as np
import sys
import cPickle as pkl
from threading import Lock

import roslib
roslib.load_manifest('hrl_face_adls')

import rospy
from std_msgs.msg import Int8, String, Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
import tf.transformations as tf_trans

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2ArmCartesianBase, PR2ArmJTransposeTask
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from hrl_ellipsoidal_control.ellipsoid_controller import EllipsoidController
from hrl_ellipsoidal_control.ellipsoidal_parameters import *
from hrl_face_adls.face_adls_parameters import *
from hrl_face_adls.msg import StringArray
from hrl_face_adls.srv import EnableFaceController, EnableFaceControllerResponse

##
# Returns a function which will call the callback immediately in
# a different thread.
def async_call(cb):
    def cb_outer(*args, **kwargs):
        def cb_inner(te):
            cb(*args, **kwargs)
        rospy.Timer(rospy.Duration(0.00001), cb_inner, oneshot=True)
    return cb_outer

class ForceCollisionMonitor(object):
    def __init__(self):
        self.last_activity_time = rospy.get_time()
        self.last_reading = rospy.get_time()
        self.last_dangerous_cb_time = 0.0
        self.last_timeout_cb_time = 0.0
        self.last_contact_cb_time = 0.0
        self.in_action = False
        self.lock = Lock()
        self.dangerous_force_thresh = rospy.get_param("~dangerous_force_thresh", 10.0)
        self.activity_force_thresh = rospy.get_param("~activity_force_thresh", 3.0)
        self.contact_force_thresh = rospy.get_param("~contact_force_thresh", 3.0)
        self.timeout_time = rospy.get_param("~timeout_time", 30.0)
        rospy.Subscriber('/netft_gravity_zeroing/wrench_zeroed', WrenchStamped, 
                         async_call(self.force_cb), queue_size=1)
        def check_readings(te):
            time_diff = rospy.get_time() - self.last_reading
            if time_diff > 3.:
                rospy.logerr("Force monitor hasn't recieved force readings for %.1f seconds!"
                              % time_diff)
        rospy.Timer(rospy.Duration(3), check_readings)

    def force_cb(self, msg):
        self.last_reading = rospy.get_time()
        if self.lock.acquire(False):
            force_mag = np.linalg.norm([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            if (force_mag > self.dangerous_force_thresh and
                    rospy.get_time() - self.last_dangerous_cb_time > DANGEROUS_CB_COOLDOWN):
                self.dangerous_cb(self.dangerous_force_thresh)
                self.last_dangerous_cb_time = rospy.get_time()
            if (force_mag > self.contact_force_thresh and 
                    rospy.get_time() - self.last_contact_cb_time > CONTACT_CB_COOLDOWN):
                self.contact_cb(self.contact_force_thresh)
                self.last_contact_cb_time = rospy.get_time()
            if force_mag > self.activity_force_thresh:
                self.update_activity()
            if self.is_inactive() and rospy.get_time() - self.last_timeout_cb_time > TIMEOUT_CB_COOLDOWN:
                self.timeout_cb(self.timeout_time)
                self.last_timeout_cb_time = rospy.get_time()
            self.lock.release()

    def is_inactive(self):
        return not self.in_action and rospy.get_time() - self.last_activity_time > self.timeout_time

    def register_contact_cb(self, cb=lambda x:None):
        self.contact_cb = async_call(cb)

    def register_dangerous_cb(self, cb=lambda x:None):
        self.dangerous_cb = async_call(cb)

    def register_timeout_cb(self, cb=lambda x:None):
        self.timeout_cb = async_call(cb)

    def update_activity(self):
        self.last_activity_time = rospy.get_time()

    def start_activity(self):
        self.in_action = True

    def stop_activity(self):
        self.last_activity_time = rospy.get_time()
        self.in_action = False

class FaceADLsManager(object):
    def __init__(self):

        self.ell_ctrl = EllipsoidController()
        self.ctrl_switcher = ControllerSwitcher()

        self.global_input_sub = rospy.Subscriber("/face_adls/global_move", String, 
                                                 async_call(self.global_input_cb))
        self.local_input_sub = rospy.Subscriber("/face_adls/local_move", String, 
                                                async_call(self.local_input_cb))
        self.state_pub = rospy.Publisher('/face_adls/controller_state', Int8, latch=True)
        self.feedback_pub = rospy.Publisher('/face_adls/feedback', String)
        self.global_move_poses_pub = rospy.Publisher('/face_adls/global_move_poses', StringArray, 
                                                     latch=True)
        def enable_controller_cb(req):
            if req.enable:
                success = self.enable_controller(req.end_link, req.ctrl_params)
            else:
                success = self.disable_controller()
            return EnableFaceControllerResponse(success)
        self.controller_enabled_pub = rospy.Publisher('/face_adls/controller_enabled', Bool, latch=True)
        self.enable_controller_srv = rospy.Service("/face_adls/enable_controller", 
                                                   EnableFaceController, enable_controller_cb)
        self.disable_controller()

    def publish_feedback(self, message=None, transition_id=None):
        if message is not None:
            rospy.loginfo("[face_adls_manager] %s" % message)
            self.feedback_pub.publish(message)
        if transition_id is not None:
            self.state_pub.publish(Int8(transition_id))

    def enable_controller(self, end_link="%s_gripper_shaver45_frame",
                          ctrl_params="$(find hrl_face_adls)/params/l_jt_task_shaver45.yaml",
                          ctrl_name='%s_cart_jt_task'):
        if not self.ell_ctrl.params_loaded():
            self.publish_feedback(Messages.NO_PARAMS_LOADED)
            return False
        is_scratching = rospy.get_param('/is_scratching', False) # TODO BETTER SOLUTION!
        if is_scratching: # TODO
            end_link = '%s_gripper_brush45_frame'
            ctrl_params = "$(find hrl_face_adls)/params/l_jt_task_brush45.yaml"
            ctrl_name = '%s_cart_jt_task_brush'

        self.ctrl_switcher.carefree_switch('l', ctrl_name, ctrl_params, reset=False)
        rospy.sleep(0.2)
        cart_arm = create_pr2_arm('l', PR2ArmJTransposeTask, 
                                  controller_name=ctrl_name, 
                                  end_link=end_link, timeout=5)
        self.ell_ctrl.set_arm(cart_arm)

        shaving_side = rospy.get_param('/shaving_side', 'r') # TODO Make more general
        if not is_scratching: # TODO
            self.ell_ctrl.set_bounds(LAT_BOUNDS[shaving_side], LON_BOUNDS[shaving_side],
                                     HEIGHT_BOUNDS[shaving_side])
        # check if arm is near head
        if not self.ell_ctrl.arm_in_bounds():
            self.publish_feedback(Messages.ARM_AWAY_FROM_HEAD)
            return False

        self.publish_feedback(Messages.ENABLE_CONTROLLER)

        self.force_monitor = ForceCollisionMonitor()
        # registering force monitor callbacks
        def dangerous_cb(force):
            if not self.ell_ctrl.is_moving() and self.check_controller_ready():
                ell_ep = self.ell_ctrl.get_ell_ep()
                if ell_ep[2] < SAFETY_RETREAT_HEIGHT * 0.9:
                    self.publish_feedback(Messages.DANGEROUS_FORCE % force)
                    self.retreat_move(SAFETY_RETREAT_HEIGHT, SAFETY_RETREAT_VELOCITY, forced=True)
        self.force_monitor.register_dangerous_cb(dangerous_cb)
        def timeout_cb(time):
            if not self.ell_ctrl.is_moving() and self.check_controller_ready():
                start_time = rospy.get_time()
                ell_ep = self.ell_ctrl.get_ell_ep()
                rospy.loginfo("ELL EP TIME: %.3f " % (rospy.get_time() - start_time) + str(ell_ep) + " times: %.3f %.3f" % (self.force_monitor.last_activity_time, rospy.get_time()))
                if ell_ep[2] < RETREAT_HEIGHT * 0.9 and self.force_monitor.is_inactive():
                    self.publish_feedback(Messages.TIMEOUT_RETREAT % time)
                    self.retreat_move(RETREAT_HEIGHT, LOCAL_VELOCITY)
        self.force_monitor.register_timeout_cb(timeout_cb)
        def contact_cb(force):
            self.force_monitor.update_activity()
            if self.ell_ctrl.is_moving() and not self.is_forced_retreat:
                self.ell_ctrl.stop_moving(True)
                self.publish_feedback(Messages.CONTACT_FORCE % force)
                rospy.loginfo("Arm stopped due to making contact.")
        self.force_monitor.register_contact_cb(contact_cb)
        self.force_monitor.update_activity()
        self.is_forced_retreat = False

        self.global_poses = rospy.get_param('/face_adls/%s_global_poses' % shaving_side)
        self.global_move_poses_pub.publish(self.global_poses.keys())

        if shaving_side == 'r' and not is_scratching: # TODO
            self.gripper_rot = tf_trans.quaternion_from_euler(np.pi, 0, 0)
        else:
            self.gripper_rot = tf_trans.quaternion_from_euler(0, 0, 0)

        self.controller_enabled_pub.publish(Bool(True))
        return True

    def disable_controller(self):
        self.ell_ctrl.stop_moving(wait=True)
        self.ell_ctrl.set_arm(None)
        self.controller_enabled_pub.publish(Bool(False))
        self.publish_feedback(Messages.DISABLE_CONTROLLER)
        return True

    def controller_enabled(self):
        return self.ell_ctrl.arm is not None

    def retreat_move(self, height, velocity, forced=False):
        self.force_monitor.start_activity()
        if not self.check_controller_ready():
            return
        
        if forced:
            self.is_forced_retreat = True
        self.ell_ctrl.execute_ell_move(((0, 0, height), (0, 0, 0)), ((0, 0, 1), 0), 
                                       self.gripper_rot, velocity, blocking=False)
        rospy.loginfo("[face_adls_manager] Retreating from current location.")
        self.ell_ctrl.wait_until_stopped()
        self.is_forced_retreat = False
        self.force_monitor.stop_activity()
        rospy.loginfo("[face_adls_manager] Finished retreat.")

    def stop_move(self):
        self.force_monitor.update_activity()
        if not self.ell_ctrl.is_moving():
            return False
        self.ell_ctrl.stop_moving(True)
        rospy.loginfo("Stopped controller.")
        return True

    def global_input_cb(self, msg):
        if not self.check_controller_ready() or self.is_forced_retreat:
            return
        if self.stop_move():
            rospy.loginfo("[face_adls_manager] Preempting other movement for global move.")
            #self.publish_feedback(Messages.GLOBAL_PREEMPT_OTHER)
        self.force_monitor.start_activity()
        goal_pose = self.global_poses[msg.data]
        goal_pose_name = msg.data
        self.publish_feedback(Messages.GLOBAL_START % goal_pose_name)
        try:
            if not self.ell_ctrl.execute_ell_move(((0, 0, RETREAT_HEIGHT), (0, 0, 0)), ((0, 0, 1), 0), 
                                                  self.gripper_rot, APPROACH_VELOCITY, blocking=True):
                raise Exception
            if not self.ell_ctrl.execute_ell_move(((goal_pose[0][0], goal_pose[0][1], RETREAT_HEIGHT), 
                                                  (0, 0, 0)), 
                                                  ((1, 1, 1), 0), 
                                                  self.gripper_rot, GLOBAL_VELOCITY, blocking=True):
                raise Exception
            if not self.ell_ctrl.execute_ell_move((goal_pose[0], (0, 0, 0)), ((1, 1, 1), 0), 
                                                  self.gripper_rot, GLOBAL_VELOCITY, blocking=True):
                raise Exception
        except:
            self.publish_feedback(Messages.GLOBAL_PREEMPT % goal_pose_name)
            self.force_monitor.stop_activity()
            return
        self.publish_feedback(Messages.GLOBAL_SUCCESS % goal_pose_name)
        self.force_monitor.stop_activity()

    def check_controller_ready(self):
        if not self.ell_ctrl.params_loaded() or not self.controller_enabled():
            #rospy.logerr("Ellipsoidal parameters not loaded")
            return False
        return True

    def local_input_cb(self, msg):
        if not self.check_controller_ready() or self.is_forced_retreat:
            return
        if self.stop_move():
            rospy.loginfo("[face_adls_manager] Preempting other movement for local move.")
            #self.publish_feedback(Messages.LOCAL_PREEMPT_OTHER)
        self.force_monitor.start_activity()
        button_press = msg.data 
        if button_press in ell_trans_params:
            self.publish_feedback(Messages.LOCAL_START % button_names_dict[button_press])
            change_trans_ep = ell_trans_params[button_press]
            success = self.ell_ctrl.execute_ell_move((change_trans_ep, (0, 0, 0)), ((0, 0, 0), 0), 
                                                    self.gripper_rot, ELL_LOCAL_VEL, blocking=True)
        elif button_press in ell_rot_params:
            self.publish_feedback(Messages.LOCAL_START % button_names_dict[button_press])
            change_rot_ep = ell_rot_params[button_press]
            success = self.ell_ctrl.execute_ell_move(((0, 0, 0), change_rot_ep), ((0, 0, 0), 0), 
                                                    self.gripper_rot, ELL_ROT_VEL, blocking=True)
        elif button_press == "reset_rotation":
            self.publish_feedback(Messages.ROT_RESET_START)
            success = self.ell_ctrl.execute_ell_move(((0, 0, 0), np.mat(np.eye(3))), ((0, 0, 0), 1), 
                                                    self.gripper_rot, ELL_ROT_VEL, blocking=True)
        else:
            rospy.logerr("[face_adls_manager] Unknown ellipsoidal local command")

        if success:
            self.publish_feedback(Messages.LOCAL_SUCCESS % button_names_dict[button_press])
        else:
            self.publish_feedback(Messages.LOCAL_PREEMPT % button_names_dict[button_press])
        self.force_monitor.stop_activity()

def main():
    rospy.init_node("face_adls_manager")

    #from pr2_traj_playback.arm_pose_move_controller import ArmPoseMoveBehavior, TrajectoryLoader
    #from pr2_traj_playback.arm_pose_move_controller import CTRL_NAME_LOW, PARAMS_FILE_LOW
    #r_apm = ArmPoseMoveBehavior('r', ctrl_name=CTRL_NAME_LOW,
    #                            param_file=PARAMS_FILE_LOW)
    #l_apm = ArmPoseMoveBehavior('l', ctrl_name=CTRL_NAME_LOW,
    #                            param_file=PARAMS_FILE_LOW)
    #traj_loader = TrajectoryLoader(r_apm, l_apm)
    #if False:
    #    traj_loader.move_to_setup_from_file("$(find hrl_face_adls)/data/l_arm_shaving_setup_r.pkl",
    #                                        velocity=0.1, reverse=False, blocking=True)
    #    traj_loader.exec_traj_from_file("$(find hrl_face_adls)/data/l_arm_shaving_setup_r.pkl",
    #                                    rate_mult=0.8, reverse=False, blocking=True)
    #if False:
    #    traj_loader.move_to_setup_from_file("$(find hrl_face_adls)/data/l_arm_shaving_setup_r.pkl",
    #                                        velocity=0.3, reverse=True, blocking=True)

    fam = FaceADLsManager()
    #fam.enable_controller()
    rospy.spin()

if __name__ == "__main__":
    main()
