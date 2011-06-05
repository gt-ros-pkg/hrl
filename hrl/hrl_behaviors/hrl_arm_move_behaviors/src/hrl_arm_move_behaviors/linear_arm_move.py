#! /usr/bin/python

import numpy as np
import copy

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_srvs.srv
import tf.transformations as tf_trans

from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from hrl_arm_move_behaviors.msg import LinearMoveRelativeGoal, LinearMoveRelativeAction
from hrl_arm_move_behaviors.msg import LinearMoveRelativeResult, LinearMoveRelativeSetupGoal
from hrl_arm_move_behaviors.msg import LinearMoveRelativeSetupAction, LinearMoveRelativeSetupResult
from pr2_collision_monitor.srv import JointDetectionStart, ArmMovingWait

##
# Transforms the given position by the offset position in the given quaternion
# rotation frame
def transform_in_frame(pos, quat, off_point):
    quatmat = np.mat(tf_trans.quaternion_matrix(quat))
    quatmat[0:3,3] = np.mat(pos).T
    trans = np.matrix([off_point[0],off_point[1],off_point[2],1.]).T
    transpos = quatmat * trans
    return transpos.T.A[0,0:3]

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

def pose_mat_to_msg(B):
    pos = B[0:3,3].T.tolist()
    quat = tf_trans.quaternion_from_matrix(B)
    return Pose(Point(*pos[0]), Quaternion(*quat))


class LinearMoveRelative(object):
    def __init__(self):
        self.MOVE_VELOCITY = 0.05
        self.SETUP_VELOCITY = 0.15
        self.JOINTS_BIAS = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.BIAS_RADIUS = 0.012
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]
        #self.INIT_ANGS = [0.31224765812286392, 0.67376890754823038, -0.033887965389851837, -2.0722385314846998, -3.1341338126914398, -1.3392539168745463, -0.04655005168062587]

        self.arm = rospy.get_param("~arm", default="r")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

        self.cm = ControllerManager(self.arm)

        rospy.loginfo("Waiting for start_monitoring")
        rospy.wait_for_service(self.arm + '_collision_monitor/start_monitoring')
        self.start_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/start_monitoring', 
                                                  JointDetectionStart, persistent=False)
        rospy.loginfo("Waiting for stop_monitoring")
        rospy.wait_for_service(self.arm + '_collision_monitor/stop_monitoring')
        self.stop_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/stop_monitoring', 
                                                 std_srvs.srv.Empty, persistent=False)
        self.stop_detection()
        rospy.loginfo("Waiting for arm_moving_wait")
        rospy.wait_for_service(self.arm + '_arm_moving_server/arm_moving_wait')
        self.arm_moving_wait = rospy.ServiceProxy(self.arm + '_arm_moving_server/arm_moving_wait', 
                                                  ArmMovingWait, persistent=False)

        class CollisionState:
            def __init__(self, lmr):
                self.collided = False
                self.lmr = lmr
            def callback(self, msg):
                if msg.data:
                    self.collided = True
                    self.lmr.cm.joint_action_client.cancel_all_goals()
        self.coll_state = CollisionState(self)
        rospy.Subscriber(self.arm + "_collision_monitor/collision_detected", 
                         Bool, self.coll_state.callback)

        self.move_arm_act = actionlib.SimpleActionServer("~move_relative", 
                                                         LinearMoveRelativeAction, 
                                                         self.execute_move, False)
        self.move_arm_act.start()

        self.move_arm_setup_act = actionlib.SimpleActionServer("~move_relative_setup", 
                                                               LinearMoveRelativeSetupAction, 
                                                               self.execute_setup, False)
        self.move_arm_setup_act.start()

        self.pix3_pub = rospy.Publisher("/lin_pose3", PoseStamped)
        rospy.loginfo("[linear_move_relative] LinearMoveRelative loaded.")

    #############################################################################
    # Transform goal's desired location to a pose in the wrist frame such that
    # the offset frame will be located at the goal
    def transform_pose_for_wrist(self, goal, offset_frame):
        # get goal in torso frame D_torso
        self.cm.tf_listener.waitForTransform("torso_lift_link", goal.goal_pose.header.frame_id,
                                             rospy.Time(0), rospy.Duration(5))
        goal.goal_pose.header.stamp = rospy.Time()
        t_goal_f = self.cm.tf_listener.transformPose("torso_lift_link", goal.goal_pose)
        # get toolframe_B_wristframe
        f_pos_w, f_quat_w = self.cm.tf_listener.lookupTransform(offset_frame, 
                                                   self.arm + "_wrist_roll_link", rospy.Time())
        # find the wrist goal transformed from the tool frame goal
        t_goal_w_mat = (pose_msg_to_mat(t_goal_f) * pose_pq_to_mat(f_pos_w, f_quat_w))
        t_goal_w = PoseStamped()
        t_goal_w.header.frame_id = "torso_lift_link"
        t_goal_w.header.stamp = rospy.Time.now() + rospy.Duration(0.3)
        t_goal_w.pose = pose_mat_to_msg(t_goal_w_mat)
        return t_goal_w

    def execute_move(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement.")
        result = LinearMoveRelativeResult()

        move_goal = self.transform_pose_for_wrist(goal, self.tool_frame)

        wrist_pose = self.cm.get_current_wrist_pose_stamped("torso_lift_link")

        rospy.loginfo("[linear_move_relative] Waiting for arm to stop moving.")
        while not rospy.is_shutdown():
            # Wait for the arm to stop moving (non blocking)
            if not self.arm_moving_wait(False, 0.0).is_moving:
                break
            rospy.sleep(0.05)
        rospy.loginfo("[linear_move_relative] The arm is not moving.")
        self.pix3_pub.publish(move_goal)

        move_result = self.cm.move_cartesian_ik(move_goal, start_pose=wrist_pose,
                                                collision_aware=False, 
                                                blocking=False,
                                                step_size=.005, pos_thres = .005, rot_thres = .05,
                                                settling_time=rospy.Duration(2),
                                                joints_bias=self.JOINTS_BIAS, 
                                                bias_radius=self.BIAS_RADIUS,
                                                vel=self.MOVE_VELOCITY)
        print "move_result", move_result

        if move_result == "no solution":
            self.move_arm_setup_act.set_aborted(result)
            return

        while not rospy.is_shutdown():
            # Wait for the arm to start moving (non blocking)
            if self.arm_moving_wait(False, 0.0).is_moving:
                break
            rospy.sleep(0.05)
        rospy.loginfo("[linear_move_relative] Arm has started moving.")

        self.start_detection("overhead_grasp", 1.0)

        rospy.sleep(2)
        self.arm_moving_wait(True, 10.0) # Wait for the arm to stop moving
        self.stop_detection()
        rospy.loginfo("[linear_move_relative] Relative arm movement complete.")

        result.collided = self.coll_state.collided
        self.move_arm_act.set_succeeded(result)
        self.coll_state.collided = False

    def execute_setup(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement setup.")
        result = LinearMoveRelativeSetupResult()

        move_goal = self.transform_pose_for_wrist(goal, self.tool_approach_frame)
        self.pix3_pub.publish(move_goal)

        rospy.loginfo("[linear_move_relative] Waiting for arm to stop moving.")
        while not rospy.is_shutdown():
            if not self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                break
            rospy.sleep(0.05)
        rospy.loginfo("[linear_move_relative] The arm is not moving.")

        move_result = self.cm.move_arm_pose_biased(move_goal, self.JOINTS_BIAS, 
                                                   self.SETUP_VELOCITY, blocking = False,
                                                   init_angs=self.INIT_ANGS)
        print "setup_result", move_result

        if move_result == "no solution" or move_result is None:
            self.move_arm_setup_act.set_aborted(result)
            return

        # Wait for the arm to start moving 
        while not rospy.is_shutdown():
            if self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                break
            rospy.sleep(0.05)
        rospy.loginfo("[linear_move_relative] Arm has started moving.")

        self.start_detection("overhead_grasp", 0.99999)

        rospy.sleep(2)
        self.arm_moving_wait(True, 10.0) # Wait for the arm to stop moving
        self.stop_detection()
        rospy.loginfo("[linear_move_relative] Relative arm movement setup complete.")

        result.collided = self.coll_state.collided
        self.move_arm_setup_act.set_succeeded(result)
        self.coll_state.collided = False

    def test(self):
        wrist_pose = self.cm.get_current_wrist_pose_stamped("torso_lift_link")
        start_pose = copy.deepcopy(wrist_pose)
        end_pose = copy.deepcopy(wrist_pose)
        end_pose.pose.position.x += 0.1
        (trajectory, error_codes) = self.cm.ik_utilities.check_cartesian_path(start_pose,
                                              end_pose, current_angles, step_size, .1, math.pi/4, 
                                              collision_aware=0, joints_bias=self.JOINTS_BIAS, 
                                              bias_radius=self.BIAS_RADIUS, steps_before_abort=-1)
        

def main():
    rospy.init_node("linear_arm_move", anonymous=True)
    lmr = LinearMoveRelative()
    rospy.spin()

if __name__ == "__main__":
    main()
