#! /usr/bin/python

import numpy as np
import copy

import roslib; roslib.load_manifest('hrl_arm_move_behaviors')
import rospy
import actionlib
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import std_srvs.srv
import tf.transformations as tf_trans
import tf

from hrl_pr2_lib.hrl_controller_manager import HRLControllerManager as ControllerManager
from pr2_collision_monitor.srv import StartMonitor, ArmMovingWait
from hrl_arm_move_behaviors.msg import LinearMoveRelativeGoal, LinearMoveRelativeAction
from hrl_arm_move_behaviors.msg import LinearMoveRelativeResult, LinearMoveRelativeSetupGoal
from hrl_arm_move_behaviors.msg import LinearMoveRelativeSetupAction, LinearMoveRelativeSetupResult
from hrl_arm_move_behaviors.epc_linear_move import EPCLinearMove
import hrl_arm_move_behaviors.util as util

class EPCLinearMoveSrv(object):
    def __init__(self):
        self.MOVE_VELOCITY = 0.005
        self.JOINTS_BIAS = [0.0, 0.012, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.INIT_ANGS = [-0.417, -0.280, -1.565, -2.078, -2.785, -1.195, -0.369]

        self.arm = rospy.get_param("~arm", default="r")
        self.tool_frame = rospy.get_param("~tool_frame", default="r_gripper_tool_frame")
        self.tool_approach_frame = rospy.get_param("~tool_approach_frame", default="")

        self.tf_listener = tf.TransformListener()
        self.lin_move = EPCLinearMove(self.arm, tool_frame=self.tool_frame, 
                                      tf_listener=self.tf_listener, rate=5)
        self.arm_num = self.arm == "l"

        rospy.loginfo("Waiting for start_monitoring")
        rospy.wait_for_service(self.arm + '_collision_monitor/start_monitoring')
        self.start_detection = rospy.ServiceProxy(self.arm + '_collision_monitor/start_monitoring', 
                                                  StartMonitor, persistent=False)
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
#self.lmr.cm.joint_action_client.cancel_all_goals()
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

    ##
    # Wait for the arm to start/stop moving 
    def wait_arm_moving(self, stop_wait=True):
        if stop_wait:
            rospy.loginfo("[linear_move_relative] Waiting for arm to stop moving.")
        else:
            rospy.loginfo("[linear_move_relative] Waiting for arm to start moving.")
        while not rospy.is_shutdown():
            if stop_wait:
                if not self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                    break
            else:
                if self.arm_moving_wait(False, 0.0).is_moving: # non-blocking
                    break
            rospy.sleep(0.05)
        if stop_wait:
            rospy.loginfo("[linear_move_relative] Arm has stopped moving.")
        else:
            rospy.loginfo("[linear_move_relative] Arm has started moving.")

    def execute_setup(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement setup.")
        result = LinearMoveRelativeSetupResult()

        self.lin_move.reset_controllers()

        # find setup poses
        torso_B_frame = self.lin_move.get_transform("torso_lift_link", 
                                                    goal.goal_pose.header.frame_id)
        frame_B_goal = util.pose_msg_to_mat(goal.goal_pose)
        appr_B_tool = self.lin_move.get_transform(self.tool_approach_frame, self.tool_frame) 
        torso_B_tool_appr = torso_B_frame * frame_B_goal * appr_B_tool
        appr_B_wrist = self.lin_move.get_transform(self.tool_approach_frame, 
                                                  self.arm + "_wrist_roll_link") 
        torso_B_wrist = torso_B_frame * frame_B_goal * appr_B_wrist

        # wait for arm to stop moving
        self.wait_arm_moving(stop_wait=True)

        # start collision detection
        self.start_detection(joint_detect=True, finger_detect=True, force_detect=False,
                             joint_behavior="overhead_grasp", joint_sig_level=0.99)

        # move to best estimate for IK for the wrist
        angles = self.lin_move.biased_IK(torso_B_wrist, self.INIT_ANGS, 
                                         self.JOINTS_BIAS, num_iters=6)
        if angles is None:
            self.move_arm_setup_act.set_aborted(result)
            return
        self.lin_move.command_joint_angles(angles, 3.0)

        # wait for the arm to start moving, then stop moving
        self.wait_arm_moving(stop_wait=False)
        rospy.sleep(1.0)
        self.wait_arm_moving(stop_wait=True)

        # use the epc control to accruately move to the setup position
        move_success = self.lin_move.move_to_pose(torso_B_tool_appr)

        # wait for the arm to start moving, then stop moving
        self.wait_arm_moving(stop_wait=False)
        rospy.sleep(1.0)
        self.wait_arm_moving(stop_wait=True)

        self.stop_detection()

        # return result message
        result.collided = self.coll_state.collided
        self.move_arm_setup_act.set_succeeded(result)
        self.coll_state.collided = False
        rospy.loginfo("[linear_move_relative] Relative arm movement setup complete.")
        return True

    def execute_move(self, goal):
        rospy.loginfo("[linear_move_relative] Execute relative arm movement.")
        result = LinearMoveRelativeResult()

        torso_B_frame = self.lin_move.get_transform("torso_lift_link", 
                                                    goal.goal_pose.header.frame_id)
        frame_B_goal = util.pose_msg_to_mat(goal.goal_pose)
        appr_B_tool = self.lin_move.get_transform(self.tool_approach_frame, self.tool_frame) 
        torso_B_tool_appr = torso_B_frame * frame_B_goal * appr_B_tool
        torso_B_tool_goal = torso_B_frame * frame_B_goal

        self.wait_arm_moving(stop_wait=True)

        move_success = self.lin_move.linear_trajectory(torso_B_tool_appr, torso_B_tool_goal, 
                                                       velocity=self.MOVE_VELOCITY)
        self.move_arm_act.set_succeeded(result)
        return
        
        if not move_success:
            rospy.loginfo("[linear_move_relative] Relative arm movement failed.")
            self.move_arm_act.set_aborted(move_success)
            return

        self.wait_arm_moving(stop_wait=False)

        self.start_detection("overhead_grasp", 1.0)

        rospy.sleep(2)
        self.wait_arm_moving(stop_wait=True)
        self.stop_detection()
        rospy.loginfo("[linear_move_relative] Relative arm movement complete.")

        result.collided = self.coll_state.collided
        self.move_arm_act.set_succeeded(result)
        self.coll_state.collided = False


def main():
    rospy.init_node("linear_arm_move", anonymous=True)
    lmr = EPCLinearMoveSrv()
    rospy.spin()

if __name__ == "__main__":
    main()
