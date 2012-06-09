#! /usr/bin/python

import numpy as np
import cPickle as pickle
from threading import Lock
import copy

import roslib
roslib.load_manifest("pr2_traj_playback")
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import tf.transformations as tf_trans
import roslib.substitution_args
import actionlib

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2Arm, PR2ArmJointTrajectory, PR2ArmCartesianPostureBase
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from msg import TrajectoryPlayAction, TrajectoryPlayGoal
#from hrl_behavior_manager.behavior_manager import Behavior, BMPriorities

RATE = 20
JOINT_TOLERANCES = [0.03, 0.1, 0.1, 0.1, 0.17, 0.15, 0.12]
JOINT_VELOCITY_WEIGHT = [3.0, 1.7, 1.7, 1.0, 1.0, 1.0, 0.5]

##
# Allows one to move the arm through a set of joint angles safely and controller
# agnostic.  
#class ArmPoseMoveBehavior(Behavior):
class ArmPoseMoveBehavior(object):
    ##
    # @param arm_char 'r' or 'l' to choose which arm to load
    # @param ctrl_name name of the controller to load from the param_file 
    # @param param_file Location of the parameter file which specifies the controller
    #        (e.g. "$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml")
    def __init__(self, arm_char, ctrl_name="%s_arm_controller", param_file=None):
        #resource = arm_char + "_arm"
        #super(ArmPoseMoveBehavior, self).__init__(resource, BMPriorities.MEDIUM, 
        #                                          name=resource+"_pose_move")
        self.arm_char = arm_char
        self.ctrl_name = ctrl_name
        self.param_file = param_file
        self.cur_joint_traj = None
        self.running = False
        self.is_paused = False
        self.ctrl_switcher = ControllerSwitcher()

    ##
    # Switches controllers and returns an instance of the arm to control
    def load_arm(self):
        self.ctrl_switcher.carefree_switch(self.arm_char, self.ctrl_name, 
                                           self.param_file, reset=False)
        return create_pr2_arm(self.arm_char, PR2ArmJointTrajectory, 
                              controller_name=self.ctrl_name, timeout=8)

    ##
    # Plays back the specified trajectory.  The arm must currently be at the first
    # joint configuration specified in the joint trajectory within a certain degree of
    # tolerance.
    # @param joint_trajectory List of lists of length 7 representing joint angles to move through.
    # @param rate Frequency with which to iterate through the list.
    # @param blocking If True, the function will wait until done, if False, it will return
    #                 immediately
    def execute(self, joint_trajectory, rate, blocking=True):
        if self.running:
            rospy.logerr("[arm_pose_move_controller] Trajectory already in motion.")
            return False
        self.cur_arm = self.load_arm()
        if not self.can_exec_traj(joint_trajectory):
            rospy.logwarn("[arm_pose_move_controller] Arm not at trajectory start.")
            return False
        def exec_traj(te):
            self.cur_idx = 0
            self.stop_traj = False
            self.is_paused = False
            self.running = True
            rospy.loginfo("[arm_pose_move_controller] Starting trajectory.")
            r = rospy.Rate(rate)
            while (not rospy.is_shutdown() and not self.stop_traj and 
                   self.cur_idx < len(joint_trajectory)):
                self.cur_arm.set_ep(joint_trajectory[self.cur_idx], 1./rate)
                self.cur_idx += 1
                r.sleep()
            self.cur_arm.set_ep(self.cur_arm.get_ep(), 0.3)
            if self.cur_idx < len(joint_trajectory):
                self.cur_joint_traj = joint_trajectory[self.cur_idx:]
            else:
                self.cur_joint_traj = None
                self.is_paused = False
            self.last_rate = rate
            self.running = False
            rospy.loginfo("[arm_pose_move_controller] Finished trajectory.")
        if blocking:
            exec_traj(None)
        else:
            self.exec_traj_timer = rospy.Timer(rospy.Duration(0.1), exec_traj, oneshot=True)
        return True

    ##
    # Executes a linearly interpolated trajectory from the current joint angles to the
    # q_goal angles.
    # @param q_goal List of length 7 representing the desired end configuration.
    # @param rate Rate with which to execute trajectory.
    # @param velocity Speed (~rad/s) to move based on a heursitic which weighs relative
    #                 joint speeds.  The elbow will not move quicker than the velocity
    #                 in rad/s.
    # @param blocking If True, the function will wait until done, if False, it will return
    #                 immediately
    def move_to_angles(self, q_goal, rate=RATE, velocity=0.1, blocking=True):
        traj = self.get_angle_traj(q_goal, rate, velocity)
        if len(traj) == 0:
            return True
        return self.execute(traj, rate, blocking)

    def get_angle_traj(self, q_goal, rate=RATE, velocity=0.1):
        self.cur_arm = self.load_arm()
        q_cur = self.cur_arm.get_ep()
        diff = self.cur_arm.diff_angles(q_goal, q_cur)
        max_ang = np.max(np.fabs(diff) * JOINT_VELOCITY_WEIGHT)
        time_traj = max_ang / velocity
        steps = np.round(rate * time_traj)
        if steps == 0:
            return []
        t_vals = np.linspace(0., 1., steps)
        return [q_cur + diff * t for t in t_vals]

    ##
    # Determines whether or not the arm can execute the trajectory by checking the first
    # joint configuration and seeing whether or not it is within joint tolerances.
    # @param joint_trajectory List of lists of length 7 representing joint angles to move through.
    # @return True if the arm is at the beginning, False otherwise.
    def can_exec_traj(self, joint_trajectory):
        q_cur = self.cur_arm.get_ep()
        q_init = joint_trajectory[0]
        diff = self.cur_arm.diff_angles(q_cur, q_init)
        return np.all(np.fabs(diff) < JOINT_TOLERANCES)

    def is_moving(self):
        return self.running

    ##
    # Pauses the movement of the trajectory but doesn't reset its position in the array.
    def pause_moving(self):
        if self.is_moving():
            self.stop_traj = True
            while not rospy.is_shutdown() and self.running:
                rospy.sleep(0.01)
            self.is_paused = True
            return True
        return False

    ##
    # Restarts the currently running movement after being paused.
    def restart_moving(self, blocking=True):
        if not self.is_paused or self.cur_joint_traj is None:
            return False
        self.execute(self.cur_joint_traj, self.last_rate, blocking=blocking)
        return True

    ##
    # Stops the movement of the trajectory and resets the trajectory so it cannot restart.
    def stop_moving(self):
        self.stop_traj = True
        self.is_paused = False
        self.cur_joint_traj = None
        while not rospy.is_shutdown() and self.running:
            rospy.sleep(0.01)

    def preempt(self):
        self.stop_moving()

def load_arm_file(filename):
    try:
        f = file(roslib.substitution_args.resolve_args(filename), "r")
        traj, arm_char, rate = pickle.load(f)
        if arm_char not in ['r', 'l']:
            raise Exception("arm_char not r or l")
        return traj, arm_char, rate
    except Exception as e:
        print "Cannot open file."
        print "Error:", e
        return None, None, None

class TrajectoryLoader(object):
    def __init__(self, traj_ctrl_r, traj_ctrl_l):
        self.traj_ctrl_r, self.traj_ctrl_l = traj_ctrl_r, traj_ctrl_l

    def exec_traj_from_file(self, filename, rate_mult=0.8, 
                            reverse=False, blocking=True):
        traj, arm_char, rate = load_arm_file(filename)
        if traj is None:
            return None
        if reverse:
            traj.reverse()
        traj_ctrl = self.traj_ctrl_r if arm_char == 'r' else self.traj_ctrl_l
        traj_ctrl.execute(traj, rate * rate_mult, blocking)

    def move_to_setup_from_file(self, filename, velocity=0.1, rate=RATE, 
                                reverse=False, blocking=True):
        traj, arm_char, rate = load_arm_file(filename)
        if traj is None:
            return None
        if reverse:
            traj.reverse()
        traj_ctrl = self.traj_ctrl_r if arm_char == 'r' else self.traj_ctrl_l
        traj_ctrl.move_to_angles(traj[0], velocity=velocity, 
                                 rate=rate, blocking=blocking)

    def can_exec_traj_from_file(self, filename):
        traj, arm_char, rate = load_arm_file(filename)
        if traj is None:
            return None
        traj_ctrl = self.traj_ctrl_r if arm_char == 'r' else self.traj_ctrl_l
        traj_ctrl.can_exec_traj(traj)

class TrajectorySaver(object):
    def __init__(self, rate):
        self.rate = rate

    def record_trajectory(self, arm_char, blocking=True):
        self.traj = []
        self.stop_recording = False
        self.is_recording = True
        self.cur_arm = create_pr2_arm(arm_char, PR2Arm, timeout=3)
        def rec_traj(te):
            rospy.loginfo("[arm_pose_move_controller] Recording trajectory.")
            r = rospy.Rate(self.rate)
            while not rospy.is_shutdown() and not self.stop_recording:
                q = self.cur_arm.get_joint_angles().copy()
                self.traj.append(q)
                r.sleep()
            rospy.loginfo("[arm_pose_move_controller] Stopped recording trajectory.")
            self.is_recording = False
        if blocking:
            rec_traj(None)
        else:
            self.rec_traj_timer = rospy.Timer(rospy.Duration(0.1), rec_traj, oneshot=True)

    def stop_record(self, save_file):
        self.stop_recording = True
        while not rospy.is_shutdown() and self.is_recording:
            rospy.sleep(0.1)
        f = file(save_file, "w")
        pickle.dump((self.traj, self.cur_arm.arm, self.rate), f)
        f.close()

class TrajectoryServer(object):
    def __init__(self, arm_char, as_name, ctrl_name, param_file):
        self.traj_ctrl = ArmPoseMoveBehavior(arm_char, ctrl_name, param_file)

        def pause_cb(req):
            if not self.traj_ctrl.is_paused:
                if self.traj_ctrl.pause_moving():
                    rospy.loginfo("[arm_pose_move_controller] Pausing %s arm." 
                                                          % self.traj_ctrl.arm_char)
            else:
                self.traj_ctrl.restart_moving(blocking=False)
            return EmptyResponse()
        self.traj_pause_srv = rospy.Service(as_name + "_pause", Empty, pause_cb)

        def stop_cb(req):
            self.traj_ctrl.stop_moving()
            return EmptyResponse()
        self.traj_stop_srv = rospy.Service(as_name + "_stop", Empty, stop_cb)

        self.traj_srv = actionlib.SimpleActionServer(as_name, TrajectoryPlayAction, 
                                                     self.traj_play_cb, False)
        self.traj_srv.register_preempt_callback(self.traj_cancel_cb)
        self.traj_srv.start()
        self.last_goal = None

    def same_goal_as_last(self, new_goal):
        if self.last_goal is None:
            self.last_goal = copy.copy(new_goal)
            return False
        ret_val = (new_goal.filepath == self.last_goal.filepath and
                   new_goal.reverse == self.last_goal.reverse and
                   new_goal.mode == self.last_goal.mode)
        self.last_goal = copy.copy(new_goal)
        return ret_val

    def traj_play_cb(self, goal):
        rospy.loginfo("[arm_pose_move_controller] Playing trajectory on %s arm." 
                                                       % self.traj_ctrl.arm_char)
        traj, arm_char, rate = load_arm_file(goal.filepath)
        if traj is None:
            self.traj_srv.set_aborted(text="Failed to open file.")
            return
        if arm_char != self.traj_ctrl.arm_char:
            self.traj_srv.set_aborted(text="File contains wrong arm.")
            return
        if goal.reverse:
            traj.reverse()
        if self.same_goal_as_last(goal) and self.traj_ctrl.is_paused:
            self.traj_ctrl.restart_moving(blocking=True)
        else:
            if goal.mode == goal.MOVE_SETUP or goal.mode == goal.SETUP_AND_TRAJ:
                setup_traj = self.traj_ctrl.get_angle_traj(traj[0], velocity=goal.setup_velocity, 
                                                           rate=RATE)
                if goal.mode == goal.MOVE_SETUP:
                    full_traj = setup_traj
                else:
                    full_traj = setup_traj + traj
            elif goal.mode == goal.TRAJ_ONLY:
                full_traj = traj
            else:
                self.traj_srv.set_aborted(text="Unknown goal mode.")
                return
            self.traj_ctrl.execute(full_traj, rate * goal.traj_rate_mult, blocking=True)
        self.traj_srv.set_succeeded()

    def traj_cancel_cb(self):
        self.traj_ctrl.stop_moving()

CTRL_NAME_LOW = '%s_joint_controller_low'
PARAMS_FILE_LOW = '$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml'
CTRL_NAME_NONE = '%s_joint_controller_none'
PARAMS_FILE_NONE = '$(find hrl_pr2_arms)/params/joint_traj_params_electric_none.yaml'

def main():

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-f', '--filename', dest="filename", default="",
                 help="File to save trajectory to or load from.")
    p.add_option('-l', '--left_arm', dest="left_arm",
                 action="store_true", default=False,
                 help="Use left arm.")
    p.add_option('-r', '--right_arm', dest="right_arm",
                 action="store_true", default=False,
                 help="Use right arm.")
    p.add_option('-s', '--save_mode', dest="save_mode",
                 action="store_true", default=False,
                 help="Saving mode.")
    p.add_option('-t', '--traj_mode', dest="traj_mode",
                 action="store_true", default=False,
                 help="Trajectory mode.")
    p.add_option('-c', '--ctrl_name', dest="ctrl_name", default=CTRL_NAME_LOW,
                 help="Controller to run the playback with.")
    p.add_option('-p', '--params', dest="params_file", default=PARAMS_FILE_LOW,
                 help="YAML file to save parameters in or load from.")
    p.add_option('-v', '--srv_mode', dest="srv_mode", 
                 action="store_true", default=False,
                 help="Server mode.")
    p.add_option('-y', '--playback_mode', dest="playback_mode", 
                 action="store_true", default=False,
                 help="Plays back trajectory immediately.")
    p.add_option('-z', '--reverse', dest="reverse", 
                 action="store_true", default=False,
                 help="Plays back trajectory in reverse.")
    (opts, args) = p.parse_args()

    if opts.right_arm:
        arm_char = 'r'
    elif opts.left_arm:
        arm_char = 'l'
    else:
        print "Must select arm with -r or -l."
        return
    filename = opts.filename

    rospy.init_node("arm_pose_move_controller_%s" % arm_char)
    if opts.save_mode:
        assert(opts.right_arm + opts.left_arm == 1)
        if opts.traj_mode:
            ctrl_switcher = ControllerSwitcher()
            ctrl_switcher.carefree_switch(arm_char, CTRL_NAME_NONE, PARAMS_FILE_NONE, reset=False)
            traj_saver = TrajectorySaver(RATE)
            raw_input("Press enter to start recording")
            traj_saver.record_trajectory(arm_char, blocking=False)
            
            import curses
            def wait_for_key(stdscr):
                curses.init_pair(1, curses.COLOR_RED, curses.COLOR_WHITE)
                stdscr.addstr(0,0, "Press any key to stop!", curses.color_pair(1) )
                stdscr.refresh()
                c = 0
                while not rospy.is_shutdown() and c == 0:
                    c = stdscr.getch()
            curses.wrapper(wait_for_key)

            traj_saver.stop_record(roslib.substitution_args.resolve_args(opts.filename))
            ctrl_switcher.carefree_switch(arm_char, opts.ctrl_name, PARAMS_FILE_LOW, reset=False)
            return
        else:
            print "FIX"
            return
    elif opts.srv_mode:
        traj_srv = TrajectoryServer(arm_char, "/trajectory_playback_" + arm_char, 
                                    opts.ctrl_name, opts.params_file)
        rospy.spin()
        return
    elif opts.playback_mode:
        raw_input("Press enter to continue")
        # TODO FIX
        traj_load = TrajectoryLoader(opts.ctrl_name, opts.params_file)
        result = traj_load.exec_traj_from_file(opts.filename, reverse=opts.reverse, blocking=True)
        print result

if __name__ == "__main__":
    main()
