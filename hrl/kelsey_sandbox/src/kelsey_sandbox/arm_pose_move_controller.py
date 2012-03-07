#! /usr/bin/python

import numpy as np
import cPickle as pickle

import roslib
roslib.load_manifest("rospy")
roslib.load_manifest("std_msgs")
roslib.load_manifest("hrl_pr2_arms")
import rospy
from std_msgs.msg import String
import tf.transformations as tf_trans
import roslib.substitution_args

from hrl_pr2_arms.pr2_arm import create_pr2_arm, PR2Arm, PR2ArmJointTrajectory, PR2ArmCartesianPostureBase
from arm_cart_vel_control import PR2ArmCartVelocityController
from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher
from srv import TrajectoryPlay, TrajectoryPlayResponse

RATE = 20
JOINT_TOLERANCES = [0.03, 0.1, 0.1, 0.1, 0.17, 0.15, 0.12]
JOINT_VELOCITY_WEIGHT = [3.0, 1.7, 1.7, 1.0, 1.0, 1.0, 0.5]

class ArmPoseMoveController(object):
    def __init__(self, ctrl_name, param_file):
        self.ctrl_name = ctrl_name
        self.param_file = param_file
        self.cur_joint_traj = None
        self.running = False
        self.ctrl_switcher = ControllerSwitcher()

    def load_arm(self, arm_char):
        self.ctrl_switcher.carefree_switch(arm_char, self.ctrl_name, self.param_file, reset=False)
        return create_pr2_arm(arm_char, PR2ArmJointTrajectory, 
                              controller_name=self.ctrl_name, timeout=8)


    def execute_trajectory(self, joint_trajectory, arm_char, rate, blocking=True):
        # TODO Replace this with behavior_manager locking mechanism
        #if self.running:
        #    rospy.logerr("[arm_pose_move_controller] Trajectory already in motion.")
        #    return False
        self.cur_arm = self.load_arm(arm_char)
        if self.cur_joint_traj is None:
            if not self.can_exec_traj(joint_trajectory):
                rospy.logwarn("[arm_pose_move_controller] Arm not at trajectory start.")
                return False
            self.cur_joint_traj = joint_trajectory
            self.cur_idx = 0
        r = rospy.Rate(rate)
        def exec_traj(te):
            self.stop_traj = False
            self.running = True
            rospy.loginfo("[arm_pose_move_controller] Starting trajectory.")
            while (not rospy.is_shutdown() and not self.stop_traj and 
                   self.cur_idx < len(self.cur_joint_traj)):
                self.cur_arm.set_ep(self.cur_joint_traj[self.cur_idx], 1./rate)
                self.cur_idx += 1
                r.sleep()
            self.cur_arm.set_ep(self.cur_arm.get_ep(), 0.3)
            self.running = False
            if self.cur_idx == len(self.cur_joint_traj):
                self.cur_joint_traj = None
            rospy.loginfo("[arm_pose_move_controller] Finished trajectory.")
        if blocking:
            exec_traj(None)
        else:
            self.exec_traj_timer = rospy.Timer(rospy.Duration(0.1), exec_traj, oneshot=True)
        return True

    def exec_traj_from_file(self, filename, rate_mult=0.8, reverse=False, blocking=True):
        f = file(roslib.substitution_args.resolve_args(filename), "r")
        traj, arm_char, rate = pickle.load(f)
        if reverse:
            traj.reverse()
        return self.execute_trajectory(traj, arm_char, rate * rate_mult, blocking)

    def move_to_setup_from_file(self, filename, velocity=0.1, rate=RATE, reverse=False, blocking=True):
        f = file(roslib.substitution_args.resolve_args(filename), "r")
        traj, arm_char, rate = pickle.load(f)
        if reverse:
            traj.reverse()
        return self.move_to_angles(traj[0], arm_char, velocity=velocity, rate=rate, blocking=blocking)

    def can_exec_traj(self, joint_trajectory):
        q_cur = self.cur_arm.get_ep()
        q_init = joint_trajectory[0]
        diff = self.cur_arm.diff_angles(q_cur, q_init)
        return np.all(np.fabs(diff) < JOINT_TOLERANCES)

    def can_exec_traj_from_file(self, filename):
        f = file(roslib.substitution_args.resolve_args(filename), "r")
        traj, rate = pickle.load(f)
        return self.can_exec_traj(traj)

    def is_moving(self):
        return self.running

    def pause_moving(self):
        self.stop_traj = True

    def stop_moving(self):
        self.stop_traj = True
        while not rospy.is_shutdown() and self.running:
            rospy.sleep(0.01)
        self.cur_joint_traj = None

    def move_to_angles(self, q_goal, arm_char, rate=RATE, velocity=0.1, blocking=True):
        print "HEY GURLS!", arm_char, q_goal
        self.cur_arm = self.load_arm(arm_char)
        q_cur = self.cur_arm.get_ep(True)
        diff = self.cur_arm.diff_angles(q_goal, q_cur)
        max_ang = np.max(np.fabs(diff) * JOINT_VELOCITY_WEIGHT)
        time_traj = max_ang / velocity
        steps = np.round(rate * time_traj)
        if steps == 0:
            return True
        t_vals = np.linspace(0., 1., steps)
        traj = [q_cur + diff * t for t in t_vals]
        return self.execute_trajectory(traj, arm_char, rate, blocking)

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
    def __init__(self, srv_name, ctrl_name, param_file):
        self.traj_ctrl = ArmPoseMoveController(ctrl_name, param_file)
        self.traj_srv = rospy.Service(srv_name, TrajectoryPlay, self.traj_play_cb)

    def traj_play_cb(self, req):
        if req.mode == req.MOVE_SETUP:
            result = self.traj_ctrl.move_to_setup_from_file(req.filepath, reverse=req.reverse,
                                         velocity=req.setup_velocity, blocking=req.blocking)
        elif req.mode == req.TRAJ_ONLY:
            result = self.traj_ctrl.exec_traj_from_file(req.filepath, rate_mult=req.traj_rate_mult,
                                         reverse=req.reverse, blocking=req.blocking)
        elif req.mode == req.SETUP_AND_TRAJ:
            result = self.traj_ctrl.move_to_setup_from_file(req.filepath, reverse=req.reverse,
                                         velocity=req.setup_velocity, blocking=True)
            if result:
                result = self.traj_ctrl.exec_traj_from_file(req.filepath, 
                                             rate_mult=req.traj_rate_mult, 
                                             reverse=req.reverse, blocking=req.blocking)
        else:
            rospy.logerr("[arm_pose_move_controller] Bad service call!")
        return TrajectoryPlayResponse(result)

CTRL_NAME_LOW = '%s_joint_controller_low'
PARAMS_FILE_LOW = '$(find hrl_pr2_arms)/params/joint_traj_params_electric_low.yaml'
CTRL_NAME_NONE = '%s_joint_controller_none'
PARAMS_FILE_NONE = '$(find hrl_pr2_arms)/params/joint_traj_params_electric_none.yaml'

def main():
    rospy.init_node("arm_pose_move_controller")

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
    p.add_option('-n', '--srv_name', dest="srv_name", default='/trajectory_playback',
                 help="Name of service to provide.")
    (opts, args) = p.parse_args()

    ctrl_switcher = ControllerSwitcher()
    if opts.right_arm:
        arm_char = 'r'
    else:
        arm_char = 'l'
    filename = opts.filename

    if opts.save_mode:
        assert(opts.right_arm + opts.left_arm == 1)
        if opts.traj_mode:
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
        traj_srv = TrajectoryServer(opts.srv_name, opts.ctrl_name, opts.params_file)
        rospy.spin()
        return
    elif opts.playback_mode:
        raw_input("Press enter to continue")
        arm_pm_ctrl = ArmPoseMoveController(opts.ctrl_name, PARAMS_FILE_LOW)
        result = arm_pm_ctrl.exec_traj_from_file(opts.filename, reverse=opts.reverse, blocking=True)
        print result

    if False:
        ctrl_switcher.carefree_switch(arm_char, opts.ctrl_name, PARAMS_FILE_LOW, reset=False)
        arm_pm_ctrl = ArmPoseMoveController(opts.ctrl_name, PARAMS_FILE_LOW)
        arm_pm_ctrl.move_to_angles(q_test, arm_char)
        return

if __name__ == "__main__":
    main()
