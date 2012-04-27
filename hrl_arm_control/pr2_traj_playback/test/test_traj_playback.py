#! /usr/bin/python

import roslib
roslib.load_manifest("pr2_traj_playback")
import rospy
import actionlib
from pr2_traj_playback.msg import TrajectoryPlayAction, TrajectoryPlayGoal

def main():
    rospy.init_node("test_traj_playback")
    tpg = TrajectoryPlayGoal()
    tpg.filepath = "$(find pr2_traj_playback)/test/test_traj.pkl"
    tpg.traj_rate_mult = 1.
    tpg.setup_velocity = 0.3
    tpg.reverse = False
    tpg.mode = tpg.MOVE_SETUP
    sac = actionlib.ActionClient("/trajectory_playback", TrajectoryPlayAction)
    sac.wait_for_server()
    sac.send_goal(tpg)
    print "Done."

if __name__ == "__main__":
    main()
