#! /usr/bin/python

import sys

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def main():
    prefix = "/".join(sys.argv[0].split("/")[:-2]) + "/params/"
    cs = ControllerSwitcher()
    print cs.switch("r_arm_controller", "r_arm_controller", prefix + "pr2_arm_controllers_low.yaml")

if __name__ == "__main__":
    main()
