#! /usr/bin/python

import sys

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def main():
    rospy.init_node("switch_controller")
    prefix = "/".join(sys.argv[0].split("/")[:-2]) + "/params/"
    cs = ControllerSwitcher()
    if len(sys.argv) >= 4:
        print cs.switch(sys.argv[1], sys.argv[2], prefix + sys.argv[3])
    else:
        print cs.switch(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()
