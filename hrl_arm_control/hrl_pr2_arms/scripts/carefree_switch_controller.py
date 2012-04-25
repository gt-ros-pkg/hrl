#! /usr/bin/python

import sys

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def main():
    rospy.init_node("carefree_switch_controller", sys.argv)
    if len(sys.argv) <= 1 or sys.argv[1] in ["-h", "--help"]:
        print "Usage: arm new_controller '[param_file]'"
        return
    cs = ControllerSwitcher()
    if len(sys.argv) >= 4:
        param_path = sys.argv[3]
        if 'find' not in param_path:
            param_path = "$(find hrl_pr2_arms)/params/" + param_path
        print cs.carefree_switch(sys.argv[1], sys.argv[2], param_path)
    else:
        print cs.carefree_switch(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()
