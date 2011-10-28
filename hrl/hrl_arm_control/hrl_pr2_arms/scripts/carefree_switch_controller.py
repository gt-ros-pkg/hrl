#! /usr/bin/python

import sys

import roslib; roslib.load_manifest("hrl_pr2_arms")
import rospy

from hrl_pr2_arms.pr2_controller_switcher import ControllerSwitcher

def main():
    rospy.init_node("carefree_switch_controller", sys.argv)
    if len(sys.argv) <= 1:
        print "Usage: arm new_controller [param_file]"
        return
    cs = ControllerSwitcher()
    if len(sys.argv) >= 4:
        if len(sys.argv) >= 5:
            pkg = sys.argv[3]
            param_file = sys.argv[4]
        else:
            pkg = 'hrl_pr2_arms'
            param_file = sys.argv[3]
        param_path = "$(find %s)/params/%s" % (pkg, param_file)
        print cs.carefree_switch(sys.argv[1], sys.argv[2], param_path)
    else:
        print cs.carefree_switch(sys.argv[1], sys.argv[2])

if __name__ == "__main__":
    main()
