import roslib; roslib.load_manifest('hai_sandbox')
import rospy
import hrl_lib.util as ut
import sys

if __name__ == '__main__':
    p = ut.load_pickle(sys.argv[1])

