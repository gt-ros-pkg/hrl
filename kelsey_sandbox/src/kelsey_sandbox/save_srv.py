#! /usr/bin/python

import roslib; roslib.load_manifest('kelsey_sandbox')
import rospy
import roslib.message
import rosbag
import sys
import rosservice
from datetime import datetime
from hrl_lib.util import save_pickle
import yaml

#from hrl_lib import *
#from tabletop_object_detector.srv import TabletopDetection

if __name__ == "__main__":
    service_args = []
    for arg in sys.argv[2:]:
        if arg == '':
            arg = "''"
        service_args.append(yaml.load(arg))
    req, resp = rosservice.call_service(sys.argv[1], service_args) 
    a = datetime.now().isoformat('-')
    name = '-'.join(a.split(':')).split('.')[0] 
    save_pickle(resp, sys.argv[1].split('/')[-1] + name + ".pickle")

