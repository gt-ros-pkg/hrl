#! /usr/bin/python

import roslib
roslib.load_manifest("urdf_parser_python")
from urdf_parser_python.urdf_parser import *

model = create_model_from_file("test_robot.xml")
print model
