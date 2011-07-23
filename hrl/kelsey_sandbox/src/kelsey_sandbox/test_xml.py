#! /usr/bin/python

import roslib
roslib.load_manifest("kelsey_sandbox")
from urdf_interface import *

model = create_model_from_file("test_robot.xml")
print model
