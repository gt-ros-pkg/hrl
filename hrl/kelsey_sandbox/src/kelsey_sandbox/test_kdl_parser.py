#! /usr/bin/python

import roslib
#roslib.load_manifest("kelsey_sandbox")
roslib.load_manifest("python_orocos_kdl")
import urdf_interface as urdf
import kdl_parser as kdlp

model = urdf.create_model_from_file("test_robot.xml")
tree = kdlp.tree_from_urdf_model(model)
print tree
