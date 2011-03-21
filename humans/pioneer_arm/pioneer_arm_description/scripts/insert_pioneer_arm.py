#!/usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#* Copyright (c) 2011, A.M.Howard, S.Williams
#* All rights reserved.
#* 
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*     * Redistributions of source code must retain the above copyright
#*       notice, this list of conditions and the following disclaimer.
#*     * Redistributions in binary form must reproduce the above copyright
#*       notice, this list of conditions and the following disclaimer in the
#*       documentation and/or other materials provided with the distribution.
#*     * Neither the name of the <organization> nor the
#*       names of its contributors may be used to endorse or promote products
#*       derived from this software without specific prior written permission.
#*  
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#* DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
#* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

##\author Stephen Williams
##\brief WRITE THIS

import roslib
roslib.load_manifest('pioneer_arm_description')
import rospy

import sys
import argparse
import subprocess
import tempfile
from xml.dom.minidom import parse, parseString


def xacro_function_evaluator(file, xacro=None, robot=None, pairs=None):
    
    # Generate an XML stub for xacro.py to execute
    xacro_xml  = '<?xml version="1.0" ?>\n'
    xacro_xml += '<robot '
    # insert robot name if provided
    if robot:
        xacro_xml += 'name="%s" ' % robot
    xacro_xml += 'xmlns:xacro="http://ros.org/wiki/xacro">\n'
    xacro_xml += '\t<include filename="%s" />\n' % file
    
    # Insert the xacro call, if provided
    if xacro:
        xacro_xml += '\t<xacro:%s ' % xacro
        # Loop through name-value pairs and insert as arguments
        for pair in pairs:
            xacro_xml += '%s="%s" ' % (pair[0], pair[1])
        xacro_xml += '/>\n'
    
    # Close root node
    xacro_xml += '</robot>\n'

    # Save the XML stub to a temporary file
    temp = tempfile.NamedTemporaryFile(mode='w+', suffix='.xml')
    temp.write(xacro_xml)
    temp.seek(0)
    
    # Execute xacro.py on temporary file
    p = subprocess.Popen(["rosrun", "xacro", "xacro.py", temp.name], stdout=subprocess.PIPE)#, stderr="/dev/null")
    output = p.communicate()[0]
    
    # Close the temporary file. This will cause a deletion
    temp.close()
    
    # write model XML back to the parameter server
    return output


def model_inserter(xml1, xml2):

    # Parse xml1 string as xml
    try:
        model1_dom = parseString(xml1)
    except:
        raise Exception("Unable to parse model1")
    
    # Parse xml2 string as xml
    try:
        model2_dom = parseString(xml2)
    except:
        raise Exception("Unable to parse model2")
    
    # Insert model2 into the end of model1
    node = model2_dom.documentElement.firstChild
    while node:
        if node.nodeType == node.ELEMENT_NODE:
            model1_dom.documentElement.appendChild(node.cloneNode(deep=True))
        node = node.nextSibling
    
    # Output the modified model1 DOM
    return model1_dom.toprettyxml(indent="  ")


if __name__ == "__main__":
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Insert Pioneer Arm model into an existing URDF/robot_description. The output will be streamed to stdout.')
    parser.add_argument('-f', '--file', required=True, help='xml/xacro file descriping the parent model')
    parser.add_argument('-l', '--link', required=True, help='link name in the parent model to attach the Pioneer Arm')
    parser.add_argument('-x', '--x', type=float, default=0.0, help='X-offset from link origin')
    parser.add_argument('-y', '--y', type=float, default=0.0, help='Y-offset from link origin')
    parser.add_argument('-z', '--z', type=float, default=0.0, help='Z-offset from link origin')
    parser.add_argument('-R', '--roll', type=float, default=0.0, help='Roll-offset from link origin')
    parser.add_argument('-P', '--pitch', type=float, default=0.0, help='Pitch-offset from link origin')
    parser.add_argument('-Y', '--yaw', type=float, default=0.0, help='Yaw-offset from link origin')
    
    args, unknown_args = parser.parse_known_args()
    
    # Load parent model
    parent_xml = xacro_function_evaluator(file=args.file, robot='robot')
    
    # Create Pioneer Arm urdf from supplied parameters
    arm_xml = xacro_function_evaluator(file='$(find pioneer_arm_description)/urdf/pioneer_arm.urdf.xacro', xacro='pioneer_arm_urdf', pairs=[['parent', args.link], ['robot_description', 'robot_description'], ['x', args.x], ['y', args.y], ['z', args.z], ['roll', args.roll], ['pitch', args.pitch], ['yaw', args.yaw]])

    # Insert Pioneer Arm into Parent Model
    robot_xml = model_inserter(parent_xml, arm_xml)
    
    # Output robot model    
    sys.stdout.write(robot_xml)
    