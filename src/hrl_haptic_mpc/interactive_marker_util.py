
#
# Code copied from basic_controls.py and then modified.
# Author: Marc Killpack


"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import math, numpy as np

import roslib; roslib.load_manifest('interactive_markers')
import rospy
roslib.load_manifest('hrl_lib')
from hrl_lib import transforms as tr

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl


# scale - float
# color - (r,g,b,a)
# mtype - 'cube', 'sphere'
def make_marker(scale, color, mtype):
    ss = 0.3
    marker = Marker()
    if mtype == 'cube':
        marker.type = Marker.CUBE
        ss = ss * 1/(math.pow(3, 1./3))
    elif mtype == 'sphere':
        marker.type = Marker.SPHERE
    elif mtype == 'cylinder':
        marker.type = Marker.CYLINDER        
    else:
        raise RuntimeError('Undefined marker type')

    marker.scale.x = ss * scale
    marker.scale.y = ss * scale
    marker.scale.z = ss * scale

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker

def make_3dof_marker_position(ps, scale, color, mtype):
    return make_marker_flexible(True, ps, scale, color, mtype,
                                ignore_rotation = True)

def make_marker_position_xy(ps, scale, color, mtype):
    return make_marker_flexible(True, ps, scale, color, mtype,
                                ignore_rotation=True, ignore_z=True)

def make_cody_ee_marker(ps, color, orientation = None,
                            marker_array=None, control=None, 
                            mesh_id_start = 0, ns = "cody_ee",
                            offset=0.08):
    mesh_id = mesh_id_start
    # this is the palm piece
    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]

    # stub_end_effector.dae
    # stub_end_effector_mini45.dae
    # tube_with_ati_collisions.dae
    # wedge_blender.dae
    mesh.mesh_resource = "package://cody/urdf/tube_with_ati_collisions.dae"
    mesh.type = Marker.MESH_RESOURCE

    rot_default = tr.Ry(np.radians(-90))*tr.Rz(np.radians(90))

    if orientation == None:
        orientation = [0, 0, 0, 1]

    rot_buf = tr.quaternion_to_matrix(orientation)
    orientation = tr.matrix_to_quaternion(rot_buf*rot_default)
    mesh.pose.orientation.x = orientation[0]
    mesh.pose.orientation.y = orientation[1]
    mesh.pose.orientation.z = orientation[2]
    mesh.pose.orientation.w = orientation[3]

    mesh.pose.position.z = offset
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.pose.position.x = ps.point.x
        mesh.pose.position.y = ps.point.y
        mesh.pose.position.z = ps.point.z
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        marker_array.markers.append(mesh)

    if control != None:
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        return control
    elif marker_array != None:
        return marker_array


def make_pr2_gripper_marker(ps, color, orientation = None,
                            marker_array=None, control=None, 
                            mesh_id_start = 0, ns = "pr2_gripper",
                            offset=-0.17):
    mesh_id = mesh_id_start
    # this is the palm piece
    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]
    mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/gripper_palm.dae"
    mesh.type = Marker.MESH_RESOURCE
    if orientation != None:
        mesh.pose.orientation.x = orientation[0]
        mesh.pose.orientation.y = orientation[1]
        mesh.pose.orientation.z = orientation[2]
        mesh.pose.orientation.w = orientation[3]
    else:
        mesh.pose.orientation.w = 1
    mesh.pose.position.x = offset
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.pose.position.x = ps.point.x
        mesh.pose.position.y = ps.point.y
        mesh.pose.position.z = ps.point.z
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        mesh_id = mesh_id+1
        marker_array.markers.append(mesh)

    # amount to open the gripper for each finger
    angle_open = 0.4
    if orientation == None:
        rot0 = np.matrix(np.eye(3))
    else:
        rot0 = tr.quaternion_to_matrix(orientation)

    if marker_array != None:
        T0 = tr.composeHomogeneousTransform(rot0, [ps.point.x, ps.point.y, ps.point.z])
    else:
        T0 = tr.composeHomogeneousTransform(rot0, [offset, 0.0, 0.])

    #transforming the left finger and finger tip
    rot1 = tr.rot_angle_direction(angle_open, np.matrix([0, 0, 1]))
    T1 = tr.composeHomogeneousTransform(rot1, [0.07691, 0.01, 0.])
    rot2 = tr.rot_angle_direction(-1*angle_open, np.matrix([0, 0, 1]))
    T2 = tr.composeHomogeneousTransform(rot2, [0.09137, 0.00495, 0.])
    
    T_proximal = T0*T1
    T_distal = T0*T1*T2

    finger_pos = tr.tft.translation_from_matrix(T_proximal)
    finger_rot = tr.tft.quaternion_from_matrix(T_proximal)

    tip_pos = tr.tft.translation_from_matrix(T_distal)
    tip_rot = tr.tft.quaternion_from_matrix(T_distal)

    #making the marker for the left finger
    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae"
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]
    mesh.type = Marker.MESH_RESOURCE
    mesh.pose.orientation.x = finger_rot[0] 
    mesh.pose.orientation.y = finger_rot[1] 
    mesh.pose.orientation.z = finger_rot[2] 
    mesh.pose.orientation.w = finger_rot[3]
    mesh.pose.position.x = finger_pos[0] 
    mesh.pose.position.y = finger_pos[1] 
    mesh.pose.position.z = finger_pos[2] 
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        mesh_id = mesh_id+1
        marker_array.markers.append(mesh)

    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]
    mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"
    mesh.type = Marker.MESH_RESOURCE
    mesh.pose.orientation.x = tip_rot[0] 
    mesh.pose.orientation.y = tip_rot[1] 
    mesh.pose.orientation.z = tip_rot[2] 
    mesh.pose.orientation.w = tip_rot[3]
    mesh.pose.position.x = tip_pos[0] 
    mesh.pose.position.y = tip_pos[1] 
    mesh.pose.position.z = tip_pos[2] 
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        mesh_id = mesh_id+1
        marker_array.markers.append(mesh)

    #transforming the right finger and finger tip
    rot1 = tr.rot_angle_direction(3.14, np.matrix([1, 0, 0]))*tr.rot_angle_direction(angle_open, np.matrix([0, 0, 1]))
    T1 = tr.composeHomogeneousTransform(rot1, [0.07691, -0.01, 0.])
    rot2 = tr.rot_angle_direction(-1*angle_open, np.matrix([0, 0, 1]))
    T2 = tr.composeHomogeneousTransform(rot2, [0.09137, 0.00495, 0.])
    
    T_proximal = T0*T1
    T_distal = T0*T1*T2

    finger_pos = tr.tft.translation_from_matrix(T_proximal)
    finger_rot = tr.tft.quaternion_from_matrix(T_proximal)

    tip_pos = tr.tft.translation_from_matrix(T_distal)
    tip_rot = tr.tft.quaternion_from_matrix(T_distal)

    #making the marker for the right finger
    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]
    mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger.dae"
    mesh.type = Marker.MESH_RESOURCE
    mesh.pose.orientation.x = finger_rot[0] 
    mesh.pose.orientation.y = finger_rot[1] 
    mesh.pose.orientation.z = finger_rot[2] 
    mesh.pose.orientation.w = finger_rot[3]
    mesh.pose.position.x = finger_pos[0] 
    mesh.pose.position.y = finger_pos[1] 
    mesh.pose.position.z = finger_pos[2] 
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        mesh_id = mesh_id+1
        marker_array.markers.append(mesh)

    mesh = Marker()
    mesh.ns = ns
    #mesh.mesh_use_embedded_materials = True
    mesh.scale.x = 1.0
    mesh.scale.y = 1.0
    mesh.scale.z = 1.0
    mesh.color.r = color[0]
    mesh.color.g = color[1]
    mesh.color.b = color[2]
    mesh.color.a = color[3]
    mesh.mesh_resource = "package://pr2_description/meshes/gripper_v0/l_finger_tip.dae"
    mesh.type = Marker.MESH_RESOURCE
    mesh.pose.orientation.x = tip_rot[0] 
    mesh.pose.orientation.y = tip_rot[1] 
    mesh.pose.orientation.z = tip_rot[2] 
    mesh.pose.orientation.w = tip_rot[3]
    mesh.pose.position.x = tip_pos[0] 
    mesh.pose.position.y = tip_pos[1] 
    mesh.pose.position.z = tip_pos[2] 
    if control != None:
        control.markers.append( mesh )
    elif marker_array != None:
        mesh.header.frame_id = ps.header.frame_id
        mesh.id = mesh_id
        mesh_id = mesh_id+1
        marker_array.markers.append(mesh)

    if control != None:
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        return control
    elif marker_array != None:
        return marker_array

def make_6dof_gripper(fixed, ps, scale, color, robot_type = "pr2",
                      ignore_rotation = False, ignore_x=False,
                      ignore_y=False, ignore_z=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = ps.header.frame_id
    int_marker.pose.position.x = ps.point.x
    int_marker.pose.position.y = ps.point.y
    int_marker.pose.position.z = ps.point.z
    int_marker.scale = scale

    int_marker.name = 'gripper_6dof'

    control =  InteractiveMarkerControl()
    control.always_visible = True
    if robot_type == "pr2":
        control = make_pr2_gripper_marker(ps, [0.3, 0.3, 0.3, 0.7], control=control) 
        int_marker.description = 'pr2_gripper_control'
    elif robot_type == "cody":
        control = make_cody_ee_marker(ps, [1, 1, 1, 0.4], control=control) 
        int_marker.description = 'cody_ee_control'
    int_marker.controls.append( control )

    if not ignore_x:
        control = InteractiveMarkerControl()
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_y:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_z:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_rotation:
        control = InteractiveMarkerControl()
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    return int_marker
    


# initial_position - 3x1 np matrix
# pose - geometry_msgs/PointStamped
# scale, color, mtype -- see make_marker.
def make_6dof_marker(fixed, ps, scale, color, mtype):
    return make_marker_flexible(fixed, ps, scale, color, mtype,
                                ignore_rotation = False)

def make_marker_flexible(fixed, ps, scale, color, mtype,
                         ignore_rotation, ignore_x=False,
                         ignore_y=False, ignore_z=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = ps.header.frame_id
    int_marker.pose.position.x = ps.point.x
    int_marker.pose.position.y = ps.point.y
    int_marker.pose.position.z = ps.point.z
    int_marker.scale = scale

    int_marker.name = 'simple_6dof'
    int_marker.description = ''

    # insert a marker
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_marker(scale, color, mtype))
    int_marker.controls.append(control)

    if fixed:
        int_marker.name += '_fixed'
        int_marker.description += '\n(fixed orientation)'

    if not ignore_x:
        control = InteractiveMarkerControl()
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_y:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_z:
        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    if not ignore_rotation:
        control = InteractiveMarkerControl()
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.orientation.w = 1
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.orientation.w = 1
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    return int_marker


# add this menu handler to the Interactive Marker.
# server - InteractiveMarkerServer
def add_menu_handler(int_marker, menu_handler, server):
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(control)
    menu_handler.apply(server, int_marker.name)
    server.applyChanges()



