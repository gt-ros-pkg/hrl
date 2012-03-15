
import numpy as np

import roslib; roslib.load_manifest('hrl_lib')
import rospy
import visualization_msgs.msg as vm
import geometry_msgs.msg as gm
import std_msgs.msg as sdm

import hrl_lib.transforms as tr

##
# Creates a dictionary containing marker constants indexed by friendly names
def create_mdict():
    mdict = {}
    mdict['arrow']            = vm.Marker.ARROW
    mdict['cube']             = vm.Marker.CUBE
    mdict['sphere']           = vm.Marker.SPHERE
    mdict['cylinder']         = vm.Marker.CYLINDER
    mdict['line_strip']       = vm.Marker.LINE_STRIP
    mdict['line_list']        = vm.Marker.LINE_LIST
    mdict['cube_list']        = vm.Marker.CUBE_LIST
    mdict['sphere_list']      = vm.Marker.SPHERE_LIST
    mdict['points']           = vm.Marker.POINTS
    mdict['text_view_facing'] = vm.Marker.TEXT_VIEW_FACING
    mdict['mesh_resource']    = vm.Marker.MESH_RESOURCE
    mdict['triangle_list']    = vm.Marker.TRIANGLE_LIST
    return mdict


def text_marker(text, position, scale, color, frame, m_id=0):
    m = vm.Marker()
    m.header.frame_id = frame
    m.id = m_id
    m.type = create_mdict()['text_view_facing']

    m.pose.position.x = position[0,0]
    m.pose.position.y = position[1,0]              
    m.pose.position.z = position[2,0]              
    m.scale.z = scale
    m.color.r = color[0,0]
    m.color.g = color[1,0]
    m.color.b = color[2,0]
    m.color.a = color[3,0]

    return m

##
# Create a visualization_msgs.Marker message containing all the given points
def list_marker(points, colors, scale, mtype, mframe, duration=10.0, m_id=0):
    m = vm.Marker()
    m.header.frame_id = mframe
    m.id = m_id
    m.type = create_mdict()[mtype]
    m.action = vm.Marker.ADD
    m.points = [gm.Point(points[0,i], points[1,i], points[2,i]) for i in range(points.shape[1])]
    #pdb.set_trace()
    m.colors = [sdm.ColorRGBA(colors[0,i], colors[1,i], colors[2,i], colors[3,i]) for i in range(colors.shape[1])]

    m.color.r = 1.
    m.color.g = 0.
    m.color.b = 0.
    m.color.a = 1.
    m.scale.x = scale[0]
    m.scale.y = scale[1]
    m.scale.z = scale[2]

    m.lifetime = rospy.Duration(duration)

    return m


def text_marker(text, center, color, scale, mframe, 
        duration=10.0, m_id=0):

    m = vm.Marker()
    m.header.frame_id = mframe
    m.id = m_id
    m.type = create_mdict()['text_view_facing']
    m.action = vm.Marker.ADD
    m.text = text
    m.scale.z = scale
    m.pose.position.x = center[0,0]
    m.pose.position.y = center[1,0]
    m.pose.position.z = center[2,0]
    m.lifetime = rospy.Duration(duration)

    return m


def circle_marker(center, radius, scale, color, mframe, z=.03, duration=10.0, m_id=0, resolution=1.):
    angles = np.matrix(np.arange(0, 360., resolution)).T
    xs = center[0,0] + np.cos(angles) * radius
    ys = center[1,0] + np.sin(angles) * radius
    z = .03

    m = vm.Marker()
    m.header.frame_id = mframe
    m.id = m_id
    m.type = create_mdict()['line_strip']
    m.action = vm.Marker.ADD
    m.points = [gm.Point(xs[i,0], ys[i,0], z) for i in range(angles.shape[0])]
    m.colors = [sdm.ColorRGBA(color[0,0], color[1,0], color[2,0], color[3,0]) for i in range(angles.shape[0])]
    m.scale.x = scale
    m.lifetime = rospy.Duration(duration)
    return m

    

##
# Create a visualization_msgs.Marker message given a point
# @param point - 3x1 np matrix
# @param orientation - 4x1 np matrix (quaternion)
# @param mtype - string (see the function create_mdict)
def single_marker(point, orientation, mtype, mframe, scale=[.2,.2,.2], color=[1.0, 0, 0,.5], duration=10.0, m_id=0):
    m = vm.Marker()
    m.header.frame_id = mframe
    m.id = m_id
    m.type = create_mdict()[mtype]
    m.action = vm.Marker.ADD

    m.pose.position.x = point[0,0]
    m.pose.position.y = point[1,0]              
    m.pose.position.z = point[2,0]              
    m.pose.orientation.x = orientation[0,0]
    m.pose.orientation.y = orientation[1,0]
    m.pose.orientation.z = orientation[2,0]
    m.pose.orientation.w = orientation[3,0]

    m.scale.x = scale[0]
    m.scale.y = scale[1]
    m.scale.z = scale[2]
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = color[3]
    m.lifetime = rospy.Duration(duration)
    return m

##
# Creates a Marker message to display a coordinate frame
def create_frame_marker(center, frame, line_len, frame_id, m_id=0):
    clist = []
    plist = []
    alpha = line_len
    for i in range(3):
        colors = np.matrix(np.zeros((4,2)))
        colors[i,:] = 1.0
        colors[3,:] = 1.0
        clist.append(colors)
        plist.append(np.column_stack([center, center+ alpha * frame[:,i]]))
    return list_marker(np.column_stack(plist), np.column_stack(clist),
                       [.01, 0, 0], 'line_list', frame_id, m_id=m_id)


## make a quaternion from an arrow direction.
# can use this quaternion as the pose of the arrow marker to vizualise
# in rviz.
# @param direc - 3x1 np matrix.
# @return quaternion as a 4x1 np matrix
def arrow_direction_to_quat(direc):
    direc = direc / np.linalg.norm(direc)
    t = np.matrix([1.,0.,0.]).T
    if abs((t.T*direc)[0,0]) > 0.866:
        t = np.matrix([0.,1.,0.]).T

    v1 = direc
    v2 = t - (t.T*v1)[0,0] * v1
    v2 = v2 / np.linalg.norm(v2)
    v3 = np.matrix(np.cross(v1.A1, v2.A1)).T
    rot_mat = np.column_stack([v1, v2, v3])
    q = np.matrix(tr.matrix_to_quaternion(rot_mat)).T
    return q








