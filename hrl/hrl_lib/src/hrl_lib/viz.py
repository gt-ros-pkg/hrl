import roslib; roslib.load_manifest('hrl_lib')
import rospy
import visualization_msgs.msg as vm
import geometry_msgs.msg as gm
import std_msgs.msg as sdm
import numpy as np

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

##
# Create a visualization_msgs.Marker message given a point
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
def create_frame_marker(center, frame, line_len, frame_id):
    clist = []
    plist = []
    alpha = line_len
    for i in range(3):
        colors = np.matrix(np.zeros((4,2)))
        colors[i,:] = 1.0
        colors[3,:] = 1.0
        clist.append(colors)
        plist.append(np.column_stack([center, center+ alpha * frame[:,i]]))
    return viz.list_marker(np.column_stack(plist), np.column_stack(clist), [.01, 0, 0], 'line_list', frame_id)

