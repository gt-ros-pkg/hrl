import roslib; roslib.load_manifest('point_cloud_ros')
import robot_msgs.msg as rm
import numpy as np
#import psyco

def ros_pts_to_np(ros_pts):
    ''' ros PointCloud.pts -> 3xN numpy matrix
    '''
    pts_list = []
    for p in ros_pts.pts:
        pts_list.append([p.x,p.y,p.z])

    return np.matrix(pts_list).T

def np_points_to_ros(pts):
    ''' 3xN np matrix -> ros PointCloud
    '''
    p_list = []
    chlist = []
    for p in pts.T:
        p_list.append(rm.Point32(p[0,0],p[0,1],p[0,2]))
        chlist.append(0.)

    ch = rm.ChannelFloat32('t',chlist)
    pc = rm.PointCloud(None,p_list,[ch])
    return pc
