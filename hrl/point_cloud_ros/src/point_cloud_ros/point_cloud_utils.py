import roslib; roslib.load_manifest('point_cloud_ros')
import robot_msgs.msg as rm
import numpy as np
#import psyco

## PointCloud  -> 3xN np matrix
# @param ros_pointcloud - robot_msgs/PointCloud
# @return 3xN np matrix
def ros_pointcloud_to_np(ros_pointcloud):
    ''' ros PointCloud.pts -> 3xN numpy matrix
    '''
    return ros_pts_to_np(ros_pointcloud.pts)

## list of Point32 points  -> 3xN np matrix
# @param ros_points - Point32[ ] (for e.g. from robot_msgs/PointCloud or Polygon3D)
# @return 3xN np matrix
def ros_pts_to_np(ros_pts):
    pts_list = []
    for p in ros_pts:
        pts_list.append([p.x,p.y,p.z])

    return np.matrix(pts_list).T

## 3xN np matrix -> ros PointCloud
# @param pts - 3xN np matrix
# @return PointCloud as defined in robot_msgs/msg/PointCloud.msg
def np_points_to_ros(pts):
    p_list = []
    chlist = []
    for p in pts.T:
        p_list.append(rm.Point32(p[0,0],p[0,1],p[0,2]))
        chlist.append(0.)

    ch = rm.ChannelFloat32('t',chlist)
    pc = rm.PointCloud(None,p_list,[ch])
    return pc


