__docformat__ = "restructuredtext en" 

import roslib; roslib.load_manifest('trf_learn') 
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField 
from tf import transformations 
from geometry_msgs.msg import Transform, Vector3, Quaternion 
 
def pointcloud2_to_array(cloud_msg, padding=0): 
    '''  
    Converts a rospy PointCloud2 message to a numpy recordarray  
     
    Assumes all fields 32 bit floats, and there is no padding. 
    ''' 
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields] 
    for i in range(padding):
        dtype_list.append((str(i), np.float32))
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list) 
    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 
 
def get_xyz_points(cloud_array, remove_nans=True): 
    ''' 
    Pulls out x, y, and z columns from the cloud recordarray, and returns a 3xN matrix. 
    ''' 
    # remove crap points 
    if remove_nans: 
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z']) 
        cloud_array = cloud_array[mask] 
     
    # pull out x, y, and z values 
    points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float) 
    points[...,0] = cloud_array['x'] 
    points[...,1] = cloud_array['y'] 
    points[...,2] = cloud_array['z'] 
 
    return points 
 
def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True): 
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans) 
 
def xyz_array_to_pointcloud2(points, stamp=None, frame_id=None): 
    ''' 
    Create a sensor_msgs.PointCloud2 from an array 
    of points. 
    ''' 
    msg = PointCloud2() 
    if stamp: 
        msg.header.stamp = stamp 
    if frame_id: 
        msg.header.frame_id = frame_id 
    if len(points.shape) == 3: 
        msg.height = points.shape[1] 
        msg.width = points.shape[0] 
    else: 
        msg.height = 1 
        msg.width = len(points) 
    msg.fields = [ 
        PointField('x', 0, PointField.FLOAT32, 1), 
        PointField('y', 4, PointField.FLOAT32, 1), 
        PointField('z', 8, PointField.FLOAT32, 1)] 
    msg.is_bigendian = False 
    msg.point_step = 12 
    msg.row_step = 12*points.shape[0] 
    msg.is_dense = int(np.isfinite(points).all()) 
    msg.data = np.asarray(points, np.float32).tostring() 
    return msg 
