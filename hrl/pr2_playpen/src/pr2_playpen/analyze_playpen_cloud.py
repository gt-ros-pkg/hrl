#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_playpen')
import rospy
from sensor_msgs.msg import PointCloud2
import point_cloud_python as pc2py
import numpy as np
from matplotlib import pylab as pl
import draw_scene as ds
import math

class ResultsAnalyzer:

    def __init__(self):
        rospy.init_node('playpen_results')
        self.draw = ds.SceneDraw()
        self.cloud = None
#        rospy.Subscriber("playpen_segment_region", PointCloud2, self.callback)
        rospy.Subscriber("playpen_segment_object", PointCloud2, self.callback)
        self.nom_mean = np.matrix(np.zeros((3,1)))
        self.nom_cov = np.matrix(np.eye(3))
        self.start_time = rospy.get_time()

    def callback(self, data):
        self.cloud = list(pc2py.points(data, ['x', 'y', 'z']))
        self.get_cloud_stats()

    def run(self):
        print 'Ready to calculate results'
        rospy.spin()

    def get_cloud_stats(self):
        np_array_cloud = np.array(self.cloud)
        f_ind = np.array(~np.isnan(np_array_cloud).any(1)).flatten()
        f_np_array_cloud = np_array_cloud[f_ind, :]

        print 'size of remaining point cloud :', np.max(f_np_array_cloud.shape)
        if np.max(f_np_array_cloud.shape)>200:
            mean_3d = np.mean(f_np_array_cloud, axis = 0)
            cov_3d = np.cov(f_np_array_cloud.T)
            v, d = np.linalg.eig(cov_3d)
            max_var = d[:, v ==  np.max(v)]
            mean_dist = (np.matrix(mean_3d).reshape(3,1)-self.nom_mean)

            mahal_dist = mean_dist.T*0.5*np.matrix(np.linalg.inv(cov_3d)+np.linalg.inv(self.nom_cov))*mean_dist
            print "distance = ", mahal_dist
 
            if  rospy.get_time()-self.start_time < 10:
                self.nom_mean = np.matrix(mean_3d).reshape(3,1)
                self.nom_cov = np.matrix(cov_3d)
                print "still initializing"
            else:
                print "real distance now"
        else:
            print "not doing anything since point cloud is too small"
        # print "mean :\n", mean_3d
        # print "cov matrix :\n", cov_3d
        # print "eig of cov :\n", v
        # print d
        # print "max direction of var :\n", max_var
        # hs.draw.pub_body((0, 0, 0), (0, 0, 0, 1), 
    #                  (0.2, 0.2, 0.2), (1, 0,0,1), 1000000, hs.draw.Marker.SPHERE)
#        self.draw.pub_body((mean_3d[2], -1*mean_3d[0], -1*mean_3d[1]), (0, 0, 0, 1), 
#                     (0.2, 0.2, 0.2), (0, 1,0,1), 1000001, self.draw.Marker.SPHERE)

if __name__ == '__main__':
    ra = ResultsAnalyzer()
    ra.run()

