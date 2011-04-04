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
from pr2_playpen.srv import * #this is for Train and Check
import threading

class ResultsAnalyzer:

    def __init__(self):
        rospy.init_node('playpen_results')
        self.draw = ds.SceneDraw()
        self.cloud = None
#        rospy.Subscriber("playpen_segment_region", PointCloud2, self.callback)
        rospy.Subscriber("playpen_segment_object", PointCloud2, self.callback)
        self.check = rospy.Service("playpen_check_success", Check, self.serv_success)
        self.train = rospy.Service("playpen_train_success", Train, self.serv_train)
        self.nom_mean = None
        self.nom_cov = None
        self.nom_dist = None
        self.start_time = rospy.get_time()
        self.lock = threading.RLock()
        self.new_cloud = False
        self.start = 0

    def callback(self, data):
        self.lock.acquire()
        self.cloud = list(pc2py.points(data, ['x', 'y', 'z']))
        self.new_cloud = True
        print rospy.get_time()-self.start, "time in between call backs"
        self.start = rospy.get_time()
        self.lock.release()
#        self.get_cloud_stats()

    def serv_success(self, req):
        result = "none"
        self.lock.acquire()
        if self.nom_mean == None:
            print "you haven't initialized yet!!"
            return CheckResponse(result)
        np_array_cloud = np.array(self.cloud)
        f_ind = np.array(~np.isnan(np_array_cloud).any(1)).flatten()
        f_np_array_cloud = np_array_cloud[f_ind, :]
        if np.max(f_np_array_cloud.shape)>200:
            mean_3d = np.mean(f_np_array_cloud, axis = 0)
            cov_3d = np.cov(f_np_array_cloud.T)
            v, d = np.linalg.eig(cov_3d)
            max_var = d[:, v ==  np.max(v)]
            mean_dist = (np.matrix(mean_3d).reshape(3,1)-self.nom_mean)
            
            mahal_dist = mean_dist.T*0.5*np.matrix(np.linalg.inv(cov_3d)+np.linalg.inv(self.nom_cov))*mean_dist
            print "distance = ", mahal_dist
            print "nominal distance = ", self.nom_dist
        if req.exp_state == 'empty':
            if np.max(f_np_array_cloud.shape)<200:
                result = "success"
            elif mahal_dist<5*self.nom_dist: 
                result = "success"
            else:
                result = "fail"
        elif req.exp_state == 'object':
            if np.max(f_np_array_cloud.shape)<200:
                result = "fail"
            elif mahal_dist<5*self.nom_dist:
                result = "success"
            else:
                result = "fail"
        elif req.exp_state == 'objects':
            print "multiple objects is not yet supported"

        self.lock.release()
        return CheckResponse(result)

    def serv_train(self, req):
        num_samples = 0
        self.nom_mean = None
        self.nom_cov = None
        self.nom_dist = None
        while num_samples < 3:
            start_time = rospy.get_time()
            while self.new_cloud == False:
                rospy.sleep(0.05)
            print "time passed getting new cloud is :", rospy.get_time()-start_time
            self.lock.acquire()
            np_array_cloud = np.array(self.cloud)
            f_ind = np.array(~np.isnan(np_array_cloud).any(1)).flatten()
            f_np_array_cloud = np_array_cloud[f_ind, :]
            if np.max(f_np_array_cloud.shape)>200:
                mean_3d = np.mean(f_np_array_cloud, axis = 0)
                cov_3d = np.cov(f_np_array_cloud.T)
                v, d = np.linalg.eig(cov_3d)
                max_var = d[:, v ==  np.max(v)]
                if self.nom_mean == None:
                    self.nom_mean = np.matrix(mean_3d).reshape(3,1)
                    self.nom_cov = np.matrix(cov_3d)
                    self.nom_dist = 1

                else:
                    mean_dist = (np.matrix(mean_3d).reshape(3,1)-self.nom_mean)
                    mahal_dist = mean_dist.T*0.5*np.matrix(np.linalg.inv(cov_3d)+np.linalg.inv(self.nom_cov))*mean_dist
                    self.nom_mean = (self.nom_mean + np.matrix(mean_3d).reshape(3,1))*0.5
                    self.nom_cov = (np.matrix(cov_3d)+self.nom_cov)*0.5
                    self.nom_dist = (self.nom_dist + mahal_dist)*0.5

                num_samples = num_samples + 1
                
            self.new_cloud = False
            print "still initializing"
            self.lock.release()

        return TrainResponse(num_samples)

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

