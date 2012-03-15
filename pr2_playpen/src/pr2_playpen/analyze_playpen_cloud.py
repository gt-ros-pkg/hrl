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
        rospy.Subscriber("pr2_segment_region", PointCloud2, self.callback)
#        rospy.Subscriber("playpen_segment_object", PointCloud2, self.callback)
        self.check = rospy.Service("playpen_check_success", Check, self.serv_success)
        self.train_empty = rospy.Service("playpen_train", Train, self.serv_train)
        self.table_mean = None
        self.table_cov = None
        self.object_mean = None
        self.object_cov = None
        self.mean_ls =[]
        self.cov_ls = []



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
        while self.new_cloud == False:
            rospy.sleep(0.01)
        self.new_cloud = False
        self.lock.acquire()
        if self.table_mean == None or self.object_mean == None:
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
            mean_dist_table = (np.matrix(mean_3d).reshape(3,1)-self.table_mean)
            mean_dist_object = (np.matrix(mean_3d).reshape(3,1)-self.object_mean)
            mahal_dist_table = mean_dist_table.T*0.5*np.matrix(np.linalg.inv(cov_3d)+np.linalg.inv(self.table_cov))*mean_dist_table
            mahal_dist_object = mean_dist_object.T*0.5*np.matrix(np.linalg.inv(cov_3d)+np.linalg.inv(self.object_cov))*mean_dist_object
            print "table distance = ", mahal_dist_table
            print "object distance = ", mahal_dist_object
            # print "d = ", d
            # print "v = ", v
        #made a simple change so that I only look if table is empty or not
        #comparison is made from the user selected region with a base model
        #of mean covariance of the empty tabletop
        
        if np.max(f_np_array_cloud.shape)<200:
            result = "no_cloud"
        elif mahal_dist_table<mahal_dist_object:
            result = "table"
        else:
            result = "object"

        # if req.exp_state == 'empty':
        #     if np.max(f_np_array_cloud.shape)<200:
        #         result = "success"
        #     elif mahal_dist<5*self.nom_dist: 
        #         result = "success"
        #     else:
        #         result = "fail"
        # elif req.exp_state == 'object':
        #     if np.max(f_np_array_cloud.shape)<200:
        #         result = "fail"
        #     elif mahal_dist<5*self.nom_dist:
        #         result = "success"
        #     else:
        #         result = "fail"
        # elif req.exp_state == 'objects':
        #     print "multiple objects is not yet supported"

        self.lock.release()
        return CheckResponse(result)

    def serv_train(self, req):
        num_samples = 0
        if req.expected == 'table':
            self.table_mean = None
            self.table_cov = None
            print "training for empty table top"
        elif req.expected == 'object':
            self.object_mean = None
            self.object_cov = None
            print "training for object on table top"

        self.mean_ls = []
        self.cov_ls = []

        while num_samples < 11:
            start_time = rospy.get_time()
            while self.new_cloud == False:
                rospy.sleep(0.01)
            self.lock.acquire()
            np_array_cloud = np.array(self.cloud)
            f_ind = np.array(~np.isnan(np_array_cloud).any(1)).flatten()
            f_np_array_cloud = np_array_cloud[f_ind, :]

            if np.max(f_np_array_cloud.shape)>200:
                mean_3d = np.mean(f_np_array_cloud, axis = 0)
                cov_3d = np.cov(f_np_array_cloud.T)
                v, d = np.linalg.eig(cov_3d)
                max_var = d[:, v ==  np.max(v)]
                self.mean_ls.append(np.matrix(mean_3d).reshape(3,1))
                self.cov_ls.append(np.matrix(cov_3d))
                num_samples = num_samples + 1

            self.new_cloud = False
            print "still initializing"
            self.lock.release()
        buf_mean = np.matrix(np.zeros((3,1)))
        buf_cov = np.matrix(np.zeros((3,3)))

        print "here is the mean list :", self.mean_ls
        mean_arr = np.array(self.mean_ls)
        mean_arr.sort(axis=0)
        print "here is th sorted mean array  :", mean_arr

        print "here is the mean cov :\n", self.cov_ls
        cov_arr = np.array(self.cov_ls)
        cov_arr.sort(axis=0)
        print "here is the sorted cov :\n", cov_arr

        # for i in xrange(10):
        #     buf_mean = buf_mean + self.mean_ls[i]
        #     buf_cov = buf_cov + self.cov_ls[i]  #this is not exactly correct if populations
                                                    #have different # of points, but it should be approximately right
        if req.expected == 'table':
            self.table_mean = mean_arr[5]
            self.table_cov = cov_arr[5]
            # self.table_mean = buf_mean*1/10.0
            # self.table_cov = buf_cov*1/10.0
        elif req.expected == 'object':
            self.object_mean = mean_arr[5]
            self.object_cov = cov_arr[5]
            # self.object_mean = buf_mean*1/10.0
            # self.object_cov = buf_cov*1/10.0



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



# import roslib
# roslib.load_manifest('my_package')
# import sys
# import rospy
# import cv
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class image_converter:

#   def __init__(self):
#     self.image_pub = rospy.Publisher("image_topic_2",Image)

#     cv.NamedWindow("Image window", 1)
#     self.bridge = CvBridge()
#     self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

#   def callback(self,data):
#     try:
#       cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
#     except CvBridgeError, e:
#       print e

#     (cols,rows) = cv.GetSize(cv_image)
#     if cols > 60 and rows > 60 :
#       cv.Circle(cv_image, (50,50), 10, 255)

#     cv.ShowImage("Image window", cv_image)
#     cv.WaitKey(3)

#     try:
#       self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
#     except CvBridgeError, e:
#       print e

# def main(args):
#   ic = image_converter()
#   rospy.init_node('image_converter', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print "Shutting down"
#   cv.DestroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)
