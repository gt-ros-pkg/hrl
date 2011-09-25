#!/usr/bin/env python  
# Calculating and displaying 2D Hue-Saturation histogram of a color image
import roslib
roslib.load_manifest('pr2_playpen')
import rospy
from pr2_playpen.srv import * #this is for Train and Check
import sys
import cv
import numpy as np
import cPickle
from collections import deque
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class HistAnalyzer:

    def __init__(self, background_noise, mask, topic='/playpen_kinect/rgb/image_color'):
        rospy.Subscriber(topic, Image, self.get_img)
        self.check = rospy.Service("playpen_check_success_hist", Check, self.serv_success)
        self.train_empty = rospy.Service("playpen_train_hist", Train, self.serv_train)
        self.background_noise = background_noise
        self.h_bins = 30
        self.s_bins = 32
        self.h_ranges = [0, 180]
        self.s_ranges = [0, 255]
        self.ranges = [self.h_ranges, self.s_ranges]
        self.hist = None
        self.mask = mask
        self.avg_noise = None
        self.online_img = None
        self.bridge = CvBridge()
        size = cv.GetSize(background_noise[0])
        self.IavgF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.IdiffF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.IprevF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.IhiF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.IlowF = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ilow1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ilow2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ilow3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ihi1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ihi2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Ihi3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        cv.Zero(self.IavgF)
        cv.Zero(self.IdiffF)
        cv.Zero(self.IprevF)
        cv.Zero(self.IhiF)
        cv.Zero(self.IlowF)
        self.Icount = 0.00001
        self.Iscratch = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Iscratch2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Igray1 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Igray2 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Igray3 = cv.CreateImage(size, cv.IPL_DEPTH_32F, 1)
        self.Imaskt = cv.CreateImage(size, cv.IPL_DEPTH_8U, 1)
        cv.Zero(self.Iscratch)
        cv.Zero(self.Iscratch2)
        self.first = 1

    def accumulateBackground(self, img):
        cv.CvtScale(img, self.Iscratch, 1, 0)
        if (not self.first):
            cv.Acc(self.Iscratch, self.IavgF)
            cv.AbsDiff(self.Iscratch, self.IprevF, self.Iscratch2)
            cv.Acc(self.Iscratch2, self.IdiffF)
            self.Icount += 1.0
        self.first = 0
        cv.Copy(self.Iscratch, self.IprevF)

    def setHighThresh(self, thresh):
        cv.ConvertScale(self.IdiffF, self.Iscratch, thresh)
        cv.Add(self.Iscratch, self.IavgF, self.IhiF)
        #cv.Split(self.IhiF, self.Ihi1, self.Ihi2, self.Ihi3, None)

    def setLowThresh(self, thresh):
        cv.ConvertScale(self.IdiffF, self.Iscratch, thresh)
        cv.Sub(self.IavgF, self.Iscratch, self.IlowF)
        #cv.Split(self.IlowF, self.Ilow1, self.Ilow2, self.Ilow3, None)


    def createModelsfromStats(self):
        cv.ConvertScale(self.IavgF, self.IavgF, float(1.0/self.Icount))
        cv.ConvertScale(self.IdiffF, self.IdiffF, float(1.0/self.Icount))

        cv.AddS(self.IdiffF, cv.Scalar(1.0, 1.0, 1.0), self.IdiffF)
        self.setHighThresh(200.0)
        self.setLowThresh(200.0)

    def backgroundDiff(self, img, Imask):
        print "got into backgroundDiff"
        cv.CvtScale(img, self.Iscratch, 1, 0)
        #cv.Split(self.Iscratch, self.Igray1, self.Igray2, self.Igray3, None)
        #cv.InRange(self.Igray1, self.Ilow1, self.Ihi1, Imask)
        cv.InRange(self.Iscratch, self.IlowF, self.IhiF, Imask)

        # cv.InRange(self.Igray2, self.Ilow2, self.Ihi2, self.Imaskt)
        # cv.Or(Imask, self.Imaskt, Imask)

        # cv.InRange(self.Igray3, self.Ilow3, self.Ihi3, self.Imaskt)
        # cv.Or(Imask, self.Imaskt, Imask)

        cv.SubRS(Imask, 255, Imask)
        #cv.SaveImage('/home/mkillpack/Desktop/mask.png', Imask)
        #cv.Erode(Imask, Imask)
        return Imask


    def get_img(self, msg):
        try:
            self.online_img = self.bridge.imgmsg_to_cv(msg, "bgr8")
        except CvBridgeError, e:
            print e

    def serv_train(self, req):
        if req.expected== 'table':
            print 'great'
        num_samples = 0
        return TrainResponse(num_samples)


    def serv_success(self, req):
        result = "none"
        return CheckResponse(result)


    def calc_hist(self):
        self.hist = cv.CreateHist([self.h_bins, self.s_bins], cv.CV_HIST_ARRAY, self.ranges, 1)
        hsv = cv.CreateImage(cv.GetSize(self.background_noise[0]), 8, 3)
        h_plane = cv.CreateMat(self.background_noise[0].height, self.background_noise[0].width, cv.CV_8UC1)
        s_plane = cv.CreateMat(self.background_noise[0].height, self.background_noise[0].width, cv.CV_8UC1)
        for i in xrange(len(self.background_noise)):
            cv.CvtColor(self.background_noise[i], hsv, cv.CV_BGR2HSV)
            cv.Split(hsv, h_plane, s_plane, None, None)            
            planes = [h_plane, s_plane]#, s_plane, v_plane]
            cv.CalcHist([cv.GetImage(i) for i in planes], self.hist, True, self.mask)            
        #cv.NormalizeHist(self.hist, 10000.0)

    def check_for_hist(self):
        if self.hist == None:
            print "YOU CAN'T CALCULATE NOISE WITHOUT HIST MODEL OF TABLETOP"
            exit

    def calc_stats(self):
        cv.NamedWindow("noise", cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow("img1_back", cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow("img2_back", cv.CV_WINDOW_AUTOSIZE)
        self.check_for_hist()
        self.avg_noise = cv.CreateImage(cv.GetSize(self.background_noise[0]), 8, 1)
        cv.Zero(self.avg_noise)

        for i in xrange(len(self.background_noise)-1):
            cv.ShowImage("noise", self.avg_noise)
            back_proj_img1, hist1 = self.back_project_hs(self.background_noise[i])
            back_proj_img2, hist2 = self.back_project_hs(self.background_noise[i+1])

            self.accumulateBackground(back_proj_img1)

            cv.ShowImage("img1_back", back_proj_img1)
            cv.ShowImage("img2_back", back_proj_img2)
            scratch = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            scratch2 = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            
            # do something clever with ands ors and diffs 
            cv.Zero(scratch)
            cv.Zero(scratch2)
            cv.Sub(back_proj_img2, back_proj_img1, scratch2) #noise, but includes object if failed, 

            #cv.Sub(scratch2, self.avg_noise, scratch)            
            #cv.Or(self.avg_noise, scratch2, self.avg_noise)

            cv.Or(self.avg_noise, scratch2, self.avg_noise)

            cv.ShowImage("diff_back", scratch2)
            cv.ShowImage("diff_noise_scratch", scratch)

            cv.WaitKey(-1)
        self.createModelsfromStats()
        print self.Icount
        cv.NamedWindow("Ilow", cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow("Ihi", cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow("IavgF", cv.CV_WINDOW_AUTOSIZE)

        cv.ShowImage("Ihi", self.IhiF)
        cv.ShowImage("Ilow", self.IlowF)
        cv.ShowImage("IavgF", self.IavgF)

    def compare_imgs(self, img1, img2):
        back_proj_img, hist1 = self.back_project_hs(img1)
        back_proj_img2, hist2 = self.back_project_hs(img2)

        scratch = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
        scratch2 = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
        cv.Zero(scratch)
        cv.Zero(scratch2)

        #cv.Sub(back_proj_img, back_proj_img2, scratch2) #opposite noise, but excludes object 
        cv.Sub(back_proj_img2, back_proj_img, scratch2) #noise, but includes object if failed, 
        cv.Sub(scratch2, ha.avg_noise, scratch)
        
        return scratch
        

    def back_project_hs(self, img):
        self.check_for_hist()
        hsv = cv.CreateImage(cv.GetSize(img), 8, 3)
        scratch = cv.CreateImage(cv.GetSize(img), 8, 1)
        back_proj_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        cv.CvtColor(img, hsv, cv.CV_BGR2HSV)
        h_plane_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        s_plane_img = cv.CreateImage(cv.GetSize(img), 8, 1)
        cv.Split(hsv, h_plane_img, s_plane_img, None, None)            
        cv.CalcBackProject([h_plane_img, s_plane_img], back_proj_img, self.hist)
        #cv.Threshold(back_proj_img, back_proj_img, 200, 255, cv.CV_THRESH_BINARY)
        #cv.MorphologyEx(back_proj_img, back_proj_img, None, None, cv.CV_MOP_OPEN, 1)
        #cv.MorphologyEx(back_proj_img, back_proj_img, None, None, cv.CV_MOP_CLOSE, 2)    
        return back_proj_img, self.hist


if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()

    p.add_option('--batch', action='store', dest='batch_folder', type="string",
                 default=None, help='check for success or failure in batch mode with already stored data')
    p.add_option('--online', action='store', dest='topic', type="string", 
                 default=None, help='tries to run success or failure detection online using histograms and backprojection')

    opt, args = p.parse_args()


    if opt.batch_folder != None:
        folder = opt.batch_folder+'/background_noise/'
        background_noise = deque() #[]

        cv.NamedWindow("Source", cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow("final", cv.CV_WINDOW_AUTOSIZE)

        for i in xrange(130):
            try:
                background_noise.append(cv.LoadImage(folder+'file'+str(i).zfill(3)+'.png'))
            except:
                print "no file # ", i
        mask = cv.LoadImage(folder+'mask.png', 0)

        ha = HistAnalyzer(background_noise, mask)
        ha.calc_hist()
        ha.calc_stats()

        cv.ShowImage("Source", ha.avg_noise)
        cv.WaitKey(-1)

        back_sum_ls = deque() #[]

        for i in xrange(130):
            try:
                img = cv.LoadImage(folder+'file'+str(i).zfill(3)+'.png')
                result = ha.compare_imgs(img, ha.background_noise[0])
                back_sum_ls.append(float(cv.Sum(result)[0]))
            except:
                print "no training file # ", i

        avg = np.mean(back_sum_ls)
        std = np.std(back_sum_ls)

        print "avg and std are :", avg, std

        n = 0
        sum_val = 0

        for i in xrange(9):
            file_h = open(opt.batch_folder+'/object'+str(i).zfill(3)+'.pkl', 'r')
            res_dict = cPickle.load(file_h)
            for j in xrange(len(res_dict['success'])):
                try:
                    #check for object before starting ...##########################
                    img = cv.LoadImageM(opt.batch_folder+'/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_before_pr2.png')
                    result = ha.compare_imgs(img, ha.background_noise[-1])

                    Imask = cv.CreateImage(cv.GetSize(ha.background_noise[0]), cv.IPL_DEPTH_8U, 1)
                    cv.Zero(Imask)
                    cv.Zero(ha.Imaskt)
                    back_img, hist = ha.back_project_hs(img)
                    Imask = ha.backgroundDiff(back_img, Imask)

                    cv.MorphologyEx(Imask, Imask, None, None, cv.CV_MOP_OPEN, 1)
                    cv.MorphologyEx(Imask, Imask, None, None, cv.CV_MOP_CLOSE, 2)    

                    cv.ShowImage("final", Imask)
                    cv.WaitKey(-1)

                    is_object = False
                    loc_sum = float(cv.Sum(result)[0])
                    if loc_sum < avg+5*std:
                        res_dict['success'][j] = None
                        print "no object to start with!?"
                    else:
                        is_object = True
                        print "there's an object let's check for success ..."


                    #check for success if object was in workspace to start with
                    if is_object == True:
                        img = cv.LoadImageM(opt.batch_folder+'/object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_after_pr2.png')
                        result2 = ha.compare_imgs(img, ha.background_noise[-1])
                        n = n+1
                        sum_val = sum_val + float(cv.Sum(result2)[0])

                        #print "here's the sum :", cv.Sum(result2)[0]
                        cv.ShowImage("Source", img)
                        cv.ShowImage("final", result2)
                        Imask = cv.CreateImage(cv.GetSize(ha.background_noise[0]), cv.IPL_DEPTH_8U, 1)
                        cv.Zero(Imask)
                        cv.Zero(ha.Imaskt)
                        Imask = ha.backgroundDiff(ha.back_project_hs(img), Imask)

                        cv.ShowImage("final", Imask)
                        cv.WaitKey(-1)

                        loc_sum = float(cv.Sum(result2)[0])
                        if loc_sum < avg+5*std:
                            res_dict['success'][j] = True
                            print "success ! \t:", loc_sum, "\t compared to \t", avg, 5*std
                            #ha.background_noise.popleft()
                            #ha.background_noise.append(img)
                        else:
                            res_dict['success'][j] = False
                            print "epic fail ! \t:", loc_sum, "\t compared to \t", avg, 5*std

                except:
                    print "problem in try statement"
            file_h.close()
            try:
                os.system('rm '+opt.batch_folder+'/object'+str(i).zfill(3)+'.pkl')
            except:
                print "PROBLEM DELETING OLD FILE..., it didn't exist"
                exit
            file_h2 =  open(opt.batch_folder+'/object'+str(i).zfill(3)+'.pkl', 'w')
            cPickle.dump(res_dict, file_h2)
            file_h2.close()
            print "recalculating hist and noise..."
            #ha.calc_hist()
            #ha.calc_stats()
            print "done!"

        print "average error for objects present :", sum_val/n

    elif opt.topic != None:
        pass
        
