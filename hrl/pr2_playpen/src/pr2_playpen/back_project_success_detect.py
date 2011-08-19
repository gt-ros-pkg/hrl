#!/usr/bin/env python  
# Calculating and displaying 2D Hue-Saturation histogram of a color image
import roslib
roslib.load_manifest('opencv2')
import sys
import cv

cv.NamedWindow("back_projection", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("back_modified", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("back_modified2", cv.CV_WINDOW_AUTOSIZE)

def hs_histogram(src, patch):
    # Convert to HSV
    hsv = cv.CreateImage(cv.GetSize(src), 8, 3)
    cv.CvtColor(src, hsv, cv.CV_BGR2HSV)
    hsv_patch= cv.CreateImage(cv.GetSize(patch), 8, 3)

    # Extract the H and S planes
    h_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)
    h_plane_img = cv.CreateImage(cv.GetSize(src), 8, 1)
    h_plane_patch = cv.CreateMat(patch.rows, patch.cols, cv.CV_8UC1)
    s_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)
    s_plane_img = cv.CreateImage(cv.GetSize(src), 8, 1)
    s_plane_patch = cv.CreateMat(patch.rows, patch.cols, cv.CV_8UC1)
    v_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)

    cv.Split(hsv, h_plane, s_plane, v_plane, None)
    cv.Split(hsv, h_plane_img, s_plane_img, None, None)
    cv.Split(hsv_patch, h_plane_patch, s_plane_patch, None, None)
    #cv.Split(src, h_plane, s_plane, v_plane, None)
    planes = [h_plane_patch, s_plane_patch]#, s_plane, v_plane]

    h_bins = 30
    s_bins = 32
    v_bins = 30
    hist_size = [h_bins, s_bins]
    # hue varies from 0 (~0 deg red) to 180 (~360 deg red again */
    h_ranges = [0, 180]
    # saturation varies from 0 (black-gray-white) to
    # 255 (pure spectrum color)
    s_ranges = [0, 255]
    v_ranges = [0, 255]
    ranges = [h_ranges, s_ranges]#, s_ranges, v_ranges]
    scale = 10
    hist = cv.CreateHist([h_bins, s_bins], cv.CV_HIST_ARRAY, ranges, 1)
    cv.CalcHist([cv.GetImage(i) for i in planes], hist)
    (_, max_value, _, _) = cv.GetMinMaxHistValue(hist)

    hist_img = cv.CreateImage((h_bins*scale, s_bins*scale), 8, 3)

    back_proj_img = cv.CreateImage(cv.GetSize(src), 8, 1)
    cv.CalcBackProject([h_plane_img, s_plane_img], back_proj_img, hist)
    
    # for h in range(h_bins):
    #     for s in range(s_bins):
    #         bin_val = cv.QueryHistValue_2D(hist, h, s)
    #         intensity = cv.Round(bin_val * 255 / max_value)
    #         cv.Rectangle(hist_img,
    #                      (h*scale, s*scale),
    #                      ((h+1)*scale - 1, (s+1)*scale - 1),
    #                      cv.RGB(intensity, intensity, intensity), 
    #                      cv.CV_FILLED)
    return back_proj_img, hist

def back_project_hs(src, patch):
    # Convert to HSV
    hsv = cv.CreateImage(cv.GetSize(src), 8, 3)
    cv.CvtColor(src, hsv, cv.CV_BGR2HSV)
    hsv_patch= cv.CreateImage(cv.GetSize(patch), 8, 3)
    cv.CvtColor(patch, hsv_patch, cv.CV_BGR2HSV)

    # Extract the H and S planes
    h_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)
    h_plane_img = cv.CreateImage(cv.GetSize(src), 8, 1)
    h_plane_patch = cv.CreateMat(patch.rows, patch.cols, cv.CV_8UC1)
    s_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)
    s_plane_img = cv.CreateImage(cv.GetSize(src), 8, 1)
    s_plane_patch = cv.CreateMat(patch.rows, patch.cols, cv.CV_8UC1)
    v_plane = cv.CreateMat(src.rows, src.cols, cv.CV_8UC1)

    cv.Split(hsv, h_plane, s_plane, v_plane, None)
    cv.Split(hsv, h_plane_img, s_plane_img, None, None)
    cv.Split(hsv_patch, h_plane_patch, s_plane_patch, None, None)
    #cv.Split(src, h_plane, s_plane, v_plane, None)
    planes = [h_plane_patch, s_plane_patch]#, s_plane, v_plane]
    # planes = [s_plane_patch]#, s_plane, v_plane]

    h_bins = 30
    s_bins = 32
    hist_size = [h_bins, s_bins]
    # hue varies from 0 (~0 deg red) to 180 (~360 deg red again */
    h_ranges = [0, 180]
    s_ranges = [0, 255]
    # saturation varies from 0 (black-gray-white) to
    # 255 (pure spectrum color)
    ranges = [h_ranges, s_ranges]#, s_ranges, v_ranges]
    #ranges = [s_ranges]#, s_ranges, v_ranges]
    scale = 1
    hist = cv.CreateHist([h_bins, s_bins], cv.CV_HIST_ARRAY, ranges, 1)
    #hist = cv.CreateHist([s_bins], cv.CV_HIST_ARRAY, ranges, 1)
    cv.CalcHist([cv.GetImage(i) for i in planes], hist)
    
    (min_value, max_value, _, _) = cv.GetMinMaxHistValue(hist)

    #cv.NormalizeHist(hist, 20*250.0)
    print "min hist value is :", min_value
    print "max hist value is :", max_value

    back_proj_img = cv.CreateImage(cv.GetSize(src), 8, 1)

    #cv.NormalizeHist(hist, 2000)

    cv.CalcBackProject([h_plane_img, s_plane_img], back_proj_img, hist)
    back_modified = cv.CreateImage(cv.GetSize(src), 8, 1)
    back_modified2 = cv.CreateImage(cv.GetSize(src), 8, 1)

    # cv.Dilate(back_proj_img, back_proj_img)
    # cv.Erode(back_proj_img, back_proj_img)
    #cv.Smooth(back_proj_img, back_modified)
    
    #cv.AdaptiveThreshold(back_proj_img, back_modified, 255, adaptive_method=cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C)
    #cv.Threshold(back_proj_img, back_modified, 250, 255, cv.CV_THRESH_BINARY)
    #cv.MorphologyEx(back_modified,back_modified2, None, None, cv.CV_MOP_CLOSE, 3)
    #cv.MorphologyEx(back_modified,back_modified2, None, None, cv.CV_MOP_CLOSE, 1)    
    # cv.MorphologyEx(back_proj_img,back_modified2, None, None, cv.CV_MOP_CLOSE, 1)
    #cv.MorphologyEx(back_modified2,back_modified2, None, None, cv.CV_MOP_OPEN, 2)    




    cv.MorphologyEx(back_proj_img,back_modified, None, None, cv.CV_MOP_OPEN, 1)
    cv.MorphologyEx(back_modified,back_modified, None, None, cv.CV_MOP_CLOSE, 2)    
    cv.Threshold(back_modified, back_modified, 250, 255, cv.CV_THRESH_BINARY)

    # cv.MorphologyEx(back_proj_img,back_modified2, None, None, cv.CV_MOP_CLOSE, 1)
    # cv.MorphologyEx(back_modified2,back_modified2, None, None, cv.CV_MOP_OPEN, 2)    




    #cv.FloodFill(back_modified, (320, 240), cv.Scalar(255), cv.Scalar(30), cv.Scalar(30), flags=8)

    # for i in xrange (10):
    #     cv.MorphologyEx(back_modified,back_modified, None, None, cv.CV_MOP_OPEN, 3)
    #     cv.MorphologyEx(back_modified,back_modified, None, None, cv.CV_MOP_CLOSE, 1)    

    
    #cv.SubRS(back_modified, 255, back_modified)

    # cv.CalcBackProject([s_plane_img], back_proj_img, hist)
    # cv.Scale(back_proj_img, back_proj_img, 30000)
    cv.ShowImage("back_projection", back_proj_img)
    cv.ShowImage("back_modified", back_modified)
    cv.ShowImage("back_modified2", back_modified2)

    cv.WaitKey(0)


    
    #return back_proj_img, hist
    return back_modified, hist
    #return , hist

if __name__ == '__main__':
    folder = sys.argv[1]
    
    cv.NamedWindow("Source", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("final", cv.CV_WINDOW_AUTOSIZE)
    src2 = cv.LoadImageM(folder+'object'+str(0).zfill(3)+'_try'+str(0).zfill(3)+'_after_pr2.png')
    patch_images = []

    avg_noise = cv.CreateImage(cv.GetSize(src2), 8, 1)
    cv.Zero(avg_noise)
    for k in xrange(1):
        patch_images.append(cv.LoadImageM('/home/mkillpack/Desktop/patch2.png'))
    #for i in [4]:
    for i in xrange(9):
        for j in xrange(100):
            print folder+'object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_after_pr2.png'
            src = cv.LoadImageM(folder+'object'+str(i).zfill(3)+'_try'+str(j).zfill(3)+'_after_pr2.png')
            cv.ShowImage("Source", src)

            back_proj_img, hist1 = back_project_hs(src, patch_images[0])
            back_proj_img2, hist2 = back_project_hs(src2, patch_images[0])
            scratch = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            scratch2 = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)
            
            # do something clever with ands ors and diffs 
            cv.Zero(scratch)
            cv.Zero(scratch2)


            #idea is to have a background model from back_proj_img2, or at least an emtpy single shot
            ###cv.Sub(back_proj_img, back_proj_img2, scratch)


            #cv.SubRS(back_proj_img, 255, scratch)
            ###cv.SubRS(back_proj_img2, 255, scratch2)
            #cv.Sub(back_proj_img, back_proj_img2, scratch2) #opposite noise, but excludes object 
            cv.Sub(back_proj_img2, back_proj_img, scratch2) #noise, but includes object if failed, 
                                                            #would need to learn before then update selectively 
                                                            #Maybe want both added in the end. 
            cv.Sub(scratch2, avg_noise, scratch)            
            cv.Or(avg_noise, scratch2, avg_noise)

            ##adding this part fills in wherever the object has been too, heatmaps?
            #cv.Sub(back_proj_img2, back_proj_img, scratch)
            #cv.Or(avg_noise, scratch, avg_noise)
            #


            #cv.Sub(back_proj_img2, avg_noise, back_proj_img2)
            #cv.Sub(scratch,, back_proj_img2)
            cv.ShowImage("final", scratch)
            #cv.Sub(scratch, avg_noise, scratch2)



            #cv.And(scratch, back_proj_img2, scratch2)
            #cv.SubRS(scratch2, 255, scratch)                
            #cv.ShowImage("final", back_proj_img)

            print cv.CompareHist(hist1, hist2, cv.CV_COMP_BHATTACHARYYA)

            #making a mask
            #mask = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)

            #cv.SubRS(back_proj_img2, 255, back_proj_img2)
            #cv.SubRS(back_proj_img, 255, back_proj_img, mask=back_proj_img2)
            #cv.SubRS(back_proj_img, 255, back_proj_img)

            #cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_OPEN, 8)
            #cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_CLOSE, 8)
            #cv.ShowImage("back_projection", back_proj_img2)
            #cv.WaitKey(0)

            cv.Scale(back_proj_img, back_proj_img, 1/255.0)
            print "here's the sum :", cv.Sum(scratch2)

