# Calculating and displaying 2D Hue-Saturation histogram of a color image
import roslib
roslib.load_manifest('opencv2')
import sys
import cv

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
    cv.CalcBackProject([h_plane_img, s_plane_img], back_proj_img, hist)
    # cv.Dilate(back_proj_img, back_proj_img)
    # cv.Erode(back_proj_img, back_proj_img)
    cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_CLOSE, 1)
    cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_OPEN, 2)
#    cv.SubRS(back_proj_img, 255, back_proj_img)

    # cv.CalcBackProject([s_plane_img], back_proj_img, hist)
    # cv.Scale(back_proj_img, back_proj_img, 30000)
    
    return back_proj_img, hist
    #return , hist

if __name__ == '__main__':
    src = cv.LoadImageM(sys.argv[1])
    src2 = cv.LoadImageM(sys.argv[2])
    patch = cv.LoadImageM(sys.argv[3])

    cv.NamedWindow("Source", cv.CV_WINDOW_AUTOSIZE)
    cv.ShowImage("Source", src)

    back_proj_img, hist1 = back_project_hs(src, patch)
    back_proj_img2, hist2 = back_project_hs(src2, patch)

    cv.NamedWindow("back_projection", cv.CV_WINDOW_AUTOSIZE)
    cv.ShowImage("back_projection", back_proj_img)

    print cv.CompareHist(hist1, hist2, cv.CV_COMP_BHATTACHARYYA)
    cv.NamedWindow("back_projection2", cv.CV_WINDOW_AUTOSIZE)
    cv.ShowImage("back_projection2", back_proj_img2)
    
    cv.WaitKey(0)

    #making a mask
    mask = cv.CreateImage(cv.GetSize(back_proj_img2), 8, 1)

    cv.SubRS(back_proj_img2, 255, back_proj_img2)
    cv.SubRS(back_proj_img, 255, back_proj_img, mask=back_proj_img2)
    cv.SubRS(back_proj_img, 255, back_proj_img)

    cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_OPEN, 8)
    #cv.MorphologyEx(back_proj_img,back_proj_img, None, None, cv.CV_MOP_CLOSE, 8)
    cv.ShowImage("back_projection", back_proj_img)
    cv.WaitKey(0)

    cv.Scale(back_proj_img, back_proj_img, 1/255.0)
    print "here's the sum :", cv.Sum(back_proj_img)

