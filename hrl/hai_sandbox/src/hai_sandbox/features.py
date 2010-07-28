import roslib; roslib.load_manifest('hai_sandbox')
import cv
import numpy as np

def clone(something):
    if something.__class__ == cv.cvmat:
        return cv.CloneMat(something)
    else:
        return cv.CloneImage(something)

def draw_surf(image, keypoints, color):
    rimage = clone(image)
    for loc, lap, size, d, hess in keypoints:
        loc = tuple(np.array(np.round(loc), dtype='int').tolist())
        cv.Circle(rimage, loc, int(round(size/2.)), color, 1, cv.CV_AA)
        cv.Circle(rimage, loc, 2, color, -1, cv.CV_AA)
    return rimage

def draw_harris(image, keypoints, color):
    rimage = clone(image)
    for loc in keypoints:
        loc = tuple(np.array(np.round(loc), dtype='int').tolist())
        cv.Circle(rimage, loc, 5, color, 1, cv.CV_AA)
    return rimage

def draw_star(image, keypoints, color):
    rimage = clone(image)
    color_arr = np.array(color)
    max_resp = - 999999
    min_resp =   999999
    for _, _, response in keypoints:
        max_resp = max(response, max_resp)
        min_resp = min(response, min_resp)
    range_resp = max_resp - min_resp

    for loc, size, response in keypoints:
        loc = tuple(np.array(np.round(loc), dtype='int').tolist())
        color_weight = ((response - min_resp) / range_resp)
        c = tuple((color_weight * color_arr).tolist())
        cv.Circle(rimage, loc, int(round(size/2.0)), c, 1, cv.CV_AA)
    return rimage

#list of ((x,y), size, response)
def star(image):
    star_stor = cv.CreateMemStorage()
    star_keypoints = cv.GetStarKeypoints(image, star_stor) #list of ((x,y), size, response)
    del star_stor
    return star_keypoints

##
# surf_keypoints => keypoints (x,y), laplacian, size, direction , hessian
# surf_descriptors => list of len 128 lists
def surf(image_gray, params=(1,1500,3,4)):
    surf_stor = cv.CreateMemStorage()
    surf_r = cv.ExtractSURF(image_gray, None, surf_stor, params)
    del surf_stor
    return surf_r

##
# list of (x, y)
def harris(image_gray):
    eig_image = cv.CreateImage(cv.GetSize(image_gray), cv.IPL_DEPTH_32F, 1)
    temp_image = cv.CreateImage(cv.GetSize(image_gray), cv.IPL_DEPTH_32F, 1)
    return cv.GoodFeaturesToTrack(image_gray, eig_image, temp_image, 300, .1, 1.0, useHarris = True) #list of (x,y)

def grayscale(image):
    image_gray = cv.CreateImage(cv.GetSize(image), cv.IPL_DEPTH_8U,1)
    cv.CvtColor(image, image_gray, cv.CV_BGR2GRAY)
    return image_gray
