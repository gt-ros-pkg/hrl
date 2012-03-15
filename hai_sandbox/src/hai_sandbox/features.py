import roslib; roslib.load_manifest('hai_sandbox')
import cv
import numpy as np
import scipy.spatial as sp
import math

class SURFMatcher:
    def __init__(self):
        self.model_images = {}
        self.model_fea = {}

    def add_file(self, model_name, label): 
        model_img = cv.LoadImage(model_name)
        self.add_model(model_img, label)

    def add_model(self, model_img, label):
        mgray = grayscale(model_img)
        m_loc, m_desc = surf(mgray)
        self.model_images[label] = model_img
        self.model_fea[label] = {'loc': m_loc, 'desc': m_desc}

    def build_db(self):
        fea_l = []
        labels_l = []
        locs_l = []
        for k in self.model_fea:
            fea_l.append(np.array(self.model_fea[k]['desc']))
            locs_l.append(np.array(self.model_fea[k]['loc']))
            labels_l.append(np.array([k for i in range(len(self.model_fea[k]['desc']))]))

        self.labels = np.row_stack(labels_l)
        self.locs = np.row_stack(locs_l)
        self.tree = sp.KDTree(np.row_stack(fea_l))

    def match(self, desc, thres=.6):
        dists, idxs = self.tree.query(np.array(desc), 2)
        ratio = dists[0] / dists[1]
        if ratio < threshold:
            desc = self.tree.data[idxs[0]]
            loc = self.locs[idxs[0]]
            return desc, loc
        else:
            return None

def concat_images(a, b):
    img_height = max(a.height, b.height)
    c = cv.CreateImage((a.width+b.width, img_height), a.depth, a.channels)
    a_area = cv.GetSubRect(c, (0,0, a.width, a.height))
    b_area = cv.GetSubRect(c, (a.width, 0, b.width, b.height))
    cv.Add(a, a_area, a_area)
    cv.Add(b, b_area, b_area)
    return c

def clone(something):
    if something.__class__ == cv.cvmat:
        return cv.CloneMat(something)
    else:
        return cv.CloneImage(something)

def draw_surf(image, keypoints, color):
    rimage = clone(image)

    for loc, lap, size, d, hess in keypoints:
        loc = tuple(np.array(np.round(loc), dtype='int').tolist())
        circ_rad = int(round(size/4.))
        cv.Circle(rimage, loc, circ_rad, color, 1, cv.CV_AA)
        cv.Circle(rimage, loc, 2, color, -1, cv.CV_AA)

        drad = math.radians(d)
        line_len = circ_rad
        loc_end = (np.matrix(np.round( circ_rad * np.matrix([np.cos(drad), np.sin(drad)]).T + np.matrix(loc).T), dtype='int')).A1.tolist()
        cv.Line(rimage, loc, tuple(loc_end), color, thickness=1, lineType=cv.CV_AA)

    return rimage

def draw_surf2(image, keypoints, colors):
    rimage = clone(image)
    for i, k in enumerate(keypoints):
        loc, lap, size, d, hess = k 
        loc = tuple(np.array(np.round(loc), dtype='int').tolist())
        c = tuple(np.matrix(colors[:,i],dtype='int').T.A1)
        color = (int(c[0]), int(c[1]), int(c[2]))
        #cv.Circle(rimage, loc, int(round(size/2.)), color, 1, cv.CV_AA)
        cv.Circle(rimage, loc, 5, color, 1, cv.CV_AA)
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
def surf(image_gray, params=(1, 3000,3,4)):
    surf_stor = cv.CreateMemStorage()
    surf_r = cv.ExtractSURF(image_gray, None, surf_stor, params)
    del surf_stor
    return surf_r
##
# @param image image
# @param params surf params
def surf_color(image, params=(1,3000,3,4)):
    gray = grayscale(image)
    return surf(gray, params)

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
