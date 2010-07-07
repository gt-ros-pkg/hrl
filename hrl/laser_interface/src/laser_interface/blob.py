from pkg import *
import cv
import numpy as np
import hrl_opencv.adaptors as ad

def bound(value, min_val, max_val):
    return min(max(value, min_val), max_val)

class Rect:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def as_cv_rect(self):
        return (int(round(self.x)), int(round(self.y)), int(round(self.width)), int(round(self.height)))

    def top_left(self):
        return (int(round(self.x)), int(round(self.y)))

    def bottom_right(self):
        return (int(round(self.x+self.width)), int(round(self.y+self.height)))

    def keep_inside(self, minx, maxx, miny, maxy):
        x      = bound(self.x,       minx,  maxx)
        y      = bound(self.y,       miny,  maxy)
        mx     = bound(x+self.width, minx,  maxx)
        my     = bound(y+self.height, miny, maxy)
        return Rect(x,y, mx-x, my-y)

    def __repr__(self):
        return 'Rect: ' + str(self.x) + ' ' + str(self.y)+ ' ' + str(self.width)+ ' ' + str(self.height)

def remove_large_blobs(binary_image, max_area, max_dim=30):#, show=False):
    blob_statistics(binary_image, max_area, max_dim)#, show=True)
    return binary_image

##
#    WARNING: this function destructively modifies binary_image if max_area is set
#
def blob_statistics(binary_image, max_area=99999.0, max_dim=99999.0):#, show=False):
    statistics                = []
    storage                   = cv.CreateMemStorage(0)
    #FindContours(image,        storage, mode=CV_RETR_LIST, method=CV_CHAIN_APPROX_SIMPLE, offset=(0, 0))
    contours = cv.FindContours(binary_image, storage, cv.CV_RETR_TREE, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
    #number_contours, contours = cv.FindContours(binary_image, storage, cv.sizeof_CvContour, cv.CV_RETR_TREE, cv.CV_CHAIN_APPROX_SIMPLE, (0,0))
    #TODO: FIGURE OUT WHAT THE EQUIV OF SIZEOF IS IN OPENCV2
    #import pdb
    #pdb.set_trace()

    original_ptr = contours
    while contours != None:
        try:
            bx, by, bwidth, bheight = cv.BoundingRect(contours, 0)
            bounding_rect = Rect(bx, by, bwidth, bheight)
            moments = cv.Moments(contours, 0)
            #area = moments.m00
            #approximation to area since cvMoments' area seem broken
            area = bounding_rect.width * bounding_rect.height
            if False:
                #TODO NOT WORKING!!
                if moments.m00 == 0.0:
                    centroid = (bounding_rect.x, bounding_rect.y)
                else:
                    centroid = (moments.m10/moments.m00, moments.m01/moments.m00)
            else:
                if bwidth > 0:
            	    cx = bx + bwidth/2.
                else:
            	    cx = bx
    
                if bheight > 0:
            	    cy = by + bheight/2.
                else:
            	    cy = by
                centroid = (cx, cy)
                #if show: 
                #    print 'areas is', area, bounding_rect.width, bounding_rect.height
                if area > max_area or bounding_rect.width > max_dim or bounding_rect.height > max_dim:
                    cv.DrawContours(binary_image, contours, cv.Scalar(0), cv.Scalar(0), 0, cv.CV_FILLED) 
                else:
                    stats = {'area': area, 'centroid': centroid, 'rect': bounding_rect}
                    statistics.append(stats)
                contours = contours.h_next()
        except Exception, e:
            pass
            #This is due to OPENCV BUG and not being able to see inside contour object'
            break
    return statistics

def draw_blobs(frame, blobs, classification_window_width):
    if frame.nChannels == 1:
        color = cv.Scalar(200)
    else:
        color = cv.Scalar(255, 0, 0)

    for b in blobs:
        rect = blob_to_rect(b, classification_window_width)
        if rect != None:
            cv.Rectangle(frame, rect.top_left(), rect.bottom_right(), color, 1)

def blob_to_rect(blob, classification_window_width):
    x_center, y_center = blob['centroid']
    x_center           = int(x_center)
    y_center           = int(y_center)
    x_start            = x_center - classification_window_width
    y_start            = y_center - classification_window_width
    patch_size         = classification_window_width*2+1
    r                  = Rect(x_start, y_start, patch_size, patch_size).keep_inside(0, 639, 0,479) #TODO: remove this magic number
    if r.width < patch_size or r.height < patch_size:
        #print "classified_stream_to_classifier_matrix: not using connected comp", 
        #print blob['centroid'], 'w', r.width, 'h', r.height, 'preferred', patch_size
        return None
    else:
        return r

def blob_to_input_instance(image, blob, classification_window_width):
    patch_size = classification_window_width*2+1
    small_r    = blob_to_rect(blob, classification_window_width=classification_window_width)
    big_r      = blob_to_rect(blob, classification_window_width=classification_window_width*2)
    if big_r == None or small_r == None:
        return None
    small_patch        = cv.CloneMat(cv.GetSubRect(image, small_r.as_cv_rect()))
    big_patch          = cv.CloneMat(cv.GetSubRect(image, big_r.as_cv_rect()))
    #cv.ShowImage('patch', small_patch)
    #cv.ShowImage('big_patch', big_patch)
    big_patch_rescaled = cv.CreateImage((int(classification_window_width/2), int(classification_window_width/2)), 8, 3)
    cv.Resize(big_patch, big_patch_rescaled, cv.CV_INTER_LINEAR );

    np_patch_small   = np.asarray(small_patch)
    np_patch_big     = ad.cv2array(big_patch_rescaled)
    np_resized_small = np.matrix(np_patch_small.reshape(patch_size*patch_size*3, 1))
    np_resized_big   = np.matrix(np_patch_big.reshape(np_patch_big.shape[0] * np_patch_big.shape[1] * 3, 1))
    return np.concatenate((np_resized_small, np_resized_big), axis=0)
