from pkg import *
import cv
import laser_interface.blob as blob

class CombineMasks:
    def __init__(self, sample_image, channels=1):
        self.combined = cv.CreateImage(cv.GetSize(sample_image), 8 , channels)

    def combine(self, images):
        cv.Set(self.combined, 1)
        for img in images:
            cv.Mul(self.combined, img, self.combined)
        return self.combined

class Mask:
    def __init__(self, sample_image):
        self.thres_red_img         = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thres_green_img       = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thres_blue_img        = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.merged_frame          = cv.CreateImage(cv.GetSize(sample_image), 8 , 3)

    def mask(self, mask, r, g, b):
        cv.Mul(r, mask, self.thres_red_img)
        cv.Mul(g, mask, self.thres_green_img)
        cv.Mul(b, mask, self.thres_blue_img)
        cv.Merge(self.thres_blue_img, self.thres_green_img, self.thres_red_img, None, self.merged_frame);
        return self.merged_frame

class SplitColors:
    def __init__(self, sample_image):
        self.green_img             = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.red_img               = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.blue_img              = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)

    def split(self, image):
        cv.Split(image, self.blue_img, self.green_img, self.red_img, None);
        return (self.red_img, self.green_img, self.blue_img)

class BrightnessThreshold:
    def __init__(self, sample_image, max_area): #, tune=False):
        #self.thres_low  = thres_low
        #self.thres_high = thres_high
        self.max_area = max_area
        #self.set_thresholds([thres_low, thres_high])
        #self.csplit = SplitColors(sample_image)
        #if should_mask:
        #    self.mask   = Mask(sample_image)
        self.thresholded_low      = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thresholded_high     = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.thresholded_combined = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        #self.should_mask = should_mask
        #self.channel = channel
        #self.debug = False
        #self.tune = tune

        #if tune:
        #    cv.NamedWindow('low', 1)
        #    cv.NamedWindow('high', 1)
    #def set_thresholds(self, thresholds):
    #    self.thres_low = thresholds[0]
    #    self.thresh_high = thresholds[1]

    def get_thresholded_image(self):
        return self.thresholded_combined

    def threshold(self, thres_low, thres_high, thres_chan):
        result_val = 1 #Change result_val to 255 if need to view image

        cv.Threshold(thres_chan, self.thresholded_low, thres_low, result_val, cv.CV_THRESH_BINARY)
        cv.Dilate(self.thresholded_low, self.thresholded_low) #thresholded_low thresholded image using threshold for dark regions
        blob.remove_large_blobs(self.thresholded_low, self.max_area)

        cv.Threshold(thres_chan, self.thresholded_high, thres_high, result_val, cv.CV_THRESH_BINARY)
        cv.Dilate(self.thresholded_high, self.thresholded_high) #thresholded_high thresholded image using threshold for bright regions
        blob.remove_large_blobs(self.thresholded_high, self.max_area)#, show=True)

        cv.Or(self.thresholded_low, self.thresholded_high, self.thresholded_combined)
        return self.thresholded_combined

class MotionSubtract:
    def __init__(self, sample_image, max_area, adaptation_rate=0.8, threshold=10):
        self.max_area              = max_area
        self.accumulator           = cv.CreateImage(cv.GetSize(sample_image), 32, 1)
        cv.SetZero(self.accumulator)
        self.thresholded_img       = cv.CreateImage(cv.GetSize(sample_image), 8 , 1)
        self.difference_img        = cv.CreateImage(cv.GetSize(sample_image), 32 , 1)
        self.green32_img           = cv.CreateImage(cv.GetSize(sample_image), 32 , 1)
        self.adaptation_rate       = adaptation_rate
        self.threshold             = threshold

    def get_thresholded_image(self):
        return self.thresholded_img

    def subtract(self, thres_chan):
        cv.RunningAvg(thres_chan, self.accumulator, self.adaptation_rate)
        cv.CvtScale(thres_chan, self.green32_img)
        cv.Sub(self.green32_img, self.accumulator, self.difference_img)
        cv.Threshold(self.difference_img, self.thresholded_img, self.threshold, 1, cv.CV_THRESH_BINARY)
        cv.Dilate(self.thresholded_img, self.thresholded_img, iterations=1)
        blob.remove_large_blobs(self.thresholded_img, max_area = self.max_area)
        return self.thresholded_img

def make_visible_binary_image(img):
    cv.Threshold(img, img, 0, 255, cv.CV_THRESH_BINARY)

