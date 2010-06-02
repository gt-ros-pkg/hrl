import roslib
roslib.load_manifest('laser_interface')
import laser_interface.camera as cam
from   laser_interface.laser_detector import *
import rospy
import cv

stereo_camera = '/wide_stereo'
image_type = 'image_rect_color'
coi = 1

stereo = cam.ROSStereoListener([stereo_camera + '/left/' + image_type, stereo_camera + '/right/' + image_type])
sample_frame, r = stereo.next()
threshold = [100, 240]

splitter = SplitColors(sample_frame)
intensity_filter = BrightnessThreshold(sample_frame, thres_low=threshold[0], thres_high=threshold[1], tune=True)
#intensity_filtered = intensity_filter.threshold(coi)

cv.NamedWindow('low', 1)
cv.NamedWindow('high', 1)
cv.NamedWindow('combined', 1)
cv.NamedWindow('original', 1)

def low_thresh_func(val):
    threshold[0] = val
    intensity_filter.set_thresholds(threshold)
    print 'low', val

def high_thresh_func(val):
    threshold[1] = val
    intensity_filter.set_thresholds(threshold)
    print 'high', val

cv.CreateTrackbar('low_threshold', 'low', threshold[0], 255, low_thresh_func)
cv.CreateTrackbar('high_threshold', 'high', threshold[1], 255, high_thresh_func)

#for image, _ in stereo:
while not rospy.is_shutdown():
    image, _ = stereo.next()
    r, g, b = splitter.split(image)
    filtered_image = intensity_filter.threshold(g)
    cv.ShowImage('low', intensity_filter.thresholded_low)
    cv.ShowImage('high', intensity_filter.thresholded_high)
    #cv.ShowImage('original', g)
    cv.ShowImage('combined', filtered_image)
    cv.WaitKey(33)
    


