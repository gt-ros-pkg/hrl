import roslib; roslib.load_manifest('hai_sandbox')
from cv_bridge.cv_bridge import CvBridge, CvBridgeError
import cv
import sys
import hrl_lib.rutils as ru
import hai_sandbox.features as fea

forearm_cam_l = '/l_forearm_cam/image_rect_color'
ws_l = '/wide_stereo/left/image_rect_color'
ws_r = '/wide_stereo/right/image_rect_color'

fname = sys.argv[1]
bridge = CvBridge()

cv.NamedWindow('surf', 1)
cv.NamedWindow('harris', 1)
cv.NamedWindow('star', 1)

for topic, msg, t in ru.bag_iter(fname, [ws_l]):
    image = bridge.imgmsg_to_cv(msg, 'bgr8')
    image_gray = fea.grayscale(image)

    surf_keypoints, surf_descriptors = fea.surf(image_gray)
    cv.ShowImage('surf', fea.draw_surf(image, surf_keypoints, (255, 0, 0)))

    harris_keypoints = fea.harris(image_gray)
    cv.ShowImage('harris', fea.draw_harris(image, harris_keypoints, (0, 255, 0)))

    cv.WaitKey(10)
    
