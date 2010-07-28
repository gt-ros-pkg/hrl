import roslib; roslib.load_manifest('hai_sandbox')
import cv
import sys
import hai_sandbox.features as fea

if __name__ == '__main__':
    fname = sys.argv[1]
    image = cv.LoadImage(fname)
    image_gray = cv.CreateImage((640,480), cv.IPL_DEPTH_8U,1)
    cv.CvtColor(image, image_gray, cv.CV_BGR2GRAY)

    star_keypoints = fea.star(image) 
    surf_keypoints, surf_descriptors = fea.surf(image_gray)
    harris_keypoints = fea.harris(image_gray)

    cv.NamedWindow('surf', 1)
    cv.NamedWindow('harris', 1)
    cv.NamedWindow('star', 1)
    while True:
        cv.ShowImage('surf', fea.draw_surf(image, surf_keypoints, (255, 0, 0)))
        cv.ShowImage('harris', fea.draw_harris(image, harris_keypoints, (0, 255, 0)))
        cv.ShowImage('star', fea.draw_star(image, star_keypoints, (0, 0, 255)))
        k = cv.WaitKey(33)
        if k == 27:
            break

    #Canny(image, edges, threshold1, threshold2, aperture_size=3) => None
