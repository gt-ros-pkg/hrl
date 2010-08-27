import roslib; roslib.load_manifest('hai_sandbox')
import cv
import hai_sandbox.features as fea
import hrl_camera.ros_camera as rc
import rospy

#prosilica = rc.Prosilica('prosilica', 'streaming')
#prosilica = rc.ROSCamera('/narrow_stereo/right/image_rect')
prosilica = rc.ROSCamera('/wide_stereo/right/image_rect_color')
cv.NamedWindow('surf', 1)
while not rospy.is_shutdown():
    f = prosilica.get_frame()
    loc, desc = fea.surf_color(f, params=(0, 3000, 3, 4))
    fdrawn = fea.draw_surf(f, loc, (0, 255, 0))
    cv.ShowImage('surf', fdrawn)
    cv.WaitKey(33)

