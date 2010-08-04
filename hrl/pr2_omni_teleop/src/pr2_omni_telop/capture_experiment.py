#! /usr/bin/python
import roslib; roslib.load_manifest('pr2_omni_teleop')
import hrl_camera.ros_camera as rc
import cv
import hrl_lib.rutils as ru
import hrl_lib.util as ut
import math


if __name__ == '__main__':
    import sys

    base_name = sys.argv[1]
    test = 'laser'
    ls = ru.LaserScanner('point_cloud_srv')
    prosilica = rc.Prosilica('prosilica', 'polled')
    points = ls.scan_np(math.radians(180.), math.radians(-180.), 20.)
    image = prosilica.get_frame()

    pkl_name = '%s.pkl' % base_name
    img_name = '%s.png' % filename
    ut.save_pickle(points,  pkl_name)
    cv.SaveImage(img_name, image)
    print 'Saved to %s and %s.' % (pkl_name, img_name)


