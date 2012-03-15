import camera_config as cc
import camera_uuid as cu

def find_camera(name):
    parameters = cc.camera_parameters[name]
    opencv_id = cu.lookup_by_name(name)
    classname = parameters['class']
    import_statement = 'import ' + classname
    instantiation = classname + '.' + classname + '(parameters, opencv_id)'
    exec import_statement
    return eval(instantiation)


if __name__ == '__main__':
    import sys
    import roslib
    roslib.load_manifest('hrl_camera')
    import cv
    name = sys.argv[1]
    cv.NamedWindow(name, cv.CV_WINDOW_AUTOSIZE)
    c = find_camera(name)
    while True:
        f = c.get_frame()
        cv.ShowImage(name, f)
        cv.WaitKey(33)

