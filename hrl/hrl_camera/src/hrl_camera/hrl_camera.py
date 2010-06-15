import camera_config as cc

def find_camera(name, id = None):
    parameters = cc.camera_parameters[name]
    if id == None:
        import camera_uuid as cu
        opencv_id = cu.lookup_by_name(name)
    else:
        opencv_id = id
    classname = parameters['class']
    import_statement = 'import ' + classname
    instantiation = classname + '.' + classname + '(parameters, opencv_id)'
    print 'import_statement:', import_statement
    print 'instantiation:', instantiation
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

