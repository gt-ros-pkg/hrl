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


#if __name__ == '__main__':
#    import sys
#    import roslib
#    roslib.load_manifest('hrl_camera')
#    import cv
#    import time
#    #name = sys.argv[1]
#    names = ['snozzberry_hand', 'ele_carriage']
#    
#    [cv.NamedWindow(name, cv.CV_WINDOW_AUTOSIZE) for name in names]
#    cs = [find_camera(name) for name in names]
#    print '0 frame-rate before', cs[0].get_frame_rate()
#    cv.SetCaptureProperty(cs[0].capture, cv.CV_CAP_PROP_FPS, 7.5)
#    print '0 frame-rate after', cs[0].get_frame_rate()
#
#    print '1 frame-rate before', cs[1].get_frame_rate()
#    cv.SetCaptureProperty(cs[1].capture, cv.CV_CAP_PROP_FPS, 7.5)
#    print '1 frame-rate before', cs[1].get_frame_rate()
#
#    #cs[0].set_frame_rate(7.5)
#    #cs[1].set_frame_rate(7.5)
#    #print 'snozz_hand frame-rate', cs[1].get_frame_rate()
#    while True:
#        #t = time.time()
#        #f1 = cs[0].get_frame()
#        #time.sleep(1)
#        f2 = cs[0].get_frame()
#        cv.ShowImage('snozzberry_hand', f2)
#
#        f = cs[1].get_frame()
#        cv.ShowImage('ele_carriage', f)
#
#        #time.sleep(1)
#        #fs = [c.get_frame() for c in cs]
#        #print 'fps', 1.0 / (time.time() - t)
#        #[cv.ShowImage(name, f) for name, f in zip(names, fs)]
#        cv.WaitKey(44)


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
