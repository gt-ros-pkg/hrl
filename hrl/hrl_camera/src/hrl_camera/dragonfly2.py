import roslib                      # Needed to load opencv
roslib.load_manifest('hrl_camera') #
import cv
import hrl_opencv.adaptors as ad
import hrl_opencv.util as vut
import camera_uuid as uuid
import camera_setup_lib as csl
import numpy as np
import pdb

class dragonfly2:
    ##
    # @param camera_configuration a dictionary of parameters needed for this camera
    def __init__(self, camera_configuration, opencv_id):
        self.config = camera_configuration
        self.device = opencv_id
        self._set_registers()

        #create capture and related attributes
        self.capture = cv.CaptureFromCAM(self.device)
        if not self.capture:
            raise RuntimeError("Cannot open camera!\n")
        cur_codec = cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_MODE)
        print "dragonfly2: current codec interpretation is : ", cur_codec
        integ = cv.SetCaptureProperty(self.capture,cv.CV_CAP_PROP_MODE,
                                      self.config['cv_cap_prop_mode'])

        #self.set_frame_rate(3.75) # set it really low to start out.
                                  # increase later.

        fps = cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS)
        print "dragonfly2: fps : ", fps
        next_codec = cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_MODE)
        print "dragonfly2: current codec interpretation is : ", next_codec
        self._make_undistort_matrices()


    def _make_undistort_matrices(self):
        p = self.config
        some_arr = np.array([[p['focal_length_x_in_pixels'], 0, p['optical_center_x_in_pixels']],
                                                     [0, p['focal_length_y_in_pixels'], p['optical_center_y_in_pixels']],
                                                     [0, 0, 1.0]])
        self.intrinsic_cvmat = ad.array2cvmat(some_arr)
        self.distortion_cvmat = ad.array2cvmat(np.array([[p['lens_distortion_radial_1'],
                                                       p['lens_distortion_radial_2'],
                                                       p['lens_distortion_tangential_1'],
                                                       p['lens_distortion_tangential_2']]]))
        self.size = (int(p['calibration_image_width']), int(p['calibration_image_height']))

        #Sanity check
        size_image = cv.QueryFrame(self.capture)
        camera_image_size = cv.GetSize(size_image)
        if not ((camera_image_size[0] == self.size[0]) and (camera_image_size[1] == self.size[1])):
            raise RuntimeError('Size of image returned by camera and size declared in config. file do not match.' 
                               + '  Config:' + str(self.size) + ' Camera: ' + str(camera_image_size))


        #Set up buffers for undistortion
        self.raw_image       = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 1)
        self.gray_image      = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 1)
        self.undistort_mapx  = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.undistort_mapy  = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.unbayer_image   = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 3)

        self.color = p['color']
        if self.color == True:
            self.cvbayer_pattern = p['opencv_bayer_pattern'] 
        if self.color == True:
            self.undistort_image = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 3)
        else:
            self.undistort_image = cv.CreateImage(self.size, cv.IPL_DEPTH_8U, 1)

        #import pdb
        #pdb.set_trace()
        cv.InitUndistortMap(self.intrinsic_cvmat, self.distortion_cvmat, 
                            self.undistort_mapx, self.undistort_mapy)

    def _set_registers(self):
        csl.init_bus1394()
        # Mode
        csl.setRegister(self.device,0x604,0xA0000000)
        # Format
        csl.setRegister(self.device,0x608,0x20000000)
        #sets raw bayer image format for mono image format modes 
        csl.setRegister(self.device, 0x1048, 0x80000081)

        mode  = csl.getRegister(self.device, 0x604)
        format= csl.getRegister(self.device, 0x608)
        rate  = csl.getRegister(self.device, 0x600)
        #software_trigger
        print "dragonfly2: mode", hex(mode)
        print "dragonfly2: format", hex(format)
        print "dragonfly2: rate", hex(rate)

    def get_frame(self):
        self.raw_image = self.get_raw_frame()
        return self.undistort_frame()

    ## returns color image. does NOT undistort the image.
    def get_frame_debayered(self):
        self.raw_image = self.get_raw_frame()
        return self.convert_color()

    def get_raw_frame(self):
        # Assumes that we are going to debayer the image later, so
        # returns a single channel image.
        im = cv.QueryFrame(self.capture)
        cv.Split(im, self.gray_image, None, None, None)
        return self.gray_image

    def undistort_frame(self):
        img = self.convert_color()
        cv.Remap(img, self.undistort_image, self.undistort_mapx, self.undistort_mapy, 
                 cv.CV_INTER_LINEAR, cv.ScalarAll(0)) 
        return self.undistort_image

    def convert_color(self):
        if self.color == True:
            cv.CvtColor(self.raw_image, self.unbayer_image, self.cvbayer_pattern)
            return self.unbayer_image
        else:
            return self.raw_image

    ## Set frame rate: 7.5, 15, 30, or 60Hz
    def set_frame_rate(self, rate):
        csl.set_frame_rate(self.device, rate)

    def set_brightness(self, brightness=150, shutter_time=97, gain=450):
        csl.set_brightness(self.device, brightness, None, shutter_time, gain)

    ## auto - probably only used for displaying images.
    def set_auto(self):
        csl.set_auto(self.device)
    
if __name__ == '__main__':
    cva = ad.array2cv(np.ones((2,3)))
