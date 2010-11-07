import roslib                      # Needed to load opencv
roslib.load_manifest('hrl_camera') #

import cv
import hrl_opencv.adaptors as ad
import camera_setup_lib as csl
import numpy as np

class NoFrameException(Exception):
    
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)

class camera:
    ##
    # @param camera_configuration a dictionary of parameters needed for this camera
    def __init__(self, camera_configuration, opencv_id):
        self.config = camera_configuration
        self.device = opencv_id
        #self._set_registers()

        #create capture and related attributes
        self.capture = cv.CaptureFromCAM(self.device)
        if not self.capture:
            raise RuntimeError("Cannot open camera!\n")
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

        cv.InitUndistortMap(self.intrinsic_cvmat, self.distortion_cvmat, 
                            self.undistort_mapx, self.undistort_mapy)
        self.corrected_orientation = None


    def get_frame(self):
        self.raw_image = self.get_raw_frame()
        im = self.undistort_frame()
        if self.config.has_key('upside_down'):
            if self.config['upside_down']:
                if self.corrected_orientation == None:
                    self.corrected_orientation = cv.CloneImage(im)
                cv.Flip(im, self.corrected_orientation, -1)
                im = self.corrected_orientation
        return im


    ## returns color image. does NOT undistort the image.
    def get_frame_debayered(self):
        self.raw_image = self.get_raw_frame()
        return self.convert_color()

    def get_raw_frame(self):
        # Assumes that we are going to debayer the image later, so
        # returns a single channel image.
        im = cv.QueryFrame(self.capture)

        if im == None:
            raise NoFrameException('')
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

    ## 
    # Set frame rate: 7.5, 15, 30, or 60Hz
    # by default we set this to a low value
    # to not have to deal with firewire bandwidth
    # issues
    def set_frame_rate(self, rate=7.5):
        cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS, rate)
        #csl.set_frame_rate(self.device, rate)

    def get_frame_rate(self):
        fps = cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS)
        return fps

    def set_brightness(self, brightness=150, shutter_time=97,
                       gain=450, exposure=None):
        csl.set_brightness(self.device, brightness, exposure, shutter_time, gain)

    ## auto - probably only used for displaying images.
    def set_auto(self):
        csl.set_auto(self.device)
    
#if __name__ == '__main__':
#    cva = ad.array2cv(np.ones((2,3)))
