import roslib                      # Needed to load opencv
roslib.load_manifest('hrl_camera') #
import cv
import camera_setup_lib as csl
import camera

class dragonfly2(camera.camera):
    def __init__(self, camera_configuration, opencv_id):
        self.config = camera_configuration
        self.device = opencv_id
        self._set_registers()
        camera.camera.__init__(self, camera_configuration, opencv_id)

        #create capture and related attributes
        #self.capture = cv.CaptureFromCAM(self.device)
        #if not self.capture:
        #    raise RuntimeError("Cannot open camera!\n")
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

    
