import _camera_setup_lib

def _swig_repr(self):
    try: strthis = "proxy of " + self.this.__repr__()
    except: strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

import types
try:
    _object = types.ObjectType
    _newclass = 1
except AttributeError:
    class _object : pass
    _newclass = 0
del types


init_bus1394   = _camera_setup_lib.init_bus1394
getCameraUID   = _camera_setup_lib.getCameraUID
endCameraSetup = _camera_setup_lib.endCameraSetup
setFrameRate   = _camera_setup_lib.setFrameRate
getRegister    = _camera_setup_lib.getRegister
setRegister    = _camera_setup_lib.setRegister

#All hex setting to the cameras confined to this code
brightness_register_dict = { 'brightness':   0x800,  'exposure': 0x804, 
                             'shutter_time': 0x81c,  'gain':     0x820,
                             'white_balance': 0x80c, 'gamma': 0x818}

register_dict            = { 'frame_rate':    0x600, 'trigger_mode':  0x830, 
                             'trigger_delay': 0x834, 'pio_direction': 0x11f8, 
                             'strobe_0_cnt':  0x1500 }

fps_dict                 = { 60:0xa0000000,
                             30:0x80000000,
#                             20:0x70000000, 20 does not seem to be an
#                             option
                             15:0x60000000,
                             7.5:0x40000000,
                             3.75:0x20000000,
                             1.875:0x10000000 }

# Dictionary from registers to fps.  It's named sps because sps is fps backawards ;)
spf_dict                 = { '0xa':60,
                             '0x8':30,
                             '0x6':15,
                             '0x4':7.5 }


register_dict.update( brightness_register_dict )

def get_registers( index ):
    """
    index would be the .device property of a Camera object
    
    [register function, register value]"""
    return [(key,getRegister( index, register_dict[key])) for key in register_dict.keys()]

def get_frame_rate( index ):
    """
    Get the frame rate
    """
    val = (0xe0000000 & getRegister( index, register_dict['frame_rate'])) >> 7*4
    return spf_dict[hex(int(val))]    


def set_frame_rate( index, rate ):
    frame_rate = fps_dict[rate]
    setRegister( index, register_dict['frame_rate'], frame_rate)


def set_stereo_slaving( master, slave ):
    '''
    master and slave would be the .device property of Camera objects
    
    This function assumes that the right camera is slave off of the left'''
    setRegister( master, register_dict['pio_direction'], 0x80000000)
    setRegister( slave, register_dict['strobe_0_cnt'], 0x82000000)

    setRegister( slave, register_dict['trigger_mode'], 0x83110000 )
    setRegister( master, register_dict['trigger_mode'], 0x80100000 )

    for key in brightness_register_dict.keys():
        rdval = getRegister( master, register_dict[key])
        setRegister( slave, register_dict[key] ,rdval)


def get_brightness_settings( index ):
    """
    index would be the .device property of a Camera object
    
    [register function, register value]"""
    return [(key, 0xfff & getRegister( index, brightness_register_dict[key])) for key in brightness_register_dict.keys()]


def set_auto( index ):
    """Set a particular camera to automatically ajdust brightness and exposure"""
    setRegister( index, register_dict['brightness'], 0x83000000)
    setRegister( index, register_dict['exposure'], 0x83000000)
    setRegister( index, register_dict['shutter_time'], 0x8300000e)
    setRegister( index, register_dict['gain'], 0x8300000f)
    print 'set auto being called'


def get_gamma(index):
    return getRegister(index, brightness_register_dict['gamma'])

def set_gamma(index,gamma):
    ''' gamma: 0 or 1
    '''
    setRegister(index, brightness_register_dict['gamma'], 0x82000000+gamma)

def get_whitebalance(index):
    return getRegister(index, brightness_register_dict['white_balance'])

def set_whitebalance(index,r_val,b_val):
    setRegister(index, brightness_register_dict['white_balance'], 0x82000000+r_val+b_val*4096)

def set_brightness( index, brightness=None, exposure=None, shutter_time=None, gain=None ):
    """If brightness is not specified auto mode is used for all settings. If shutter_time
    and gain are specified, exposure does nothing. All values should be set between 0-4095"""

    def limit_fff( parameter ):
        if parameter > 0xfff:
            parameter = 0xfff
        elif parameter < 0:
            parameter = 0

        return parameter

    if brightness == None and exposure != None:
        setRegister( index, register_dict['brightness'], 0x83000000 )
        setRegister( index, register_dict['exposure'], 0x82000000+limit_fff(exposure))
        setRegister( index, register_dict['shutter_time'], 0x83000000)
        setRegister( index, register_dict['gain'], 0x83000000)
    
    elif brightness == None:
        set_auto( index )
    
    else:
            if shutter_time != None or gain != None:
                setRegister( index, register_dict['brightness'], 0x82000000+limit_fff(brightness))
                setRegister( index, register_dict['exposure'], 0x80000000)
                setRegister( index, register_dict['shutter_time'], 0x82000000+limit_fff(shutter_time))
                setRegister( index, register_dict['gain'], 0x82000000+limit_fff(gain))
            else:
                if exposure == None:
                    setRegister( index, register_dict['brightness'], 0x82000000+limit_fff(brightness))
                    setRegister( index, register_dict['exposure'], 0x83000000)
                    setRegister( index, register_dict['shutter_time'], 0x83000000)
                    setRegister( index, register_dict['gain'], 0x83000000)
                else:
                    setRegister( index, register_dict['brightness'], 0x82000000+limit_fff(brightness))
                    setRegister( index, register_dict['exposure'], 0x82000000+limit_fff(exposure))
                    setRegister( index, register_dict['shutter_time'], 0x83000000)
                    setRegister( index, register_dict['gain'], 0x83000000)




