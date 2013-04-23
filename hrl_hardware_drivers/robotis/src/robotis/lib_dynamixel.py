#!/usr/bin/python
#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

## Controlling Robotis Dynamixel RX-28, RX-64, and MX-64 servos from python
## using the USB2Dynamixel adaptor.

## Authors: Travis Deyle, Advait Jain, Marc Killpack, and Phillip Grice (Healthcare Robotics Lab, Georgia Tech.)

import serial
import time
import thread
import sys, optparse
import math

class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
        try:
            self.dev_name = string.atoi( dev_name ) # stores the serial port as 0-based integer for Windows
        except:
            self.dev_name = dev_name # stores it as a /dev-mapped string for Linux / Mac

        self._mutex = thread.allocate_lock()

        self._acq_mutex()
        self.servo_dev = self._open_serial( baudrate )
        self._rel_mutex()

    def _acq_mutex(self):
        self._mutex.acquire()

    def _rel_mutex(self):
        self._mutex.release()

    def _open_serial(self, baudrate):
        servo_dev = None
        try:
            servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            # Closing the device first seems to prevent "Access Denied" errors on WinXP
            # (Conversations with Brian Wu @ MIT on 6/23/2010)
            servo_dev.close()
            servo_dev.setParity('N')
            servo_dev.setStopbits(1)
            servo_dev.open()

            servo_dev.flushOutput()
            servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError('lib_dynamixel: Serial port not found!\n')
        if(servo_dev == None):
            raise RuntimeError('lib_dynamixel: Serial port not found!\n')
        return servo_dev

    def _write_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.write( msg )

    def _send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self._write_serial( out )

    def _read_serial(self, nBytes=1):
        ''' Reads data from the servo
        '''
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def _receive_reply(self, id):
        ''' Reads the status packet returned by the servo
        '''
        start = self._read_serial( 2 )
        if start != '\xff\xff':
            raise RuntimeError('lib_dynamixel: Failed to receive start bytes\n')
        servo_id = self._read_serial( 1 )
        if ord(servo_id) != id:
            raise RuntimeError('lib_dynamixel: Incorrect servo ID received: %d\n' % ord(servo_id))
        data_len = self._read_serial( 1 )
        err = self._read_serial( 1 )
        data = self._read_serial( ord(data_len) - 2 )
        checksum = self._read_serial( 1 ) # I'm not going to check...
        return id, [ord(v) for v in data], ord(err)

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def _send_instruction(self, instruction, id, status_return=True):
        ''' Fills out packet metadata, manages mutex, sends packet, and handles response.
        '''
        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]
        self._acq_mutex()
        try:
            self._send_serial( msg )
            if status_return:
                id, data, err = self._receive_reply(id)
            else:
                id = 0xFE
                data = []
                err = 0 #No Error Received
        except:
            self._rel_mutex()
            raise
        self._rel_mutex()
        if err != 0:
            self._process_err( err, id )
        return data

    def _process_err( self, err, id ):
        ''' Process and raise errors received from the robotis servo.
        '''
        msg = "Errors reported by Dynamixel ID %d:" %id
        if err & 1:
            msg += "\n\tInput Voltage Error: Applied Voltage outside of set operating range."
        if err & 2:
          msg += "\n\tAngle Limit Error: Received Goal position outside of set limits."
        if err & 4:
          msg += "\n\tOverHeating Error: Temperature exceeds set temperature limit."
        if err & 8:
          msg += "\n\tRange Error: Received command beyond usage range."
        if err & 16:
          msg += "\n\tCheckSum Error: Invalid Checksum in Instruction Packet."
        if err & 32:
          msg += "\n\tOverload Error: Current load exceeds set torque limit"
        if err & 64:
            msg += "\n\tInstruction Error: Received undefined instruction,"+\
            "or action command received before command was registered\n"
        raise RuntimeError(msg)

    def read_address(self, id, address, nBytes=1):
        ''' reads nBytes from address on the servo at id.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self._send_instruction( msg, id )

    def write_address(self, id, address, data):
        ''' writes data at the address on the servo of id.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self._send_instruction( msg, id )

    def ping(self, id):
        ''' pings servo at id
        '''
        msg = [ 0x01 ]
        return self._send_instruction( msg, id )

    def reset_to_factory(self, id):
        '''Send the reset instruction to the servo at id.
           This will return the servo to ID 1 with a baudrate of 57600,
           and reset the EEPROM to factory defaults.  REQUIRES CONFIRMATION.
        '''
        msg = [0x06]
        resp = raw_input("This will reset all parameters on the Dynamixel "+\
                         "to factory defaults.\n"+\
                         "Are you sure you wish to continue? (y/n)")
        if resp == 'y' or resp == 'Y' or resp == 'yes':
            print "Resetting to factory defaults: ID = 1, baudrate=57600"
            return self._send_instruction(msg, id)
        else:
            print "Aborting..."
            return None

    def sync_write(self, data):
        '''writes data to address 0xFE (254), the broadcast address.
           sends data to all servos on a bus
        '''
        msg = [ 0x83 ] + data
        return self._send_instruction(msg, id=0xFE, status_return=False)

class Dynamixel_Chain(USB2Dynamixel_Device):
    ''' Class that manages multiple servos on a single Dynamixel Device
    '''
    def __init__(self, dev='/dev/ttyUSB0', baudrate='57600', ids=None, series=['RX']):
        ''' Accepts device file, baudrate, and a list of id numbers for servos (if known)
        '''
        USB2Dynamixel_Device.__init__(self, dev, baudrate)

        valid_servo_ids = self._find_servos(ids)
        if len(valid_servo_ids) == 0:
            raise RuntimeError("No valid servo IDs Found")

        if type(series) == str:
            series = [series]*len(valid_servo_ids)
        elif len(series) == 1:
            series = series*len(valid_servo_ids)

        self.servos = {}
        for id, series in zip(valid_servo_ids, series):
            self.servos[id] = Robotis_Servo(id, series)

    def _find_servos(self, ids=None):
        ''' Finds all servo IDs on the USB2Dynamixel, or check given ids
        '''
        if ids is None:
            print 'Scanning all possible ID\'s for Servos'
            ids = range(254)
            suggested_ids = False
        else:
            print 'Scanning for Servos with ID(\'s): %s' %ids
            suggested_ids = True
        self.servo_dev.setTimeout( 0.03 ) # To make the scan faster
        servos = []
        for i in ids:
            if self._id_on_device(i):
                print '\n FOUND A SERVO @ ID %d\n' % i
                servos.append(i)
            else:
                if suggested_ids:
                    print "Cannot find ID %s on %s" %(i, self.dev_name)

        self.servo_dev.setTimeout( 1.0 ) # Restore to original
        return servos

    def _id_on_device(self, id):
        '''check for a specific id on the dynamixel device. pings servo and reads id.
        '''
        try:
            self.ping(id)
            servo_id = self.read_address(id,3)[0]
            assert servo_id == id, "ID value received from servo did not match id of call"
            return True
        except:
            return False

    def init_cont_turn(self, id):
        '''sets CCW angle limit to zero and allows continuous turning (good for wheels).
        After calling this method, simply use 'set_angvel' to command rotation.  This
        rotation is proportional to torque according to Robotis documentation.
        '''
        self.write_address(id, 0x08, [0,0])

    def kill_cont_turn(self, id):
        '''resets CCW angle limits to allow commands through 'move_angle' again.
        '''
        self.write_address(id, 0x08, [255, 3])

    def is_moving(self, id):
        ''' returns True if servo (id) is moving.
        '''
        data = self.read_address(id, 0x2e, 1 )
        return data[0] != 0

    def read_voltage(self, id):
        ''' returns voltage (Volts) seen by servo (id).
        '''
        data = self.read_address(id, 0x2a, 1 )
        return data[0] / 10.

    def read_temperature(self, id):
        ''' returns the temperature (Celcius) of servo (id).
        '''
        data = self.read_address(id, 0x2b, 1 )
        return data[0]

    def read_load(self, id):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data = self.read_address(id, 0x28, 2)
        load = data[0] + (data[1] >> 6) * 256
        if data[1] >> 2 & 1 == 0:
            return -1.0 * load
        else:
            return 1.0 * load

    def read_encoder(self, id):
        ''' returns position in encoder ticks of servo at id.
        '''
        data = self.read_address(id, 0x24, 2)
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_angle(self, id):
        ''' returns the angle (radians) of servo at id.
        '''
        return self.servos[id].encoder_to_angle(self.read_encoder(id))

    def read_angles(self, ids=None):
        ''' return a list of current joint angles for servos with given ids
        '''
        if ids is None:
            ids = self.servos.keys()
        angles = [self.read_angle(id) for id in ids]
        return angles, ids

    def _bytes_to_angvel(self, hi, lo):
        '''returns the current angular velocity from hi, lo bytes
        '''
        val = lo + hi * 256
        raw_mag = (val % 1024) * 0.11 #binary value ~= 0.11rpm/unit
        mag = (raw_mag / 60.) * 2 * math.pi
        if val / 1024 == 1:
            mag *= -1
        return mag

    def read_angvel(self, id):
        ''' returns the angular velocity (rad/s) of servo at id.
        '''
        data = self.read_address(id, 38, 2)
        return self._bytes_to_angvel(data[1], data[0])

    def read_angvels(self, ids=None):
        '''return a list of current angular velocities for servos with given ids
        '''
        if ids is None:
            ids = self.servos.keys()
        angvels = [self.read_angvel(id) for id in ids]
        return angvels, ids

    def read_ang_angvel(self, id):
        '''returns the angular position and angular velocity from a single read
        '''
        data = self.read_address(id, 36, 4)
        enc_val = data[0] + data[1] * 256
        ang = self.servos[id].encoder_to_angle(enc_val)
        angvel = self._bytes_to_angvel(data[3], data[2])
        return ang, angvel

    def read_angs_angvels(self, ids=None):
        '''return lists of current angular positions and velocities for given ids
        '''
        if ids is None:
            ids = self.servos.keys()
        angles = []
        angvels = []
        for id in ids:
            ang, angvel = self.read_ang_angvel(id)
            angles.append(ang)
            angvels.append(angvel)
        return angles, angvels, ids

    def move_angle(self, id, ang, angvel=None, blocking=False):
        ''' move servo with id to angle (radians) with velocity (rad/s)
        '''
        if angvel == None:
            angvel = self.servos[id].settings['max_speed']
        else:
            angvel = self.servos[id].clip_angvel(angvel)
        av_hi, av_lo = self.__angvel_to_bytes(angvel)

        ang = self.servos[id].clip_angle(ang)
        enc_val = self.servos[id].angle_to_encoder(ang)
        ang_hi, ang_lo = self.__encoder_to_bytes(id, enc_val)

        self.write_address(id, 30, [ang_lo, ang_hi, av_lo, av_hi])

        if blocking == True:
            while(self.is_moving(id)):
                continue

    def move_angles_sync(self, angs, angvels=None, ids=None):
        ''' move servos with id's to angles with angvels using a single sync_write.
            clips angles to allowed range, and limits angvel to max allowed.
        '''
        if ids is None:
            ids = self.servos.keys()
        if angvels is None:
            angvels = [servo.settings['max_speed'] for servo in [self.servos[id] for id in ids]]
        #Check that there is an angle, angvel for each id
        assert len(ids) == len(angvels) ,  "Number of ids and angvels do not match"
        assert len(ids) == len(angs) , "Number of ids and angles do not match"

        msg = [0x1E, 0x04] #Start address, length of data per servo (4 bytes)
        for id, ang, vel in zip(ids, angs, angvels):
            ang = self.servos[id].clip_angle(ang)
            enc_tics = self.servos[id].angle_to_encoder(ang)
            ang_hi, ang_lo = self.__encoder_to_bytes(id, enc_tics)
            vel = self.servos[id].clip_angvel(vel)
            vel_hi, vel_lo = self.__angvel_to_bytes(vel)
            msg.extend([id, ang_lo, ang_hi, vel_lo, vel_hi])
        self.sync_write(msg)

    def move_to_encoder(self, id, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1

        hi, lo = self.__encoder_to_bytes(id, n)
        return self.write_address(id, 0x1e, [lo,hi] )

    def enable_torque(self, id):
        return self.write_address(id, 0x18, [1])

    def disable_torque(self, id):
        return self.write_address(id, 0x18, [0])

    def __encoder_to_bytes(self, id, n):
        ''' convert encoder value to hi, lo bytes
        '''
        n = min( max( n, 0 ), self.servos[id].settings['max_encoder'] )
        hi, lo = n / 256, n % 256
        return hi, lo

    def __angvel_to_bytes(self, angvel):
        ''' Convert Angular velocity, in rad/sec, to hi, lo bytes.
        '''
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc < 0:
            hi, lo = abs(angvel_enc) / 256 + 4, abs(angvel_enc) % 256
        else:
            hi, lo = angvel_enc / 256, angvel_enc % 256
        return hi, lo

    def set_angvel(self, id, angvel):
        ''' set angvel (rad/s) of servo id
        '''
        hi, lo =  self.__angvel_to_bytes(angvel)
        return self.write_address(id, 0x20, [lo, hi] )

    def set_id(self, current_id, new_id):
        ''' changes the servo id from current_id to new_id
        '''
        if (new_id < 0) or (new_id > 253):
            raise RuntimeWarning("Robotis ID must be between 0 and 253")
        resp = self.write_address(current_id, 0x03, [new_id])
        valid_servo_ids = self._find_servos([new_id])
        self.servos[new_id] = Robotis_Servo(new_id, series=self.servos[current_id].series)
        self.servos.pop(current_id)
        return resp

    def set_baudrate(self, id, baudrate=0x22):
        ''' Set the baudrate of the servo at id. Smaller == Faster. Default: 34 -> 57600.
        '''
        return self.write_address(id, 0x04, [baudrate])

    def set_return_delay(self, id, delay=250):
        ''' Set Return Delay Time (0-255). Smaller = shorter. Default=250.
        '''
        return self.write_address(id, 0x05, [delay])

class Robotis_Servo():
    ''' Class to use a robotis RX-28 or RX-64 servo.
    '''
    def __init__(self, servo_id, series = 'RX' ):
        '''servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
            series - Just a convenience for defining "good" defaults on MX series.
                     When set to "MX" it uses these values, otherwise it uses values
                     better for AX / RX series.  Any of the defaults can be overloaded
                     on a servo-by-servo bases in servo_config.py
        '''
        self.servo_id = servo_id
        self.series = series
        # To change the defaults, load some or all changes into servo_config.py
        if series == 'MX':
            defaults = {
                'home_encoder': 0x7FF,
                'max_encoder': 0xFFF,
                'rad_per_enc': 2*math.pi / 0xFFF,
                'max_ang': math.pi,
                'min_ang': -math.pi,
                'flipped': False,
                'max_speed': math.radians(180)
                }
        elif series == 'RX': # Common settings for RX-series.  Can overload in servo_config.py
            defaults = {
                'home_encoder': 0x200,
                'max_encoder': 0x3FF,  # Assumes 0 is min.
                'rad_per_enc': math.radians(300.0) / 1024.0,
                'max_ang': math.radians(148),
                'min_ang': math.radians(-148),
                'flipped': False,
                'max_speed': math.radians(180)
                }
        else:
            raise RuntimeError('Unrecognized Series name: %s' %self.series)

        # Set various parameters.  Load from servo_config.
        self.settings = defaults
        try:
            import servo_config as sc
            if sc.servo_param.has_key( self.servo_id ):
                for key, val in sc.servo_param [ self.servo_id ].iteritems():
                    self.settings[key] = val
            else:
                print 'Warning: servo_id ', self.servo_id, ' not found in servo_config.py.  Using defaults.'
        except:
            print 'Warning: servo_config.py not found.  Using defaults.'
        print "Created new Robotis Servo: ID = %d, series = %s" %(servo_id, series)

    def angle_to_encoder(self, ang):
        ''' return encoder position for given angle (radians)
        '''
        if self.settings['flipped']:
            ang *= -1.0
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        return enc_tics

    def encoder_to_angle(self, enc_val):
        '''return angular position (rad) from given encoder position
        '''
        ang = ((enc_val - self.settings['home_encoder']) *
                self.settings['rad_per_enc'])
        if self.settings['flipped']:
            ang *= -1.0
        return ang

    def clip_angle(self, ang):
        ''' Clip commanded joint angles to within the allowed range.
        '''
        if self.settings['flipped']:
            ang *= -1.0
        if ang < self.settings['min_ang']:
            print "Servo %d: Commanded angle (%f) below minimum (%f), commanding to minimum."\
                    %(self.servo_id, ang, self.settings['min_ang'])
            return self.settings['min_ang']
        elif ang > self.settings['max_ang']:
            print "Servo %d: Commanded angle (%f) above maximum (%f), commanding to maximum."\
                    %(self.servo_id, ang, self.settings['max_ang'])
            return self.settings['max_ang']
        else:
            return ang

    def clip_angvel(self, angvel):
        '''Clip commanded velocity to below the allowed maximum.
           negative angvels will be set to maximum.
        '''
        if angvel > self.settings['max_speed']:
            print "Servo %d: Tried to set ang vel to %f, above maximum (%f), setting to maximum."\
                    %(self.servo_id, angvel, self.settings['max_speed'])
            return self.settings['max_speed']
        if angvel < 0:
            return self.settings['max_speed']
        else:
            return angvel

def discover_servos(dev='/dev/ttyUSB0', ids=None, baudrates=None):
    '''Discover all servos on a USB2Dynamixel_Device using PING command.
       Checks all servo IDs at all Baudrates.  Can specify smaller ranges to check instead.
    '''
    if baudrates is None:
        baudrates = [3000000, 2500000, 2250000, 1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600]
    print "Searching for ID's on %s" %dev
    for baudrate in baudrates:
        print "Baudrate %d:" %baudrate
        try:
            dyn = Dynamixel_Chain(dev, baudrate=baudrate, ids = ids)
            dyn.servo_dev.close()
            del(dyn)
        except RuntimeError as rte:
            print rte

def recover_servo(dyn):
    ''' Recovers a bricked servo by booting into diagnostic bootloader and resetting '''
    raw_input('Make sure only one servo connected to USB2Dynamixel Device [ENTER]')
    raw_input('Disconnect power from the servo, but leave USB2Dynamixel connected to USB. [ENTER]')

    dyn.servo_dev.setBaudrate( 57600 )
    
    print 'Get Ready.  Be ready to reconnect servo power when I say \'GO!\''
    print 'After a second, the red LED should become permanently lit.'
    print 'After that happens, Ctrl + C to kill this program.'
    print
    print 'Then, you will need to use a serial terminal to issue additional commands.',
    print 'Here is an example using screen as serial terminal:'
    print
    print 'Command Line:  screen /dev/robot/servo_left 57600'
    print 'Type: \'h\''
    print 'Response: Command : L(oad),G(o),S(ystem),A(pplication),R(eset),D(ump),C(lear)'
    print 'Type: \'C\''
    print 'Response:  * Clear EEPROM '
    print 'Type: \'A\''
    print 'Response: * Application Mode'
    print 'Type: \'G\''
    print 'Response:  * Go'
    print
    print 'Should now be able to reconnect to the servo using ID 1'
    print
    print
    raw_input('Ready to reconnect power? [ENTER]')
    print 'GO!'

    while True:
        s.write('#')
        time.sleep(0.0001)


if __name__ == '__main__':
    usage =  ("Interface for controlling one or more robotis servos on a single bus\n"+
             "\tUse as below from the commandline, or:\n"+
             "\t\timport lib_dynamixel as ld\n"+
             "\t\tdyn = ld.Dynamixel_Chain()\n"+
             "\t\tdyn.move_angle(ang, id)")
    p = optparse.OptionParser(usage=usage)
    p.add_option('-d', action='store', type='string', dest='dev_name',
                 help='Required: Device string for USB2Dynamixel. [i.e. /dev/ttyUSB0 for Linux, \'0\' (for COM1) on Windows]')
    p.add_option('--scan', action='store_true', dest='scan', default=False,
                 help='Scan the device for servo IDs attached.')
    p.add_option('--recover', action='store_true', dest='recover', default=False,
                 help='Recover from a bricked servo (restores to factory defaults).')
    p.add_option('--ang', action='store', type='float', dest='ang',
                 help='Angle to move the servo to (degrees).')
    p.add_option('--ang_vel', action='store', type='float', dest='ang_vel',
                 help='angular velocity. (degrees/sec) [default = 50]', default=50)
    p.add_option('--id', action='store', type='int', dest='id',
                 help='id of servo to connect to, [default = 1]', default=1)
    p.add_option('--baud', action='store', type='int', dest='baud',
                 help='baudrate for USB2Dynamixel connection [default = 57600]', default=57600)

    opt, args = p.parse_args()

    if opt.dev_name == None:
        p.print_help()
        sys.exit(0)

    dyn = Dynamixel_Chain(opt.dev_name, opt.baud)

    if opt.scan:
        discover_servos( dyn )

    if opt.recover:
        recover_servo( dyn )

    if opt.ang != None:
        dyn.move_angle(opt.id, math.radians(opt.ang), math.radians(opt.ang_vel) )
