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

        self.mutex = thread.allocate_lock()

        self.acq_mutex()
        self.servo_dev = self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

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

    def write_serial(self, msg):
        # It is up to the caller to acquire / release mutex
        self.servo_dev.write( msg )

    def send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self.write_serial( out )

    def read_serial(self, nBytes=1):
        ''' Reads data from the servo
        '''
        # It is up to the caller to acquire / release mutex
        rep = self.servo_dev.read( nBytes )
        return rep

    def receive_reply(self, id):
        ''' Reads the status packet returned by the servo
        '''
        start = self.read_serial( 2 )
        if start != '\xff\xff':
            raise RuntimeError('lib_dynamixel: Failed to receive start bytes\n')
        servo_id = self.read_serial( 1 )
        if ord(servo_id) != id:
            raise RuntimeError('lib_dynamixel: Incorrect servo ID received: %d\n' % ord(servo_id))
        data_len = self.read_serial( 1 )
        err = self.read_serial( 1 )
        data = self.read_serial( ord(data_len) - 2 )
        checksum = self.read_serial( 1 ) # I'm not going to check...
        return id, [ord(v) for v in data], ord(err)

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def send_instruction(self, instruction, id, status_return=True):
        ''' Fills out packet metadata, manages mutex, sends packet, and handles response.
        '''
        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]
        self.acq_mutex()
        try:
            self.send_serial( msg )
            if status_return:
                id, data, err = self.receive_reply(id)
            else:
                id = 0xFE
                data = []
                err = 0 #No Error Received
        except:
            self.rel_mutex()
            raise
        self.rel_mutex()
        if err != 0:
            self.process_err( err, id )
        return data

    def process_err( self, err, id ):
        ''' Process and raise errors received from the robotis servo.
            TODO: WILL NOT HANDLE MULTIPLE ERRORS IN A SINGLE BYTE
        '''
        if err == 1:
            msg = "Input Voltage Error"
        elif err == 2:
            msg = "Angle Limit Error"
        elif err == 4:
            msg = "OverHeating Error"
        elif err == 8:
            msg = "Range Error"
        elif err == 16:
            msg = "CheckSum Error"
        elif err == 32:
            msg = "Instruction Error"
        elif err == 32:
            msg = "Overload Error"
        elif err == 64:
            msg = "Instruction Error"
        raise RuntimeError('lib_dynamixel: An error occurred on servo %d: %d, %s\n' %(id, err, msg))

    def read_address(self, id, address, nBytes=1):
        ''' reads nBytes from address on the servo at id.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, id )

    def write_address(self, id, address, data):
        ''' writes data at the address on the servo of id.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, id )

    def ping(self, id):
        ''' pings servo at id
        '''
        msg = [ 0x01 ]
        return self.send_instruction( msg, id )

    def sync_write(self, data):
        '''writes data to address 0xFE (254), the broadcast address.
           sends data to all servos on a bus
        '''
        msg = [ 0x83 ] + data
        return self.send_instruction(msg, id=0xFE, status_return=False)


class Dynamixel_Chain(USB2Dynamixel_Device):
    ''' Class that manages multiple servos on a single Dynamixel Device
    '''
    def __init__(self, dev='/dev/ttyUSB0', baudrate='57600', servo_ids=None):
        ''' Accepts device file, baudrate, and a list of id numbers for servos (if known)
        '''
        USB2Dynamixel_Device.__init__(self, dev, baudrate)

        valid_servo_ids = self.find_servos(servo_ids)
        self.servos = {}
        for id in valid_servo_ids:
            self.servos[id] = Robotis_Servo(id, series='RX')

    def find_servos(self, ids=None):
        ''' Finds all servo IDs on the USB2Dynamixel, or check given ids
        '''
        servos = []
        self.servo_dev.setTimeout( 0.03 ) # To make the scan faster
        if ids is None:
            print 'Scanning all possible ID\'s for Servos'
            ids = range(254)
        else:
            print 'Scanning for Servos with ID\'s: %s' %ids
        for i in ids:
            try:
                self.ping(i)
#                self.read_address(i,3)
                print '\n FOUND A SERVO @ ID %d\n' % i
                servos.append( i )
            except:
                pass
        self.servo_dev.setTimeout( 1.0 ) # Restore to original
        return servos

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
        data = self.read_address(id, 0x28, 2 )
        load = data[0] + (data[1] >> 6) * 256
        if data[1] >> 2 & 1 == 0:
            return -1.0 * load
        else:
            return 1.0 * load

    def read_encoder(self, id):
        ''' returns position in encoder ticks of servo at id.
        '''
        data = self.read_address(id, 0x24, 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_angle(self, id):
        ''' returns the angle (radians) of servo at id.
        '''
        ang = (self.read_encoder(id) -
                self.servos[id].settings['home_encoder']) * self.servos[id].settings['rad_per_enc']
        if self.servos[id].settings['flipped']:
            ang = ang * -1.0
        return ang

    def read_angles(self, ids=None):
        ''' return a list of current joint angles for servos with given ids
        '''
        if ids is None:
            ids = self.servos.keys()
        angles = []
        for id in ids:
            angles.append(self.read_angle(id))
        return angles, ids

    def move_angle(self, id, ang, angvel=None, blocking=False):
        ''' move servo with id to angle (radians) with velocity (rad/s)
        '''
        if angvel == None:
            angvel = self.servos[id].settings['max_speed']

        if angvel > self.servos[id].settings['max_speed']:
            print 'lib_dynamixel.move_angle: angvel too high - %.2f deg/s' % (math.degrees(angvel))
            print 'lib_dynamixel.ignoring move command.'
            return

        if ang > self.servos[id].settings['max_ang'] or ang < self.servos[id].settings['min_ang']:
            print 'lib_dynamixel.move_angle: angle out of range- ', math.degrees(ang)
            print 'lib_dynamixel.ignoring move command.'
            return

        self.set_angvel(id, angvel)

        enc_tics = self.servos[id].angle_to_encoder(ang)
        self.move_to_encoder(id, enc_tics )

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
            ang_hi, ang_lo = self.encoder_to_bytes(id, enc_tics)
            vel = self.servos[id].clip_ang_vels(vel)
            vel_hi, vel_lo = self.angvel_to_bytes(vel)
            msg.extend([id, ang_lo, ang_hi, vel_lo, vel_hi])
        self.sync_write(msg)

    def move_to_encoder(self, id, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1

        hi, lo = self.encoder_to_bytes(id, n)
        return self.write_address(id, 0x1e, [lo,hi] )

    def enable_torque(self, id):
        return self.write_address(id, 0x18, [1])

    def disable_torque(self, id):
        return self.write_address(id, 0x18, [0])

    def encoder_to_bytes(self, id, n):
        ''' convert encoder value to hi, lo bytes
        '''
        n = min( max( n, 0 ), self.servos[id].settings['max_encoder'] )
        hi,lo = n / 256, n % 256
        return hi, lo

    def angvel_to_bytes(self, angvel):
        ''' Convert Angular velocity, in rad/sec, to hi, lo bytes.
        '''
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc<0:
            hi,lo = abs(angvel_enc) / 256 + 4, abs(angvel_enc) % 256
        else:
            hi,lo = angvel_enc / 256, angvel_enc % 256
        return hi, lo

    def set_angvel(self, id, angvel):
        ''' set angvel (rad/s) of servo id
        '''
        hi, lo =  self.angvel_to_bytes(angvel)
        return self.write_address(id, 0x20, [lo, hi] )

    def write_id(self, current_id, new_id):
        ''' changes the servo id from current_id to new_id
        '''
        return self.write_address(current_id, 0x03, [new_id] )

    def set_baudrate(self, id, baudrate=0x22):
        ''' Set the baudrate of the servo at id.  Must be a single hex byte
        (0x00--0xFF).  Smaller number == Faster baudrate.
        Default: 0x22 (34) -> 57600.
        '''
        return self.write_address(id, 0x04, baudrate)

    def set_return_delay(self, id, delay=0xFA):
        ''' Set Return Delay Time (0x00--0xFF). Smaller = shorter. Default=250 (0xFA).
        '''
        return self.write_address(id, 0x05, delay)


class Robotis_Servo():
    ''' Class to use a robotis RX-28 or RX-64 servo.
    '''
    def __init__(self, servo_id, series = None ):
        '''servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
            series - Just a convenience for defining "good" defaults on MX series.
                     When set to "MX" it uses these values, otherwise it uses values
                     better for AX / RX series.  Any of the defaults can be overloaded
                     on a servo-by-servo bases in servo_config.py
        '''
        self.servo_id = servo_id
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
        else: # Common settings for RX-series.  Can overload in servo_config.py
            defaults = {
                'home_encoder': 0x200,
                'max_encoder': 0x3FF,  # Assumes 0 is min.
                'rad_per_enc': math.radians(300.0) / 1024.0,
                'max_ang': math.radians(148),
                'min_ang': math.radians(-148),
                'flipped': False,
                'max_speed': math.radians(180)
                }

        # Set Return Delay time - Used to determine when next status can be requested
        #Currently not used
        #data = self.read_address( 0x05, 1)
        #self.return_delay = data[0] * 2e-6

        # Set various parameters.  Load from servo_config.
        self.settings = {}
        try:
            import servo_config as sc
            if sc.servo_param.has_key( self.servo_id ):
                self.settings = sc.servo_param[ self.servo_id ]
            else:
                print 'Warning: servo_id ', self.servo_id, ' not found in servo_config.py.  Using defaults.'
        except:
            print 'Warning: servo_config.py not found.  Using defaults.'

        # Set to default any parameter not specified in servo_config
        for key in defaults.keys():
            if not self.settings.has_key( key ):
                self.settings[ key ] = defaults[ key ]

    def angle_to_encoder(self, ang):
        ''' return encoder position for given angle (radians)
        '''
        if self.settings['flipped']:
            ang = -1.0 * ang
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        return enc_tics

    def clip_angle(self, ang):
        ''' Clip commanded joint angles to within the allowed range.
        '''
        if self.settings['flipped']:
            ang = -1.0 * ang
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

    def clip_ang_vels(self, angvel):
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


def discover_servos(dev='/dev/ttyUSB0', servo_ids=None, baudrates=None):
    '''Discover all servos on a USB2Dynamixel_Device using PING command.
       Checks all servo IDs at all Baudrates.  Can specify smaller ranges to check instead.
    '''
    if baudrates is None:
        baudrates = [1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600]
    for baudrate in baudrates:
        print "Baudrate %d:" %baudrate
        dyn = Dynamixel_Chain(dev, baudrate=baudrate, servo_ids = servo_ids)
        dyn.servo_dev.close()
        del(dyn)


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
    p = optparse.OptionParser()
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

    dyn = USB2Dynamixel_Device(opt.dev_name, opt.baud)

    if opt.scan:
        find_servos( dyn )

    if opt.recover:
        recover_servo( dyn )

    if opt.ang != None:
        servo = Robotis_Servo( dyn, opt.id )
        servo.move_angle( math.radians(opt.ang), math.radians(opt.ang_vel) )
    
