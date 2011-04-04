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

## Controlling Robotis Dynamixel servos from python using the USB2Dynamixel adaptor.
## author Advait Jain (Healthcare Robotics Lab, Georgia Tech.)



import serial
import time
import sys, optparse
import servo_config as sc
import math

class robotis_servo():
    ''' class to use a robotis servo.
    '''
    def __init__(self, dev_name, servo_id, baudrate=57600,
                 max_speed=math.radians(50)):
        ''' dev_name - name of serial device of the servo controller (e.g. '/dev/robot/servo0')
            servo_id - 2,3,4 ... (2 to 253)
            baudrate - for the servo controller.
            max_speed - max allowable speed for the servo (radians/sec)
        '''
        self.dev_name = dev_name
        self.servo_dev = None
        self.__open_serial(baudrate)

        print 'WARNING: robotis_servo.py is being deprecated.'

        self.servo_id = servo_id
        self.home_encoder_value = sc.servo_param[servo_id]['home_encoder']

        if sc.servo_param[servo_id].has_key('max_ang'):
            self.max_ang = sc.servo_param[servo_id]['max_ang']
        else:
            self.max_ang = math.radians( 95.0 )

        if sc.servo_param[servo_id].has_key('min_ang'):
            self.min_ang = sc.servo_param[servo_id]['min_ang']
        else:
            self.min_ang = math.radians( -95.0 )

        if sc.servo_param[servo_id].has_key('flipped'):
            self.flipped = True
        else:
            self.flipped = False

        if self.read_location(3) == None:
            print 'robotis_servo.robotis_servo.__init__: Wrong servo ID- ', self.servo_id

        self.fast_angvel = max_speed

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        data,err = self.read_location(0x2e,1)
        return data[0]!=0

    def read_voltage(self):
        ''' returns voltage (Volts)
        '''
        data,err = self.read_location(0x2a,1)
        return data[0]/10.

    def read_temperature(self):
        ''' returns the temperature (Celcius)
        '''
        data,err = self.read_location(0x2b,1)
        return data[0]

    def read_angle(self):
        ''' returns the current servo angle (radians)
        '''
        data,err = self.read_location(0x24,2)
        ang = (data[0]+data[1]*256-self.home_encoder_value)/1024. * 300.
        if self.flipped:
            ang = ang*-1.0
        return math.radians(ang)

    def print_encoder_value(self):
        ''' position in encoder ticks
        '''
        data,err = self.read_location(0x24,2)
        enc_val = data[0]+data[1]*256
        print 'encoder_value:', enc_val

    def read_load(self):
        ''' number proportional to the torque applied by the servo.
            sign etc. might vary with how the servo is mounted.
        '''
        data,err = self.read_location(0x28,2)
        load = data[0]+(data[1]>>6)*256
#        print 'data[1],data[0]:', data[1],data[0]
        if data[1]>>2 & 1 == 0:
            return -load
        else:
            return load

    def __torque_enable(self, n):
        self.write_location(0x18, [n])

    def enable_torque(self):
        self.__torque_enable(1)

    def disable_torque(self):
        self.__torque_enable(0)

    def __move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        hi,lo = n/256,n%256
        self.write_location(0x1e, [lo,hi])

    def move_angle(self, ang, angvel=None,blocking=True):
        ''' move to angle (radians)
        '''
        if angvel == None:
            angvel = self.fast_angvel

        if angvel>self.fast_angvel:
            print 'robotis_servo.move_angle: angvel too high - %.2f deg/s'%(math.degrees(angvel))
            print 'ignoring move command.'
            return

        if ang > self.max_ang or ang < self.min_ang:
            print 'robotis_servo.move_angle: angle out of range- ', math.degrees(ang)
            return
        self.set_angvel(angvel)
        time.sleep(0.05)

        deg = math.degrees(ang)
        if self.flipped:
            deg = deg * -1.0
        enc_ticks = int(round(deg/0.29))
        enc_ticks += self.home_encoder_value
        self.__move_to_encoder(enc_ticks)

        if blocking == True:
            time.sleep(0.05)
            while(self.is_moving()):
                continue
        else:
            time.sleep(0.05)


    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''
        rpm = angvel/(2*math.pi)*60
        angvel_enc = int(round(rpm/0.111))
        hi,lo = angvel_enc/256,angvel_enc%256
        self.write_location(0x20, [lo,hi])

    def write_id(self, id):
        ''' changes the servo id
        '''
        self.write_location(0x03,[id])

    def write_location(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
        '''
        msg = [0x03,address]+data
        self.send_instruction(msg,self.servo_id)

    def read_location(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...], error
            list of parameters, error byte.
        '''
        msg = [0x02,address,nBytes]
        self.send_instruction(msg,self.servo_id)
        s = self.read_serial(6+nBytes)
        if s == '':
            print 'robotis_servo.read_location: Could not read from the servo.'
            print 'Ensure that the 3-way switch on the USB2Dynamixel is at RS485.'
            print 'Exiting...'
            sys.exit(0)
        l = [ord(a) for a in s[4:-1]]
        if l == []:
            return None
        return l[1:], l[0]

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = (~chksum)%256
        return chksum

    def send_instruction(self, instruction, id):
        msg = [id,len(instruction)+1]+instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum(msg)
        msg = [0xff,0xff]+msg+[chksum]
        self.__send_serial(msg)

    def __send_serial(self, msg):
        """ sends the command to the servo
        """
        str = ''
        for m in msg:
            str += chr(m)

        self.servo_dev.flushInput()
        self.servo_dev.write(str)

    def read_serial(self, nBytes=1):
        rep = self.servo_dev.read(nBytes)
        return rep

    def __open_serial(self, baudrate):
        try:
            self.servo_dev = serial.Serial(self.dev_name, timeout=1.)
            self.servo_dev.setBaudrate(baudrate)
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()

            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError("robotis_servo: Serial port not found!\n")
        if(self.servo_dev == None):
            raise RuntimeError("robotis_servo: Serial port not found!\n")



if __name__ == '__main__':

    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='servo_dev_name',
                 default='/dev/robot/servo0', help='servo device string. [default= /dev/robot/servo0]')
    p.add_option('--ang', action='store', type='float', dest='ang',
                 help='angle to move the servo to (degrees).')
    p.add_option('--ang_vel', action='store', type='float', dest='ang_vel',
                 help='angular velocity. (degrees/sec) [default = 50]', default=50)
    p.add_option('--id', action='store', type='int', dest='id',
                 help='id of servo to connect to, [default = 2]', default=2)

    opt, args = p.parse_args()
    servo_dev_name = opt.servo_dev_name
    ang = opt.ang
    ang_vel = opt.ang_vel
    id = opt.id

    servo = robotis_servo(servo_dev_name,id)
    servo.move_angle(math.radians(ang), math.radians(ang_vel))
    time.sleep(0.5)
    print 'Servo angle:', math.degrees(servo.read_angle())


