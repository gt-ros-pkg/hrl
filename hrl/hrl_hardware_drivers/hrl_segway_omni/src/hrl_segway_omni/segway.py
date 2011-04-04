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

#  \author Cressel Anderson (Healthcare Robotics Lab, Georgia Tech.)

import usb
import pickle
import time
import numpy as np

import math
import struct

class Mecanum_Properties():
    def __init__( self ):
        self.r1 = .2032/2. # 8in wheels
        self.r2 = .2032/2. # 8in wheels
        
        self.R = 4*2.54/100 # wheel radius in meters
        
        self.la = .6223/2 # 1/2 width of the base
        self.lb = .33655/2 # 1/2 length of the base

class Mecanum( Mecanum_Properties ):
    def __init__( self ):
        self.segway_front = Segway( side='front')
        self.segway_back  = Segway( side='back' )

        Mecanum_Properties.__init__(self)
        self.get_status()

    def set_velocity( self, xvel, yvel, avel ):
        """xvel and yvel should be in m/s and avel should be rad/s"""
        yvel = -yvel

        R = self.R
        la = self.la
        lb = self.lb
        
        # eq. 17 from Knematic modeling for feedback control of an
        # omnidirectional wheeled mobile robot by Muir and Neuman at
        # CMU
        J = 1/R * np.matrix([-1,1, (la + lb),
                              1,1,-(la + lb),
                             -1,1,-(la + lb),
                              1,1, (la + lb)  ]).reshape(4,3)

        # their coordinate frame is rotated
        dP = np.matrix( [ -yvel, -xvel, avel] ).T

        w = J*dP

        # they use a goofy ordering

        flw = w[1,0]
        frw = w[0,0]
        blw = w[2,0]
        brw = w[3,0]

        front_cmd = self.segway_front.send_wheel_velocities( flw, frw )
        back_cmd  = self.segway_back.send_wheel_velocities(  blw, brw )
        
        if front_cmd == False or back_cmd == False:
            print 'Velocities out of spec: ',flw,frw,blw,brw
            print 'Perhaps we should consider sending the max command so motion doesn\'t hang.'

    def stop(self):
        self.set_velocity(0.,0.,0.)

    def get_status( self ):
        pass

class Segway_Properties():
    def __init__( self ):
        self._integrated_wheel_displacement = 24372/(2*math.pi) # countsp / rad
        #vv = 39 # countsp/sec / count_velocity    --- original
        #tv = 19 # countsp/sec / count_velocity    --- original
        vv = 39 # countsp/sec / count_velocity 
        tv = 22 # countsp/sec / count_velocity 
        self._V_vel = vv /self._integrated_wheel_displacement # rad/sec / count_velocity
        self._T_vel = tv /self._integrated_wheel_displacement # rad/sec / count_velocity

        #  w = | a   b | * V
        #      | c   d | 

        a = self._V_vel
        b = self._T_vel
        c = self._V_vel
        d = -self._T_vel
        self.A = np.matrix([a,b,c,d]).reshape(2,2)
        self.A_inv = 1/(a*b-c*d) * np.matrix([ d, -b, -c, a ]).reshape(2,2)

        #  w = A*V
        #  V = A_inv*w

        # in addition to what should command the platform velocities should be

        self._power_base_battery_voltage = 1/4. #(volt)/count
        self._ui_battery_voltage = .0125 #(volt)/count
        self._ui_battery_voltage_offset = 1.4 #volts

class Segway( Segway_Properties ):
    
    def __init__( self, side='front' ):
        """side should be 'front' or 'back'"""
        Segway_Properties.__init__(self)

        self.side = side
        self.segway = None
        self.connect()

        self.pitch_ang = 0
        self.pitch_rate = 0
        self.yaw_ang = 0
        self.yaw_rate = 0
        self.LW_vel = 0
        self.RW_vel = 0
        self.yaw_rate = 0
        self.servo_frames = 0
        self.integrated_wheel_displacement_left = 0
        self.integrated_wheel_displacement_right = 0
        self.integrated_for_aft_displacement = 0
        self.integrated_yaw_displacement = 0
        self.left_motor_torque = 0
        self.right_motor_torque = 0
        self.operational_mode = 0
        self.controller_gain_schedule = 0
        self.ui_battery_voltage = 0
        self.power_base_battery_voltage = 0
        self.velocity_commanded = 0
        self.turn_commanded = 0
        
    def connect(self):
        buses = usb.busses()
        segway_handle_list = []
        for bus in buses:
            for device in bus.devices:
                if device.idVendor == 1027 and device.idProduct == 59177:
                    h = device.open()
                    serial_num = int(h.getString(3,10))
                    if serial_num == 215:
                        if self.side == 'front':
                            print 'Connected to front segway'
                            self.segway = h
                            self.segway.claimInterface(0)
                    elif serial_num == 201:
                        if self.side == 'back':
                            print 'Connected to rear segway'
                            self.segway = h
                            self.segway.claimInterface(0)
                    else:
                        raise RuntimeError( 'Unknown_segway connected.' +
                                ' Serial Number is ',serial_num )
                        

    def calc_checksum(self, msg):
        checksum = 0
        for byt in msg:
            checksum = (checksum+byt)%65536
        checksum_hi = checksum >> 8
        checksum &= 0xff
        checksum = (checksum+checksum_hi)%65536
        checksum_hi = checksum >> 8
        checksum &= 0xff
        checksum = (checksum+checksum_hi)%65536
        checksum = (~checksum+1)&0xff
        return checksum

    def compose_velocity_cmd(self,linvel,angvel):
        byt_hi = 0x04
        byt_lo = 0x13

        if self.side == 'back':    # because the front segway is
            linvel = -linvel   # turned around
        linvel_counts = int(linvel)
        angvel_counts = int(angvel)

        if abs(linvel_counts)>1176:
            print 'connect.compose_velocity_cmd: Linear velocity is too high. counts: %d'%linvel
            return []

        if abs(angvel_counts)>1024:
            print 'connect.compose_velocity_cmd: Angular velocity is too high. counts: %d'%angvel
            return []

        usb_msg_fixed = [0xf0,0x55,0x00,0x00,0x00,0x00,byt_hi,byt_lo,0x00]
        can_vel_msg = [(linvel_counts>>8)&0xff,linvel_counts&0xff,(angvel_counts>>8)&0xff,angvel_counts&0xff,0x00,0x00,0x00,0x00]
        msg_send = usb_msg_fixed + can_vel_msg
        msg_send.append(self.calc_checksum(msg_send))

        return msg_send


    def send_command(self, linvel0, angvel0 ):
        msg0 = self.compose_velocity_cmd(linvel0,angvel0)
        if msg0 == []:
            return False

        for i in range(1):
            self.segway.bulkWrite(0x02,msg0)

    def send_wheel_velocities( self, lvel, rvel ):
        w = np.matrix([lvel,rvel]).T

        V = self.A_inv*w

        #print 'V = ',V
        xv = V[0,0]
        tv = V[1,0]

        #print 'Left vel = ',lvel
        #print 'Right vel = ',rvel
        #print 'Forward command = ',xv
        #print 'Turn command = ',tv
        
        return self.send_command( xv, tv )

    def parse_usb_cmd(self,msg):
        if len(msg) < 18:
            return

        if self.calc_checksum(msg[:-1]) != msg[-1]:
            #got garbage rejecting
            return

        id = ((msg[4]<<3)|((msg[5]>>5)&7))&0xfff

        data = msg[9:17]
        if id == 0x400:
            # unused
            pass
        elif id == 0x401:
            self.pitch_ang = self._2_bytes( data[0], data[1] )
            self.pitch_rate = self._2_bytes( data[2], data[3] )
            self.yaw_ang = self._2_bytes( data[4], data[5] )
            self.yaw_rate = self._2_bytes( data[6], data[7] )
        elif id == 0x402:
            self.LW_vel = self._2_bytes( data[0], data[1] )#/self._LW_vel
            self.RW_vel = self._2_bytes( data[2], data[3] )#/self._RW_vel
            self.yaw_rate = self._2_bytes( data[4], data[5] )
            self.servo_frames = self._2_bytes_unsigned( data[6], data[7] )
        elif id == 0x403:
            self.integrated_wheel_displacement_left = self._4_bytes(data[2],data[3],data[0],data[1])
            self.integrated_wheel_displacement_right = self._4_bytes(data[6],data[7],data[4],data[5])
            pass
        elif id == 0x404:
            self.integrated_for_aft_displacement = self._4_bytes(data[2],data[3],data[0],data[1])
            self.integrated_yaw_displacement = self._4_bytes(data[6],data[7],data[4],data[5])
        elif id == 0x405:
            self.left_motor_torque = self._2_bytes( data[0], data[1] )
            self.right_motor_torque = self._2_bytes( data[2], data[3] )
        elif id == 0x406:
            self.operational_mode = self._2_bytes( data[0], data[1] )
            self.controller_gain_schedule = self._2_bytes( data[2], data[3] )
            self.ui_battery_voltage = self._2_bytes( data[4], data[5] )*self._ui_battery_voltage + self._ui_battery_voltage_offset
            self.power_base_battery_voltage = self._2_bytes( data[6], data[7] )*self._power_base_battery_voltage
        elif id == 0x407:
            self.velocity_commanded = self._2_bytes( data[0], data[1] )#/self._LW_vel
            self.turn_commanded = self._2_bytes( data[2], data[3] )
        elif msg[1] == 0x00:
            # print 'heartbeat id = ',hex(msg[6]),hex(msg[7])
            pass
        elif id == 0x680:
            # CU Status Message
            pass
        else:
            #print 'no message parsed:', hex(id)
            pass

    def _2_bytes( self,high, low ):
        return struct.unpack('>h',chr(high)+chr(low))[0]

    def _2_bytes_unsigned( self,high, low ):
        return struct.unpack('>H',chr(high)+chr(low))[0]

    def _4_bytes( self,highhigh, lowhigh, highlow, lowlow ):
        return struct.unpack('>l',chr(highhigh)+chr(lowhigh)+chr(highlow)+chr(lowlow))[0]

    def clear_read(self):
        rd = self.segway.bulkRead(0x81,1000)

    def read(self):
        before = time.time()
        rd = self.segway.bulkRead(0x81,9*(18+2))
        msg = [(a & 0xff) for a in rd]

        i=0
        while 1:
            try:
                msg.pop(i*62)
                msg.pop(i*62)
                i += 1
            except IndexError:
                break

        #find the start 
        idx1 = msg.index(0xf0)

        if msg[idx1+18] != 0xf0:
            # if this is not the start of a message get rid of the bad start
            msg = msg[idx1+1:]
        else:
            # we found the start
            while len(msg) > 17:
                try:
                    single_msg = msg[idx1:idx1+18]
                    if (single_msg[1] == 0x55 and single_msg[2] == 0xaa) or single_msg[1] == 0x00:
                        self.parse_usb_cmd(single_msg)
                    msg = msg[18:]
                except IndexError:
                    break

    def print_feedback( self ):
        print 'self.integrated_wheel_displacement_left = ',self.integrated_wheel_displacement_left
        print 'self.integrated_wheel_displacement_right = ',self.integrated_wheel_displacement_right 
        print 'self.LW_vel = ',self.LW_vel
        print 'self.RW_vel = ',self.RW_vel
        print 'frame = ',self.servo_frames
        print 'self.velocity_commanded = ',self.velocity_commanded 
        print 'self.turn_commanded = ',self.turn_commanded 
        #print 'self.yaw_rate = ',self.yaw_rate


if __name__ == '__main__':

    import segway
    seg= segway.Segway()
    seg.clear_read()
#    send_command_fixed(segway_rear)
    rrates = []
    lrates = []

    for vel in range(31):
        linvel = 2000.*(vel)/30 - 1000.
        angvel = 0.

        start = time.time()
        last = start
        lastwrite = start
        lastread = start
        while 1:
    #        seg.send_command(200,0.0)
    #        seg.send_wheel_velocities(100.0,100.0)
            if time.time() - lastwrite > 0.1:
                print 'loop 1',time.time() - start
                #seg.send_command( linvel, angvel )
                lastwrite = time.time()
                #seg.send_platform_velocities( 0,0 )
                seg.send_wheel_velocities(1.,1.)
            if time.time() - lastread > 0.01:
                seg.read()
                lastread = time.time()
            #seg.print_feedback()
            if time.time() - start > 2.0:
                break

        left_start = seg.integrated_wheel_displacement_left
        right_start = seg.integrated_wheel_displacement_right
        first_points = time.time()

        while 1:
    #        seg.send_command(200,0.0)
    #        seg.send_wheel_velocities(100.0,100.0)
            if time.time() - lastwrite > 0.1:
                print 'loop 1.5',time.time() - start
                #seg.send_command( linvel, angvel )
                lastwrite = time.time()
                #seg.send_platform_velocities( 0,0 )
                seg.send_wheel_velocities(1.,1.)
            if time.time() - lastread > 0.01:
                seg.read()
                lastread = time.time()
            #seg.print_feedback()
            if time.time() - start > 5.0:
                break

        left_stop = seg.integrated_wheel_displacement_left
        right_stop = seg.integrated_wheel_displacement_right
        last_points = time.time()

        diff = last_points - first_points
        print 'Time: ',diff
        print 'left rate: ',(left_stop - left_start)/diff
        print 'right rate: ',(right_stop - right_start)/diff
        
        rrates.append(((right_stop - right_start)/diff,linvel))
        lrates.append(((left_stop - left_start)/diff,linvel))

        while 1:
            if time.time() - lastread > 0.01:
                seg.read()
                lastread = time.time()
            #seg.print_feedback()
            if seg.LW_vel ==0 and seg.RW_vel == 0:
                break

    print 'rrates:',rrates
    print 'lrates:',lrates


    import pylab
    x = []
    y = []
    x1 = []
    y1 = []
    for a, b in rrates:
        x.append(b)
        y.append(a)
    for a, b in lrates:
        x1.append(b)
        y1.append(a)
    pylab.plot(x,y,x1,y1)
    pylab.show()
    #while 1:
    #    print 'loop 2'
    #    seg.read()
        #seg.print_feedback()
#        print 'time: ', time.time()-start
#        if seg.LW_vel == 0 and seg.RW_vel == 0:
#            break
#    print 'total time: ', time.time()-start
#    print 'time to stop: ', time.time()-stop

#    msg = []
#    while True:
#        msg += list(handle_rmp0.bulkRead(0x81,100))

#        idx1 = msg.index(0xf0)
#        idx2 = msg.index(0xf0,idx1+1)
#        if idx2-idx1 == 18:
#            single_msg = msg[idx1:idx2]
#            if single_msg[1] == 0x55 and single_msg[2] == 0xaa:
#                parse_usb_cmd(single_msg)
#
#        msg = msg[idx2:]






