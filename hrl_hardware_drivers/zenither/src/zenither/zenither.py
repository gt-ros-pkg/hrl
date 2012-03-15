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
## code to control Animatics Smart Servos.
## author Cressel Anderson, Advait Jain, Hai Nguyen (Healthcare Robotics Lab, Georgia Tech.)

import serial
import sys,time
from time import sleep
from threading import RLock
import threading

#import util as ut
#import math_util as mut

import roslib; roslib.load_manifest('zenither')
import rospy
from hrl_msgs.msg import FloatArray
import hrl_lib.rutils as ru
import hrl_lib.util as ut

import zenither_config as zc

class ZenitherCollideException(Exception):
    def __init__(self):
        pass

class ZenitherClient:
    def __init__(self):
        self.listener = ru.FloatArrayListener('zenither_client', 'zenither_pose', 10)

    def pose(self, fresh=True):
        v = self.listener.read(fresh)
        return v[0,0]

class PoseBroadcast(threading.Thread):
    def __init__(self, zenither, frequency=100):
        threading.Thread.__init__(self)
        try:
            rospy.init_node('ZenitherPose')
            print 'ZenitherPose: ros is up!'
        except rospy.ROSException:
            pass

        self.frequency = frequency
        self.zenither  = zenither
        self.should_run = True
        self.pose       = self.zenither.get_position_meters() 

        name = 'zenither_pose'
        print 'PoseBroadcast: publishing', name, 'with type FloatArray'
        self.channel    = rospy.Publisher(name, FloatArray)
        self.start()

    def run(self): 
       print 'PoseBroadcast: started.'
       while self.should_run and not rospy.is_shutdown(): 
           sleep(1.0/self.frequency)
           self.pose = self.zenither.get_position_meters()
           self.channel.publish(FloatArray(None, [self.pose]))

    def stop(self):
       self.should_run = False 
       self.join(3) 
       if (self.isAlive()): 
           raise RuntimeError("PoseBroadcast: unable to stop thread")

class Zenither(object):
    """
       Basic Zenither interface
       Note: servo controller will crash if two different processes try to write to
             the serial interface at the same time
    """
    zero_bias = -0.004 #actual height=height reported by zenither+zero_bias
    servo = None

    def __init__(self, robot, max_command_retries=15, dev="/dev/robot/zenither",pose_read_thread=False):
        ''' robot- 'El-E' or 'HRL2'
            pose_read_thread - if you want a separate thread that constantly reads zenither height
                               and publishes using ROS.
        '''

        if robot not in zc.calib:
            raise RuntimeError('unknown robot')
        self.robot = robot

        self.dev = dev
        self.serial_lock = RLock()
        self.servo_start(max_command_retries)

        self.calib = zc.calib[robot]
        if self.calib['HAS_BRAKE']:
            self.brake_engaged = False
#            self.engage_brake()
            self.estop()

        # look at the log message for revision 287 for explanation.
        if self.get_variable('a') != '1':
            print 'WARNING: it seems that the Zenither has been power cycled'
            print 'WARNING: please set_origin for the zenither.'
            self.calibrated = False
        else:
            self.calibrated = True

        # This might cure the sound from the zenither. -- Advait Dec26,2008
        # Changes get_position_meters of Dec26,2008 to
        # get_motor_temperature_F. since get_position_meters raises
        # exception if the zenither is uncalibrated.   -- Advait Nov14,2009
        for i in range(10):
            self.get_motor_temperature_F()

        if pose_read_thread:
            self.broadcast()

    def broadcast(self):
        self.broadcaster = PoseBroadcast(self)

    def servo_start(self, max_command_retries):
        self.serial_lock.acquire()
        try:
            #self.servo = serial.Serial("/dev/robot/zenither", timeout=0)
            self.servo = serial.Serial(self.dev, timeout=0)
        except (serial.serialutil.SerialException), e:
            raise RuntimeError("Zenither: Serial port not found!\n")
        self.max_command_retries = max_command_retries
        if(self.servo == None):
            raise RuntimeError("Zenither: Serial port not found!\n")

        #Without this platform will fall while zenith'ing
        self.servo.timeout = 1
        check = self.get_status_byte()
        if(check == ''):
            raise RuntimeError("Animatics Servo isn't responding!\n")
            self.servo.close()
            self.servo = None
        else:
            t = self.get_motor_temperature_F()
            if self.get_motor_temperature_F() >= 140.0:
                t = self.get_motor_temperature_F()
                mesg = "Animatics Servo is too hot: " + t.__str__() + "F"
                raise RuntimeError(mesg)
                self.servo.close()
                self.servo = None
            self.inputs_init()
            self.servo.write('o=0\n')        #set origin
            self.servo.write('E=10000\n')        #set error limit

            #These parameters are only used in position mode.
            self.servo.write('A=150\n')      #set accel
            self.servo.write('V=100000\n')    #set velocity
            self.servo.write('P=2000\n')        #set position
            self.servo.write('KG=50\n')      #set gravity compensation
            self.servo.write('F\n')          #apply new PID constants

        self.last_position = 0.0
            
        self.serial_lock.release()

    def reset_serial(self):
        self.serial_lock.acquire()
        self.servo.flushInput()
        self.servo.flushOutput()
        self.servo.close()
        self.servo_start(self.max_command_retries)
        self.serial_lock.release()

    def __del__(self):
        self.serial_lock.acquire()
        self.servo.close()
        self.serial_lock.release()

    def __lock_write(self, line):
        self.serial_lock.acquire()
        self.servo.write(line)
        #print 'Servo send: ',line
        self.serial_lock.release()

    def reset(self):
        self.__lock_write("Z\n")

    def set_timeout(self,timeout):
        self.serial_lock.acquire()
        self.servo.timeout = timeout
        self.serial_lock.release()

    def turn_off(self):
        self.__lock_write("OFF\n")

    def get_status_byte(self):
        self.serial_lock.acquire()
        self.servo.write("RS\n")
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return s

    def get_status_word(self):
        self.serial_lock.acquire()
        self.servo.write("RW\n")
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return s

    def report_and_clear(self):
        self.serial_lock.acquire()
        self.servo.write("RCS1\n")
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return s

    def get_mode(self):
        self.serial_lock.acquire()
        self.servo.write("RMODE\n")
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return s

    def use_position_mode(self):
        if self.calib['HAS_BRAKE']:
            self.engage_brake_on_without_trajectory()
        self.__lock_write("MP\n")

    def use_velocity_mode(self):
        if self.calib['HAS_BRAKE']:
            self.engage_brake_on_without_trajectory()
        self.__lock_write("MV\n")

    def use_torque_mode(self):
        self.__lock_write("MT\n")

    def get_factor(self,type='vel_factor'):
        #return self.calib[self.robot][type]
        return self.calib[type]

    def set_acceleration(self,a):
        '''Sets the acceleration in meters/sec^2'''
        factor = self.get_factor(type='acc_factor')
        a  = int(round(abs(a/ factor)))
        cmd = "A="+str(a)+"\n"
        self.__lock_write(cmd)

    def set_velocity(self,v):
        '''Sets the velocity in meters/sec'''
        factor = self.get_factor(type='vel_factor')
        v  = int(round(v/factor))
        cmd ="V="+str(v)+"\n"
        self.__lock_write(cmd)

    def set_origin(self):
        self.__lock_write("O=0\n")
        time.sleep(0.1)
        self.__lock_write("a 1\n") # zenither is now calibrated.
        self.calibrated = True

    def go(self):
        self.__lock_write("G\n")

    # This will cut power to the device causing it to fall
    # destroying the robot!
    # def stop(self):
    #     self.servo.write("X\n")

    def estop(self):
        self.__lock_write("S\n")
        if self.calib['HAS_BRAKE']:
            self.engage_brake_on_without_trajectory()

    def set_pos_absolute(self,p):
        '''Sets the absolute position in meters'''
        factor = self.get_factor(type='pos_factor')
        p = int(round(p/factor))
        cmd = "P="+str(p)+"\n"
        self.__lock_write(cmd)

    def set_pos_relative(self,p):
        '''Sets the relative position in meters'''
        factor = self.get_factor(type='pos_factor')
        p = int(round(p/factor))
        cmd = "D="+str(p)+"\n"
        self.__lock_write(cmd)

    ## reads a variable. (page 27 of animatics manual)
    # e.g. var = 'a' will read the variable a.
    def get_variable(self,var):
        self.serial_lock.acquire()
        self.servo.write('R'+var+'\n')
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return s.replace('\r','')

    def get_position(self):
        self.serial_lock.acquire()
        #print 'zenither.get_position: lock acquired'
        self.servo.write("RP\n")
        s = self.servo.readline(eol='\r')
        self.serial_lock.release()
        #print 'zenither.get_position: lock released'
        return s

    def get_position_meters(self):
        if self.calibrated == False:
            self.estop()
            raise RuntimeError('Zenither not calibrated. Please set_origin.')
        val = self.get_position().replace('\r','')
        while val == '':
            #print 'waiting on new position value'
            val = self.get_position().replace('\r','')
        #print 'val = ',val

        factor = self.get_factor(type='pos_factor')
        p = float(val)*factor + self.calib['ZERO_BIAS']
        return p

    def set_torque(self,t):
        self.__lock_write("T="+str(t)+"\n")

    # FOR TORQUE CONTROL MODE
    #
    # The motor does not operate as the manual leads one to
    # believe. Torque control mode does not control torque. It is a
    # very rough approximation to torque control. It is not possible
    # to have the motor operate with a specified torque if an external
    # torque is being applied to the motor. For instance:
    #
    # I have -1.0Nm applied to the shaft, and I want to keep the shaft
    # stationary. I should apply 1.0Nm to result in a net torque of
    # 0Nm (no rotation). This opperates as close to what is expected.
    #
    # If I want -1.0Nm to be the net force, then I would apply
    # 0Nm. The torque is not nearly 0Nm, but is continuous from the
    # positive regime to 0Nm.
    #
    # If I want -1.1Nm to be the net force, then I want to have the
    # motor apply -0.1Nm to the shaft. This is not possible with the
    # motor.
    # 
    # In the presence of the external torque it is impossible to
    # command a small torque in the same direction from the motor. I
    # beleive this is due to the internal control techniques of the
    # motor drive, specifically open loop control of torque. If
    # Animatics were to actively control the motor current, continuous
    # control of torque would be possible even in the presence of
    # external torques.
    # 
    # Additionally, I have explored other techniques to accurately
    # control motor current/torque such as monitoring current. The
    # only current available for monitoring is the input current to
    # the device. It includes power consumed by the drive and seems
    # unsuitible for motor control.
    # 
    # The positive regime works fine, but the transition can be
    # atrocious under external torques. Position mode is the only
    # possible control mode for smooth quick motion downward for the
    # zenither. I suspect we could do better for collision detection
    # with the actuator in position control mode, but it does not
    # involve the use of torque mode. This is the case unless
    # operating exclusively in the positive torque mode is an
    # acceptible solution. This is the case for the SM2315DT and I
    # suspect all SmartMotor servos from Animatics.


    def zenith(self, torque=None):
        if torque == None:
            torque=self.calib['zenith_torque']

        #print "Zenither: going up at torque", torque
        self.serial_lock.acquire()
        if self.robot == 'El-E':
            self.servo.write('KGOFF\n')       #disable gravity compensation
        self.use_torque_mode()
        factor = self.get_factor(type='pos_factor')
        self.set_torque(int(torque*factor/abs(factor))) #use factor to determine sign
        if self.calib['HAS_BRAKE']:
            self.disengage_brake()
        #print 'setting torque ',int(torque*factor/abs(factor)) #use factor to determine sign
        self.serial_lock.release()

    def nadir(self, torque=None):
        self.serial_lock.acquire()
        if torque == None:
            torque = self.calib['nadir_torque']
        if self.robot == 'El-E':
            self.servo.write('KGOFF\n')       #disable gravity compensation
        self.use_torque_mode()
        factor = self.get_factor(type='pos_factor')
        #print 'torque', int(torque*factor/abs(factor))
        self.set_torque(int(torque*factor/abs(factor))) #use factor to determine sign)
        if self.calib['HAS_BRAKE']:
            self.disengage_brake()
        else:
            print 'no brake'
        #print 'setting torque ',int(torque*factor/abs(factor)) #use factor to determine sign
        self.serial_lock.release()

    def torque_move_position(self,height,speed='fast',exception=True):
        ''' speed - 'fast','slow','snail'
            exception - if True then throws exception in case of collision.
            moves till height or if it detects collision.
            height - height of the bottom point of carriage above the ground.
        '''
        #if self.calib['robot'] == 'El-E':
        #    raise RuntimeError('Advait, please check this function on El-E')

        position_zenither_coord = height - self.calib['ZERO_BIAS']
        position_zenither_coord = self.limit_position(position_zenither_coord)

        if height>self.calib['max_height']:
            height = self.calib['max_height']

        if height<self.calib['min_height']:
            height = self.calib['min_height']
        
        curr_height = self.get_position_meters()

        if abs(curr_height-height)<0.002:
            return

        if curr_height>height:
            if speed=='fast':
                torque=self.calib['down_fast_torque']
            elif speed=='slow':
                torque=self.calib['down_slow_torque']
            elif speed=='snail':
                torque=self.calib['down_snail_torque']
            else:
                print 'zenither.torque_move_position: unknown speed- ',speed
                sys.exit()
            self.torque_go_down(height,torque)

        if curr_height<height:
            if speed=='fast':
                torque=self.calib['up_fast_torque']
            elif speed=='slow':
                torque=self.calib['up_slow_torque']
            elif speed=='snail':
                torque=self.calib['up_snail_torque']
            else:
                print 'zenither.torque_move_position: unknown speed- ',speed
                sys.exit()
            self.torque_go_up(height,torque)
        error = height-self.get_position_meters()
        if abs(error)>0.005 and exception == True:
            raise ZenitherCollideException

    def torque_go_down(self,height,start_torque,dist_list=[0.15,0.05]):
        ''' dist list - how much distance to move slow and snail.
            height - is in zenither coord. not height of carriage above the
                     ground.
        '''
        #if self.calib['robot'] == 'El-E':
        #    raise RuntimeError('Advait, please check this function on El-E')

        slow_torque = self.calib['down_slow_torque']
        if self.calib['robot'] == 'HRL2':
            slow_torque = self.calib['down_fast_torque']
        snail_torque = self.calib['down_snail_torque']

        h_start = self.get_position_meters()
        h_start = self.get_position_meters()

        if (h_start-height) < (dist_list[1]+0.01):
            self.nadir(snail_torque)
            current_torque = snail_torque
        elif (h_start-height) < (dist_list[0]+0.02):
            self.nadir(slow_torque)
            current_torque = slow_torque
        else:
            self.nadir(start_torque)
            current_torque = start_torque

        h_start = self.get_position_meters()
        h_start = self.get_position_meters()

        while h_start > height:
            time.sleep(0.01)
            h_now = self.get_position_meters()
            if (h_now-height) < dist_list[1]:
                if current_torque != snail_torque:
                    self.nadir(snail_torque)
                    current_torque = snail_torque
            elif (h_now-height) < dist_list[0]:
                if current_torque != slow_torque:
                    self.nadir(slow_torque)
                    current_torque = slow_torque

            if h_now == h_start:
                # stopped moving due to a collision
                break
            h_start = h_now

        self.estop()
        #print self.get_position_meters()

    def torque_go_up(self,height,start_torque,dist_list=[0.15,0.05]):
        ''' dist list - how much distance to move slow and snail.
            height - is in zenither coord. not height of carriage above the
                     ground.
        '''
        #if self.calib['robot'] == 'El-E':
        #    raise RuntimeError('Advait, please check this function on El-E')

        slow_torque  = self.calib['up_slow_torque']
        snail_torque = self.calib['up_snail_torque']

        h_start = self.get_position_meters()
        h_start = self.get_position_meters()

        if (height-h_start) < (dist_list[1]+0.01):
            self.zenith(snail_torque)
            current_torque = snail_torque
        elif (height-h_start) < (dist_list[0]+0.02):
            self.zenith(slow_torque)
            current_torque = slow_torque
        else:
            self.zenith(start_torque)
            current_torque = start_torque

        h_start = self.get_position_meters()
        h_start = self.get_position_meters()

        while h_start < height:
            h_now = self.get_position_meters()
            time.sleep(0.01)
            if (height-h_now) < dist_list[1]:
                if current_torque != snail_torque:
                    self.zenith(snail_torque)
                    current_torque = snail_torque
            elif (height-h_now) < dist_list[0]:
                if current_torque != slow_torque:
                    self.zenith(slow_torque)
                    current_torque = slow_torque

            if h_now == h_start:
                # stopped moving due to a collision
                break
            h_start = h_now

        self.estop()
        #print self.get_position_meters()

    def go_height_blocking(self, height):
        """ takes zenither up or down to height (moves slowly)
            blocking call
        """
        curHeight = self.get_position_meters()
        height = height-curHeight
        self.go_relative_blocking(height)

    def go_height(self, z, function=None, down_torque=0, up_torque=200):
        if self.robot != 'El-E':
            raise RuntimeError(
                'This function is unemplemented on robot\'s other than El-E')
        #z = mut.bound(z,0.0, .89)
        z = ut.bound(z,0.0, .89)

        initial_direction = None
        #if not ut.approx_equal(z, self.broadcaster.pose):
        if not ut.approx_equal(z, self.get_position_meters()):
            #if z > self.broadcaster.pose:
            if z > self.get_position_meters():
                self.zenith(torque = up_torque)
                initial_direction = 'up'
            else:
                self.zenith(torque = down_torque)
                initial_direction = 'down'
        else:
            return

        there         = False
        #while not ut.approx_equal(z, self.broadcaster.pose):
        while not ut.approx_equal(z, self.get_position_meters()):
            if function != None:
                #ret = function(self.broadcaster.pose)
                ret = function(self.get_position_meters())
                if ret:
                    break

            #if initial_direction == 'up' and self.broadcaster.pose > z:
            if initial_direction == 'up' and self.get_position_meters() > z:
                break

            if initial_direction == 'down' and self.get_position_meters() < z:
                break

        self.estop()

    def limit_position(self, position):

        position = ut.unipolar_limit(position, 
                                     self.calib['POS_MAX'])
        return position

    def limit_velocity(self,velocity):
        if velocity == None:
            velocity = self.calib['VEL_DEFAULT']
            return velocity

        velocity = ut.unipolar_limit(velocity, 
                                     self.calib['VEL_MAX'])

        return velocity

    def limit_acceleration(self,acceleration):
        if acceleration == None:
            acceleration = self.calib['ACC_DEFAULT']
            return acceleration

        acceleration = ut.unipolar_limit(acceleration, 
                                     self.calib['ACC_MAX'])

        return acceleration

    def move_position(self,position, velocity=None , acceleration=None, 
                      blocking = True, relative=False, approximate_pose = True):

        if self.robot != 'El-E' and self.robot != 'test_rig':
            raise RuntimeError(
                'This function is unemplemented on robot\'s other than El-E, test_rig')
        if self.calib['HAS_BRAKE']:
            self.disengage_brake()

        if position>self.calib['max_height']:
            position = self.calib['max_height']

        if position<self.calib['min_height']:
            position = self.calib['min_height']

        self.use_position_mode()       #position mode

        if relative:
            position = self.get_position_meters() + position
        
        position_zenither_coord = position - self.calib['ZERO_BIAS']
        position_zenither_coord = self.limit_position(position_zenither_coord)
        
        acceleration = self.limit_acceleration( acceleration )
        velocity = self.limit_velocity( velocity )

        p = self.get_position_meters()

        if p >= position:
            #needs to go down
            #print 'needs to go down'
            self.serial_lock.acquire()
            self.servo.write('KGON\n')       #enable gravity compensation
            self.serial_lock.release()
            if velocity > 4.0 or velocity < -4.0:
                velocity = 4.0               #not faulting here, just caution

        else:
            #needs to go up
            #print 'needs to go up'
            self.serial_lock.acquire()
            self.servo.write('KGOFF\n')       #disable gravity compensation
            self.serial_lock.release()
            if velocity > 3.0 or velocity < -3.0:
                velocity = 3.0               #ascent faults if velocity is too high

        self.set_pos_absolute(position_zenither_coord)

        self.set_velocity(velocity)
        self.set_acceleration(acceleration)
        self.go()

        if blocking:
            have_not_moved = 0
            #current_pose = self.broadcaster.pose
            current_pose = self.get_position_meters()
            last_pose = None
            print 'zenither.move_position: blocking...'
            while abs(current_pose - position) > 0.01:
                time.sleep(200/1000.0)
                current_pose = self.get_position_meters()
                #print abs(current_pose - position)
            print 'zenither.move_position: done'

        if self.calib['HAS_BRAKE']:
            self.engage_brake()


    def check_mode(self):
        pass

    def checkbits(self):
        self.serial_lock.acquire()
        self.servo.write("c=UCI\n")
        self.servo.write("Rc\n")
        c = self.servo.readline(eol='\r')
        self.servo.write("d=UDI\n")
        self.servo.write("Rd\n")
        d = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return [c,d]

    def get_motor_torque(self):
        self.serial_lock.acquire()
        self.servo.write("t=T\n")
        self.servo.write("Rt\n")
        t = self.servo.readline(eol='\r')
        self.serial_lock.release()
        return t

    def get_motor_temperature_F(self):
        return self.get_motor_temperature_C()*9.0/5.0 + 32.0

    def get_motor_temperature_C(self):
        self.serial_lock.acquire()
        self.servo.write("t=TEMP\n")
        self.servo.write("Rt\n")
        t = float(self.servo.readline(eol='\r'))
        self.serial_lock.release()
        return t

    def inputs_init(self):
        self.serial_lock.acquire()
        self.servo.write("UCI\n")#Set pin C to input
        self.servo.write("UDI\n")#Set pin D to input
        self.serial_lock.release()

    def engage_brake(self):
        self.serial_lock.acquire()
        if self.brake_engaged == False:
            self.servo.write("BRKENG\n") #Engage the brake
            self.brake_engaged = True
        self.serial_lock.release()

    def disengage_brake(self):
        self.serial_lock.acquire()
        if self.brake_engaged:
            self.servo.write("BRKRLS\n") #Disengage the brake
            self.brake_engaged = False
        self.serial_lock.release()

    def engage_brake_when_not_servoing(self):
        self.serial_lock.acquire()
        if self.brake_engaged == False:
            self.servo.write("BRKSRV\n") #Engage the brake when not
                                         #servoing
            self.brake_engaged = True
        self.serial_lock.release()

    def engage_brake_on_without_trajectory(self):
        self.serial_lock.acquire()
        if self.brake_engaged == False:
            self.servo.write("BRKTRJ\n") #Engage the brake when not
                                         #executing a trajectory
            self.brake_engaged = True
        self.serial_lock.release()


#    def test_cam_mode(self):
#        cmd='MF4\n'
#        self.__lock_write(cmd)
#        cmd='BASE=10000\n'
#        self.__lock_write(cmd)
#        cmd='SIZE=4\n'
#        self.__lock_write(cmd)
#        cmd='E=10000\n'
#        self.__lock_write(cmd)
#        cmd='aw[0] 0 5000 10000.\n'
#        self.__lock_write(cmd)
#        cmd='MC\n'
#        self.__lock_write(cmd)
#        print 'Sleeping before issuing the go command.'
#        time.sleep(1.)
#        cmd='G\n'
#        self.__lock_write(cmd)
#        print 'sent the go command.'




