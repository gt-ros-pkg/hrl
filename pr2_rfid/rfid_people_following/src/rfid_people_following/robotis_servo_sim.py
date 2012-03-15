
# ROS imports
import roslib
roslib.load_manifest('rfid_people_following')
roslib.load_manifest('robotis')
import rospy

from std_msgs.msg import Float64
from robotis.srv import None_Float
from robotis.srv import None_FloatResponse
from robotis.srv import MoveAng
from robotis.srv import MoveAngResponse
from robotis.srv import None_Int32
from robotis.srv import None_Int32Response

import time
import numpy as np, math
from threading import Thread

def standard_rad(t):
    if t > 0:
        return ((t + np.pi) % (np.pi * 2))  - np.pi
    else:
        return ((t - np.pi) % (np.pi * -2)) + np.pi


class USB2Dynamixel_Device():
    ''' Class that manages serial port contention between servos on same bus
    '''
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
        self.dev_name = dev_name



class Robotis_Servo( Thread ):
    ''' Class to use a robotis RX-28 or RX-64 servo.
    '''
    def __init__(self, USB2Dynamixel, servo_id ):
        ''' USB2Dynamixel - USB2Dynamixel_Device object to handle serial port.
                            Handles threadsafe operation for multiple servos
            servo_id - servo ids connected to USB2Dynamixel 1,2,3,4 ... (1 to 253)
                       [0 is broadcast if memory serves]
        '''
        Thread.__init__( self )

        try:
            rospy.init_node( 'RobotisServo_'+str(servo_id) )
        except rospy.ROSException:
            pass

        self.should_run = True
        self.is_moving_flag = False
        self.curr_angle = 0.0
        self.rate = 0.0
        self.des_angle = 0.0
        self.max_speed = math.radians( 50 )


        self.dyn = USB2Dynamixel

        # ID exists on bus?
        self.servo_id = servo_id
        self.start()

    def is_moving(self):
        ''' returns True if servo is moving.
        '''
        return self.is_moving_flag

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        return 

    def read_angle(self):
        ''' returns the current servo angle (radians)
        '''
        return self.curr_angle

    def move_angle(self, ang, angvel=None, blocking=True):
        ''' move to angle (radians)
        '''
        self.rate = angvel
        self.des_angle = ang
        rospy.sleep( 0.05 )
        while blocking and self.is_moving_flag:
            rospy.sleep( 0.05 )
        return

    def run( self ):
        #last_time = time.time()
        last_time = rospy.Time.now().to_nsec() * 10**-9
        while self.should_run and not rospy.is_shutdown():
            rospy.sleep( 1.0 / 30.0 )
            new_time = rospy.Time.now().to_nsec() * 10**-9
            curr = self.curr_angle
            des = self.des_angle
            
            if curr == des:
                self.is_moving_flag = False
            else:
                ang_diff = standard_rad( des - curr )
                move_mag = np.sign( ang_diff ) * (new_time - last_time) * self.rate
                new_ang = curr + move_mag
                if np.abs( ang_diff ) < np.abs( move_mag ):
                    self.curr_angle = des
                    self.is_moving_flag = False
                else:
                    self.is_moving_flag = True
                    self.curr_angle = new_ang
            
            last_time = new_time
                

class ROS_Robotis_Server():
    # This class provides ROS services for a select few lib_robotis functions
    def __init__(self, servo = None, name = '' ):
        if servo == None:
            raise RuntimeError( 'ROS_Robotis_Servo: No servo specified for server.\n' )

        self.servo = servo
        self.name = name
        
        try:
            rospy.init_node( 'robotis_servo_' + self.name )
        except rospy.ROSException:
            pass

        #self.servo.disable_torque()

        rospy.logout( 'ROS_Robotis_Servo: Starting Server /robotis/servo_' + self.name )
        self.channel = rospy.Publisher('/robotis/servo_' + self.name, Float64)

        self.__service_ang = rospy.Service('/robotis/servo_' + name + '_readangle',
                                           None_Float, self.__cb_readangle)

        self.__service_ismove = rospy.Service('/robotis/servo_' + name + '_ismoving',
                                              None_Int32, self.__cb_ismoving)

        self.__service_moveang = rospy.Service('/robotis/servo_' + name + '_moveangle',
                                               MoveAng, self.__cb_moveangle)

    def __cb_readangle( self, request ):
        ang = self.update_server()
        return None_FloatResponse( ang )

    def __cb_ismoving( self, request ):
        status = self.servo.is_moving()
        return None_Int32Response( int(status) )

    def __cb_moveangle( self, request ):
        rospy.logout( 'RobotisServo: Move %s to angle %3.2f' % (self.name, math.degrees( request.angle )))
        ang = request.angle
        angvel = request.angvel
        blocking = bool( request.blocking )
        self.servo.move_angle( ang, angvel, blocking )
        rospy.logout( 'RobotisServo: Move %s Done' % (self.name))
        return MoveAngResponse()

    def update_server(self):
        ang = self.servo.read_angle()
        self.channel.publish( Float64(ang) )
        return ang


class ROS_Robotis_Poller( Thread ):
    # A utility class that will setup and poll a number of ROS_Robotis_Servos on one USB2Dynamixel
    def __init__( self, dev_name, ids, names ):
        Thread.__init__(self)

        self.should_run = True
        self.dev_name = dev_name
        self.ids = ids
        self.names = names

        for n in self.names:
            rospy.logout( 'ROS_Robotis_Servo: Starting Up /robotis/servo_' + n + ' on ' + self.dev_name )

        self.dyn = USB2Dynamixel_Device( self.dev_name )
        self.servos = [ Robotis_Servo( self.dyn, i ) for i in self.ids ]
        self.ros_servers = [ ROS_Robotis_Server( s, n ) for s,n in zip( self.servos, self.names ) ]

        rospy.logout( 'ROS_Robotis_Servo: Setup Complete on ' + self.dev_name )

        self.start()

    def run( self ):
        while self.should_run:
            [ s.update_server() for s in self.ros_servers ]
            rospy.sleep(0.001)

        for n in self.names:
            rospy.logout( 'ROS_Robotis_Servo: Shutting Down /robotis/servo_' + n + ' on ' + self.dev_name )

    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ROS_Robotis_Servo: unable to stop thread")




# Client should come from robotis package.  All services defined.

