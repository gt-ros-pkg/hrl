#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_playpen')
import rospy
from pr2_playpen.srv import *
from robotis.lib_robotis import *


class Play:

    def __init__(self):
        dyn = USB2Dynamixel_Device('/dev/ttyUSB0')
        self.playpen = Robotis_Servo(dyn, 31)
        self.conveyor = Robotis_Servo(dyn,32)
        self.conveyor.init_cont_turn()
        rospy.init_node('playpen_server')
        s_play = rospy.Service('playpen', Playpen, self.move_playpen)
        s_conv = rospy.Service('conveyor', Conveyor, self.move_conveyor)    
        rospy.spin()

    def move_conveyor(self, data):
        delt_time = abs(data.distance)/2.39/0.685/0.0483*2/0.75
        if data.distance > 0:
            self.conveyor.set_angvel(-5)
        else:
            self.conveyor.set_angvel(5)
        rospy.sleep(delt_time)
        self.conveyor.set_angvel(0)
        print "move conveyor"
        return ConveyorResponse(1)

    def move_playpen(self, data):
        if data.command == 0:
            self.playpen.move_angle(0.28)
            print "closing playpen"
        elif data.command == 1:
            self.playpen.move_angle(1.90)
            print "opening playpen...releasing object"
        return PlaypenResponse(1)


if __name__== "__main__":
    go = Play()
