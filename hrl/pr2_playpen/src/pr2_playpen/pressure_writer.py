#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_playpen')
import rospy
import hrl_pr2_lib.pressure_listener as pl
import cPickle

class PressureWriter:

    def __init__(self):
        rospy.init_node('pressure_writer')
        self.r_grip_press = pl.PressureListener(topic='/pressure/r_gripper_motor')
        self.l_grip_press = pl.PressureListener(topic='/pressure/l_gripper_motor')
        self.r_grip_data = []
        self.l_grip_data = []

    def zero(self):
        self.r_grip_press.rezero()
        self.l_grip_press.rezero()

    def record_pressures(self, file_name, arm, time = 10):
        file_handle = open(file_name, 'w')
        self.zero()
        start = rospy.get_time()
        #might be better to do some condition, like gripper not moving or when arm is moved to side
        while rospy.get_time()-start < time:
            self.r_grip_data.append(self.r_grip_press.get_pressure_readings())
            self.l_grip_data.append(self.l_grip_press.get_pressure_readings())
            rospy.sleep(0.05)
        cPickle.dump(data, file_handle)
        file_handle.close()

    def print_pressures(self):
        self.r_grip_press.rezero()
        right_tuple = self.r_grip_press.get_pressure_readings()
#        print "here's the right gripper :\n", right_tuple
#        print "here's the raw values : \n", self.r_grip_press.rmat_raw

if __name__ == '__main__':
    pw = PressureWriter()

    while not rospy.is_shutdown():
        pw.print_pressures()
        rospy.sleep(0.5)

