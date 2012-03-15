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
        self.status = None

    def zero(self):
        self.r_grip_press.rezero()
        self.l_grip_press.rezero()

    def record_pressures(self, file_name, arm, time = 10:)
        file_handle = open(file_name, 'w')
        self.zero()
        start = rospy.get_time()
        #might be better to do some condition, like gripper not moving or when arm is moved to side
        #while rospy.get_time()-start < time:
        while not self.status == 'moving somewhere again':
            print "saving data now ..."
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

    def status_callback(self, data):
        print "here is the status :", data
#        self.status = data.feedback.state
        self.status = data
        
if __name__ == '__main__':
    pw = PressureWriter()
    rospy.Subscriber('OverheadServer/actionlib/feedback/feedback/state', String, pw.status_callback)

    while not rospy.is_shutdown():
        if pw.status == 'closing gripper':
            pw.record_pressures('test_file', 0)
        rospy.sleep(0.02)

