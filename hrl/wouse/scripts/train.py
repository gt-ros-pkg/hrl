#!/usr/bin/env python
import sys
import random
import csv

import roslib; roslib.load_manifest('wouse')
import roslib.substitution_args
import rospy
from geometry_msgs.msg import Vector3Stamped

DEGREES = ['WEAK', 'AVERAGE', 'STRONG']
ACTIONS = ['WINCE', 'SMILE', 'FROWN', 'LAUGH', 'GLARE', 'NOD', 'SHAKE', 
            'REQUEST FOR BOARD', 'EYE-ROLL','JOY', 'SUPRISE', 'FEAR', 
            'ANGER', 'DISGUST', 'SADNESS']

SYMBOLS = ["**"*20, "%%"*20, "^v"*20, '##'*20, '&&'*20, '$$'*20]

def choose_state():
    return random.choice(DEGREES), random.choice(ACTIONS)

class WouseTrainer(object):
    def __init__(self, fname):
        path = roslib.substitution_args.resolve_args('$(find wouse)/data/')
        file_out = path + fname + '.csv'
        self.csv_writer = csv.writer(open(file_out, 'wb'))
        title_row = ['Degree','Action','Time(s)','dx','dy']
        self.csv_writer.writerow(title_row)
        rospy.Subscriber('/wouse_movement', Vector3Stamped, self.movement_cb)

    def movement_cb(self, v3s):
        line = [self.degree, self.behavior, v3s.header.stamp.to_sec(),
                v3s.vector.x, v3s.vector.y]
        self.csv_writer.writerow(line)
        print "Writing"

    def run(self, run_time=60., switch_period=4.):
        end_time = rospy.Time.now() + rospy.Duration(run_time)
        switch_rate = rospy.Rate(1/switch_period)
        while rospy.Time.now()< end_time and not rospy.is_shutdown():
            self.degree = random.choice(DEGREES)
            self.behavior = random.choice(ACTIONS)
            bar = random.choice(SYMBOLS)
            print "\r\n"*10
            print bar
            print self.degree + "  " + self.behavior
            print bar
            print "\r\n"*10
            switch_rate.sleep()

if __name__=='__main__':
    rospy.init_node('wouse_trainer')
    wt = WouseTrainer(sys.argv[1])
    wt.run()
    #wt.file_out.close()
    print "Training Session Completed"

