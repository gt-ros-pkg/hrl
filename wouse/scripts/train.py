#!/usr/bin/env python
import argparse
import random
import csv

import roslib; roslib.load_manifest('wouse')
import rospy
from geometry_msgs.msg import Vector3Stamped

#DEGREES = ['WEAK', 'AVERAGE', 'STRONG']
DEGREES = ['']
ACTIONS = ['WINCE', 'NOD', 'SHAKE', 'JOY', 'SUPRISE', 'FEAR', 'ANGER', 
            'DISGUST', 'SADNESS']

SYMBOLS = ["**"*20, "%%"*20, "^v"*20, '##'*20, '&&'*20, '$$'*20]

def choose_state():
    return random.choice(DEGREES), random.choice(ACTIONS)

class WouseTrainer(object):
    """ A class for printing random facial expression commands, and saving data from a topic of wouse movement data."""
    def __init__(self, fname):
        #path = roslib.substitution_args.resolve_args('$(find wouse)/data/')
        path = '../data/'
        file_out = path + fname + '.csv'
        self.csv_writer = csv.writer(open(file_out, 'ab'))
        title_row = ['Degree','Action','Time(s)','dx','dy']
        self.csv_writer.writerow(title_row)
        rospy.Subscriber('/wouse_movement', Vector3Stamped, self.movement_cb)

    def movement_cb(self, v3s):
        """Write a new line to the csv file for incoming data."""
        line = [self.degree, self.behavior, v3s.header.stamp.to_sec(),
                v3s.vector.x, v3s.vector.y]
        self.csv_writer.writerow(line)
        print "Writing"

    def run(self, run_time, switch_period):
        """Perform training for a given time, at a given rate."""
        total=int(round(run_time/switch_period))
        count = 0
        end_time = rospy.Time.now() + rospy.Duration(run_time)
        switch_rate = rospy.Rate(1/switch_period)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            count +=1
            self.degree = random.choice(DEGREES)
            self.behavior = random.choice(ACTIONS)
            bar = random.choice(SYMBOLS)
            print "\r\n"*10
            print bar
            print "%s/%s: %s  %s" %(count, total, self.degree, self.behavior)
            print bar
            print "\r\n"*10
            switch_rate.sleep()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description="A Script for Collecting Training Data to be used by the Support Vector Machine Classifier.")
    parser.add_argument('filename', 
                        default="training_data", 
                        help="filename for saving data")
    parser.add_argument('-l', '--length', 
                        default=630, 
                        type=float,
                        help="length (duration) of the whole training session")
    parser.add_argument('-d', '--duration', 
                        default=3.5, 
                        type=float,
                        help="duration of each facial expression")
    args = parser.parse_args()
    
    rospy.init_node('wouse_trainer')
    wt = WouseTrainer(args.filename)
    wt.run(args.length, args.duration)
    print "Training Session Completed"

