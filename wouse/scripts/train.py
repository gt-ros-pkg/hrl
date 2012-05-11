#!/usr/bin/env python
import argparse
import random
import csv
import math

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

    def run(self, act_list, switch_period):
        """Perform training for a given time, at a given rate."""
        total=len(act_list)
        switch_rate = rospy.Rate(1/switch_period)
        while len(act_list) > 0 and not rospy.is_shutdown():
            self.behavior = act_list.pop(random.randint(0,len(act_list)))
            bar = random.choice(SYMBOLS)
            print "\r\n"*10
            print bar
            print "%s:     %s" %(total-len(act_list), self.behavior)
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
    num = int(math.ceil(args.length/args.duration))
    num = num + num%len(ACTIONS)
    act_list = ACTIONS*num
    length = len(act_list)*args.duration
    
    rospy.init_node('wouse_trainer')
    wt = WouseTrainer(args.filename)
    wt.run(act_list, args.duration)
    print "Training Session Completed"

