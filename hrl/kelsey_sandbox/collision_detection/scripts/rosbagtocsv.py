#!/usr/bin/env python

import roslib
roslib.load_manifest( 'rospy' )
import rospy
roslib.load_manifest( 'rosbag' )
import rosbag
import numpy as np
import csv
import sys

def read_file( fname, topic ):
    bag = rosbag.Bag( fname )
    time = rospy.Time.from_sec(0.0)
    msgs = []
    for topic, msg, t in bag.read_messages( topics = [topic] ):
        if time > t:
            print( 'ERROR messages out of order' )
            return None
        time = t
        msgs.append( msg )
    bag.close()
    return np.asarray( msgs )

arm_joint_names = ['lr_shoulder_pan_joint',
                   'lr_shoulder_lift_joint',
                   'lr_upper_arm_roll_joint',
                   'lr_elbow_flex_joint',
                   'lr_forearm_roll_joint',
                   'lr_wrist_flex_joint',
                   'lr_wrist_roll_joint']


l_arm_joint_names = map(lambda s: s.replace('lr_', 'l_'), arm_joint_names )
r_arm_joint_names = map(lambda s: s.replace('lr_', 'r_'), arm_joint_names )

joint_names = [];

if len( sys.argv ) < 4:
    print( 'usage:\n\t rosbagtocsv in.bag out.csv [-r] [-l] [-j joint_name]' );
    sys.exit()

in_fname = sys.argv[1];
out_fname = sys.argv[2];

if sys.argv[3] == '-r':
    joint_names = r_arm_joint_names
elif sys.argv[3] == '-l':
    joint_names = l_arm_joint_names
elif sys.argv[3] == '-j':
    joint_names.append( sys.argv[4] )

print( 'searching for %s in %s ...'%(joint_names, in_fname) )

joint_states = read_file( in_fname, '/joint_states' )

if joint_states is None or not len( joint_states ) > 0:
    exit

print len( joint_states )
print joint_states[0].name

idx = map( joint_states[0].name.index, joint_names )

writer = csv.writer( open( out_fname, 'wb' ), delimiter = ',', quotechar = '|' )

print( 'writing %d rows to %s ...'%( len( joint_states ), out_fname) )

if len( joint_states ) < 1:
    print('bag file doesn not contain joint_states messages.')
    sys.exit(-1)

last_time = joint_states[0].header.stamp

for js in joint_states:
    pos =  map( lambda x: js.position[x], idx )
    vel = map( lambda x: js.velocity[x], idx )
    effort = map( lambda x: js.effort[x], idx )
    time = (js.header.stamp - last_time).to_sec()
    row = sum( [[time], pos, vel, effort], [] )
    writer.writerow( row )

print( 'done!' )

