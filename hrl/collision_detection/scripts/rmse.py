#!/usr/bin/env python

""" Provides tools to compute root mean squared error for each joint in JointTrajectoryControllerState messages, either on-line by listening to a speciefied topic - or offline by reading a bag file."""

import numpy as np
import sys

import roslib
roslib.load_manifest( 'gpr_controller' )
import rospy
import rosbag
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

__author__ = "Daniel Hennes"

class RMSEListener():
    error_sum = None
    num_msg = 0

    def __init__( self, topic ):
        np.set_printoptions( precision=3 ) # , suppress=True )

        rospy.init_node( 'rsme_joint_states' )
        rospy.Subscriber( topic, JointTrajectoryControllerState, 
                          self.cb_controller_state )
        rospy.spin()

    def cb_controller_state( self, msg ):
        if self.error_sum is None:
            self.error_sum = np.asarray( msg.error.positions ) ** 2
        else:
            self.error_sum += np.asarray ( msg.error.positions ) ** 2 
        self.num_msg += 1
        RMSE = np.sqrt( self.error_sum / self.num_msg )
        print RMSE

class RMSEBagReader():
    def __init__( self, fname = 'test.bag', topic = 'r_arm_controller/state' ):
        msgs = self.read_file( fname, topic )
        print self.rmse ( msgs )

    def read_file( self, fname, topic ):
        bag = rosbag.Bag( fname )
        msgs = []
        for topic, msg, t in bag.read_messages( topics = [topic] ):
            msgs.append( msg )
        bag.close()
        return msgs

    def rmse( self, msgs ):
        error_sum = None
        num_msg = 0
        for msg in msgs:
            if error_sum is None:
                error_sum = np.asarray( msg.error.positions ) ** 2
            else:
                error_sum += np.asarray ( msg.error.positions ) ** 2 
            num_msg += 1
        if error_sum is None:
            return None
        else:
            return np.sqrt( error_sum / num_msg )
    
if __name__ == '__main__':
    args = sys.argv
    args.pop(0)
    if len( args ) == 2 and args[0] == 'listen':
        RMSEListener( args[1] )
    elif len( args ) == 3 and args[0] == 'read':
        RMSEBagReader( args[1], args[2] )
    else:
        print "Usage: rmse <subcommand> [args]\n"
        print "Available subcommands:"
        print "\tlisten [topic]"
        print "\tread [filename] [topic]"
        
