#!/usr/bin/python
import roslib
roslib.load_manifest( 'gpr_controller' )
import rospy
import rosbag

import matplotlib.pyplot as plt

from numpy import asarray, zeros
import numpy as np

import operator

joint_names = ['r_shoulder_pan_joint',
               'r_shoulder_lift_joint',
               'r_upper_arm_roll_joint',
               'r_elbow_flex_joint',
               'r_forearm_roll_joint',
               'r_wrist_flex_joint',
               'r_wrist_roll_joint']


def prod( lst ):
    return reduce( operator.mul, lst )


def read_file( fname, topic ):
    bag = rosbag.Bag( fname )
    states = []
    time = rospy.Time.from_sec(0.0)
    for topic, msg, t in bag.read_messages( topics = [topic] ):
        if time > msg.header.stamp:
            print 'messages out of order'
            return None
        time = msg.header.stamp
        states.append( msg )
    bag.close()
    return states
    
def generate_samples( joint_states ):
    joint_ids = None
    x = []
    for s in joint_states:
        if joint_ids is None:
            joint_ids = map( s.name.index, joint_names )
        
        p = asarray( map( lambda x: s.position[x], joint_ids ) )
        v = asarray( map( lambda x: s.velocity[x], joint_ids ) )
        a = np.zeros( (len( joint_names )) )
        e = asarray( map( lambda x: s.effort[x], joint_ids ) )
        x.append( np.concatenate( (p,v,a,e) ) )

    x = asarray(x).transpose()

    (_, N) = x.shape
    X = zeros( (len( joint_names ) * 4, N-1) )
    for i in range( 7 ): 
        X[i] = x[i][1:]
    
    # compute acceleration
    for i in range( 7, 14 ):
        X[i] = x[i][1:]
        X[i+7] = x[i][1:]-x[i][:-1]

    j = 1
    plt.plot(X[0 + j])
    plt.plot(X[7 + j])
    plt.plot(X[14 + j])
    plt.show()



def split_per_joint( states ):
    for s in states:
        print s.header.stamp.to_sec()#, s.header.stamp.to_tons()


def gen_trainingset( joint_states, controller_states ):
    U = []
    X = []
    # init
    while controller_states[0].header.stamp <= joint_states[0].header.stamp:
        # print controller_states[0].header.stamp, joint_states[0].header.stamp
        controller_states.pop(0)

    while len( controller_states ) > 0 and len( joint_states ) > 0:
        U.append( asarray(joint_states[0].effort) )
        x = np.concatenate( ( asarray(controller_states[0].actual.positions ), 
                           asarray( controller_states[0].actual.velocities ) ) )
                 # controller_states[0].actual.accelerations]
                 # , [] )
        X.append(x.flatten())
        #print controller_states[0].header.stamp - joint_states[0].header.stamp
        controller_states.pop(0)
        joint_states.pop(0)
    
    print len( U )
    print len( X )

    U = np.transpose(U)
    X = np.transpose(X)

    return U, X

    # p = X[0]

    # for i in range( 7 ):
    #     plt.subplot(7, 1, i+1 )
    #     plt.plot(U[i])

    # plt.show()
    

def MSE( states ):
    error = zeros( len( states[0].error.positions ) )
    for s in states:
        error += asarray( s.error.positions ) ** 2
    return error / len( states )

def target_mean( states ):
    mean = zeros( len( states[0].actual.positions ) )
    for s in states:
        mean += asarray( s.actual.positions )
    return mean / len( states )
    

def target_variance( states ):
    mean = target_mean( states )
    var = zeros( len( states[0].actual.positions ) )
    for s in states:
        var += (asarray( s.actual.positions ) - mean) ** 2
    return var / len( states )

def nMSE( states ):
    return MSE( states ) / target_variance( states )

fname = 'test.bag'
controller_states = read_file( fname, 'r_arm_controller/state' )
joint_states = read_file( fname, 'joint_states')

print '#datapoints controller %i'%len( controller_states ) 
print '#datapoints joint_states %i'%len( joint_states ) 

generate_samples( joint_states )

# gen_trainingset( joint_states, controller_states )

# print 'mean: ', target_mean( controller_states )
# print 'var: ', target_variance( controller_states )
# print 'MSE: ', MSE( controller_states )
# print 'nMSE: ', nMSE( controller_states )


