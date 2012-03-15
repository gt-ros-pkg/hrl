#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_approach_table')
import rospy

from geometry_msgs.msg import Twist
import actionlib

import costmap_services.python_client as costmap
from pr2_approach_table.srv import ApproachSrv
from pr2_approach_table.msg import ApproachAction, ApproachResult, ApproachGoal

import numpy as np, math
from collections import deque
import os
from threading import Lock

def ctime():
    return rospy.Time.now().to_time()

def retfalse():
    return False

class Approach( ):
    def __init__( self, costmap_ns = '' ):
        rospy.logout('approach_node: Initializing')
        try:
            rospy.init_node('approach_node')
        except: # Node probably already initialized elsewhere
            pass

        # Create Costmap Services obj
        self.cs = costmap.CostmapServices( accum = 3, ns = costmap_ns ) # be really conservative!
        # Note: After moving, this will require accum * -1's before stopping.

        # Publish move_base command
        self._pub = rospy.Publisher( 'approach_cmd_vel', Twist )

        # Alterative ways to call using ROS services / actionlib
        self._service = rospy.Service( '/approach_table/move_forward_srv',
                                       ApproachSrv,
                                       self.service_request )
        self._as = actionlib.SimpleActionServer( '/approach_table/move_forward_act',
                                                 ApproachAction,
                                                 execute_cb = self.action_request )
        self._as.start()
    
        rospy.logout( 'approach_node: Service ready and waiting' )


    def service_request( self, req ):
        return self.run( forward_vel = req.forward_vel,
                         forward_mult = req.forward_mult )

    def action_request( self, goal ):
        def preempt_func():
            # self._as should be in scope and bound @ function def. (I think for python...)
            check = self._as.is_preempt_requested()
            if check:
                rospy.logout( 'approach_node: action_request preempted!' )
            return check

        rv = self.run( preempt_func = preempt_func,
                       forward_vel = goal.forward_vel,
                       forward_mult = goal.forward_mult )
        rospy.logout( 'approach_node: action_request received result %d' % int(rv) )

        if preempt_func():  # this is a relatively new addition
            rospy.logout('approach_node: returning actionlib state preempted.')
            self._as.set_preempted()
        elif rv == True:
            self._as.set_succeeded( ApproachResult( int(rv) ))
            

    def run( self, preempt_func = retfalse, forward_vel = 0.05, forward_mult = 1.0 ):
        # preempt_func => returns true when a preempt request is received, else false
        rospy.logout( 'approach_node: Run called for with values: \'%1.2f, %1.2f\'' % (forward_vel, forward_mult) )

        rospy.logout('approach_node: Running')

        r = rospy.Rate( 10 )
        def check_func():
            a = not rospy.is_shutdown()
            # forward_mult is a hack to let us get closer at the current forward_vel
            #   without reducing the footprint.
            b = self.cs.scoreTraj_PosHyst( forward_vel * forward_mult, 0.0, 0.0 ) != -1.0
            c = not preempt_func()
            return a and b and c
        
        while check_func():
            move_command = Twist()
            move_command.linear.x = forward_vel

            self._pub.publish( move_command )

            # Don't do this too fast (avoid unwanted buffering / rate issues)
            try:
                r.sleep()
            except rospy.ROSInterruptException: # rospy shutdown request received
                pass

        self._pub.publish( Twist() ) # Stop moving!

        if preempt_func():
            rospy.logout( 'approach_node: Preempt was requested. May not have finished.' )
            rospy.logout( 'approach_node: Exiting' )
            return False
        else:
            rospy.logout( 'approach_node: Exiting' )
            return True
        



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--ns', action='store', type='string', dest='ns',
                 help='Uses the namespace [ns] for services: /ns/approach_table/costmap_services/cs/...',
                 default = '')
    opt, args = p.parse_args()

    ap = Approach( costmap_ns = opt.ns )

    # There are (at least) three ways to call this code.
    # 1) Python:
    
    # ap.run( forward_vel = 0.05, forward_mult = 0.5 )

    # 2) ROS service:

    # rosservice call /approach_table/move_forward_srv 0.05 0.5

    # 3) ROS actionlib:

    # try:
    #     client = actionlib.SimpleActionClient( '/approach_table/move_forward_act', ApproachAction )
    #     client.wait_for_server()
    #     client.send_goal( ActionGoal( 0.05, 0.5 ))
    #     client.wait_for_result()
    #     print client.get_result()
    # except rospy.ROSInterruptException:
    #     print 'Program interrupted before completion'

    rospy.spin()

    


