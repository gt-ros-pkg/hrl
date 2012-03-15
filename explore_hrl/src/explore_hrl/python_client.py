#!/usr/bin/python

import roslib
roslib.load_manifest('explore_hrl')
import rospy

import actionlib
import explore_hrl.msg

def explore_client( radius ):
    client = actionlib.SimpleActionClient('explore', explore_hrl.msg.ExploreAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = explore_hrl.msg.ExploreGoal( radius = radius )

    # Sends the goal to the action server.
    client.send_goal(goal)

    r = rospy.Rate( 1 )
    t0 = rospy.Time.now().to_time()
    # while True:
    #     print 'State: ', client.get_state()
    #     r.sleep()

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    #return client.get_result()  # A FibonacciResult
    return client.get_state()

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-r', action='store', type='float', dest='radius', help='Sensing radius', default=2.0)
    opt, args = p.parse_args()
    
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('explore_client_py')
        result = explore_client( opt.radius )
        if result == actionlib.GoalStatus.SUCCEEDED:
            print 'SUCCEEDED'
        else:
            print 'FAILED'
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
