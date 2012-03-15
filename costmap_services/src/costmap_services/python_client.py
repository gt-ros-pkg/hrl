#! /usr/bin/python

import roslib; 
roslib.load_manifest('costmap_services')
import rospy

from costmap_services.srv import GetCost
from costmap_services.srv import ScoreTraj

import time
import numpy as np, math
from collections import deque
import thread

class CostmapServices():
    def __init__( self, accum = 1, ns='' ):
        try:
            rospy.init_node( 'costmap_services_node', anonymous = True )
        except: # Node probably already initialized elsewhere
            pass

        self.accum = accum
        self.buff = deque() # bit faster than just using lists.

        rospy.logout( 'CostmapServices: Waiting for services (ie. %s)' % (ns + '/costmap_services/cs/costmap_getcost'))
        rospy.wait_for_service( ns + '/costmap_services/cs/costmap_getcost' )
        rospy.wait_for_service( ns + '/costmap_services/cs/costmap_scoretraj' )

        self._getcost = rospy.ServiceProxy( ns + '/costmap_services/cs/costmap_getcost', GetCost )
        self._scoretraj = rospy.ServiceProxy( ns + '/costmap_services/cs/costmap_scoretraj', ScoreTraj )
        rospy.logout( 'CostmapServices: Ready.' )

    def scoreTrajectory( self, vx, vy, vtheta ):
        # Assumes that scoretraj is properly mutexed...
        try:
            score = self._scoretraj( vx, vy, vtheta )
            return score.cost
        except rospy.exceptions.ROSInterruptException, rospy.service.ServiceException: # rospy shutdown request during service call
            return -1.0 # Obstacle detect.

    def scoreTraj_PosHyst( self, vx, vy, vtheta ):
        # positive hysteresis: requires several (accum) -1.0s in a row before returning a -1.0
        # Helps prevent clients from exiting if there is a spurrious -1.0
        nv = self.scoreTrajectory( vx, vy, vtheta )

        self.buff.append( nv )
        if len( self.buff ) > self.accum:
            self.buff.popleft()

        filt = filter( lambda x: x != -1.0, self.buff )
        if filt:
            return np.mean( filt )
        else:
            return -1.0

    def scoreTraj_NegHyst( self, vx, vy, vtheta ):
        # negative hysteresis: requires several (accum) _non_ -1.0s in a row before returning a non -1.0
        # Helps prevent clients from exiting if there is a spurrious positive value
        nv = self.scoreTrajectory( vx, vy, vtheta )

        self.buff.append( nv )
        if len( self.buff ) > self.accum:
            self.buff.popleft()

        filt = filter( lambda x: x != -1.0, self.buff )
        if len(filt) == len(self.buff): # all are positive.
            return np.mean( filt )
        else:
            return -1.0

    def getMapCost( self, x, y ):
        cost = self._getcost( x, y )
        return cost.cost


    

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--ns', action='store', type='string', dest='ns',
                 help='Uses the namespace [ns] for services: /ns/approach_table/costmap_services/cs/...',
                 default = '')
    p.add_option('--xvel', action='store', type='float', dest='xvel',
                 help='Forward x velocity [default=0.1m/s]',
                 default = 0.1)
    opt, args = p.parse_args()

    trials = 100
    cs = CostmapServices( ns = opt.ns)
    
    t0 = time.time()
    res = [ cs.getMapCost( 0.0, 0.0 ) for i in xrange(trials) ]
    t1 = time.time()
    rospy.logout( '\tTotal time: %2.2f' % (t1 - t0))
    rospy.logout( '\tCall rate (hz): %2.2f' % (trials * 1.0 / (t1 - t0) ))

    r = rospy.Rate( 10 )
    rtime = []
    while not rospy.is_shutdown():
        t0 = time.time()
        score = cs.scoreTrajectory( opt.xvel, 0.0, 0.0 )
        #r.sleep()
        t1 = time.time()

        rtime.append( t1 - t0 )
        if len(rtime) > 5:
            rtime = rtime[1:]

        rospy.logout( 'ScoreTraj: %3.2f  Avg Rate: %2.2f' % ( score, 1.0 / np.mean( rtime ) ))


