#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_behaviors')
import rospy

from rfid_behaviors.srv import FlapEarsSrv, FlapperSrv

from threading import Thread

# All this guy does is call flap repeatedly until receiving a stop signal.

class Flapper( Thread ):
    def __init__( self, serv_name = 'rfid_orient/flap' ):
        Thread.__init__( self )
        self.should_run = True

        self.should_flap = False
        self.tagid = ''
        rospy.logout( 'flapper: initializing' )
        try:
            rospy.init_node('flapper_py')
        except:
            pass

        rospy.wait_for_service( serv_name )

        self.flap = rospy.ServiceProxy( '/rfid_orient/flap', FlapEarsSrv )

        self._service = rospy.Service( '/flapper/flap',
                                             FlapperSrv,
                                             self.process_service )

        rospy.logout( 'flapper: ready' )
        self.start()

    def run( self ):
        rospy.logout( 'flapper: running' )
        r = rospy.Rate( 10 )
        while self.should_run and not rospy.is_shutdown():
            if self.should_flap:
                self.flap( self.tagid, 0.0 )
            r.sleep()
        rospy.logout( 'flapper: exiting' )

    def stop( self ):
        self.should_run = False
        self.join(3)
        if (self.isAlive()):
            raise RuntimeError("ROS_M5e: unable to stop thread")            
            

    def process_service( self, req ):
        self.tagid = req.tagid
        self.should_flap = not self.should_flap
        return True
        
if __name__ == '__main__':
    fl = Flapper()
    rospy.spin()
