#! /usr/bin/python
import roslib
roslib.load_manifest('rfid_behaviors')
roslib.load_manifest('pr2_msgs')
roslib.load_manifest('std_msgs')
import rospy

from pr2_msgs.msg import PressureState
from std_msgs.msg import Float64
from rfid_behaviors.srv import FloatFloat_Int32

import numpy as np, math
import time, string

def default_mag_func( x ):
    return np.sum( np.abs( x ))

class TactileSensor( ):
    def __init__( self, topic = '/pressure/r_gripper_motor', 
                  mag_func = default_mag_func ):
        rospy.logout( 'tactile_sensor: Initializing' )
        try:
            rospy.init_node( 'tactile_sensor' )
        except:
            pass

        self.mag_func = mag_func
        self.left = None
        self.right = None
        self.l_bias = None
        self.r_bias = None
        self.topic = topic

        self.pub = rospy.Publisher( '/readings/' + string.replace(topic,'/','_') + '_mag', Float64 )
        self.service = rospy.Service( '/readings/' + string.replace(topic,'/','_') + '_serv', 
                                      FloatFloat_Int32, 
                                      self.thresh_service )
        self.reg_sensor( )

        while self.left == None or self.right == None:
            time.sleep( 0.1 )

        self.unreg_sensor()

        rospy.logout( 'tactile_sensor: Ready' )


    def reg_sensor( self ):
        self.sub = rospy.Subscriber( self.topic, PressureState, self.cb )
        time.sleep( 0.3 )

    def unreg_sensor( self ):
        self.sub.unregister()


    def cb( self, msg ):
        self.left = np.array( list( msg.l_finger_tip ), dtype=float )
        self.right = np.array( list( msg.r_finger_tip ), dtype=float )

        if self.l_bias == None or self.r_bias == None:
            self.l_bias = np.zeros( len( self.left ))
            self.r_bias = np.zeros( len( self.right ))

        self.l_read = np.copy( self.left - self.l_bias )
        self.r_read = np.copy( self.right - self.r_bias )

        self.mag = self.mag_func( np.append( self.l_read, self.r_read ))
        #print np.append( self.l_read, self.r_read )
        self.pub.publish( self.mag )
        

    def bias( self ):
        self.reg_sensor()
        rospy.logout( 'tactile_sensor: Biasing' )
        self.l_bias = np.copy( self.left )
        self.r_bias = np.copy( self.right )
        self.unreg_sensor()
        return True

    def read( self ):
        return np.copy( self.l_read ), np.copy( self.r_read )

    def thresh_service( self, request ):
        self.thresh_detect( request.rotate, request.displace ) # yeah, jacked up names
        return int( True )

    def thresh_detect( self, threshold, timeout = 100.0 ):
        rospy.logout( 'tactile_sensor: Threshold detector activated: %3.2f, timeout: %d' % (threshold, timeout))
        self.bias()
        self.reg_sensor()
        t0 = time.time()
        t_diff = time.time() - t0
        lp = t0 # Printing status messages
        while self.mag < threshold and t_diff < timeout:
            if time.time() - lp > 1.0:
                lp = time.time()
                rospy.logout( 'tactile_sensor: Threshold still undetected' )
            time.sleep( 0.05 )
            t_diff = time.time() - t0
        self.unreg_sensor()
        rospy.logout( 'tactile_sensor: Detected (or timeout)' )
        return self.mag

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-r', '--right', action='store_true', dest='right', default=False,
                 help='Right finger')
    p.add_option('-l', '--left', action='store_true', dest='left', default=False,
                 help='left finger')
    opt, args = p.parse_args()
    
    if opt.right:
        r_tact = TactileSensor('/pressure/r_gripper_motor')
    if opt.left:
        l_tact = TactileSensor('/pressure/l_gripper_motor')

    rospy.spin()



#     print 'Reading: ', r_tact.read()

#     raw_input('Hit [ENTER]')
    
#     r_tact.bias()
#     time.sleep(3.0)
#     print 'Reading: ', r_tact.read()
    
#     raw_input('Hit [ENTER]')

#     print 'COMBINED:', np.append( *r_tact.read() )
#     print 'ERR:', np.sum(np.abs( np.append( *r_tact.read() )))

#     print 'Waiting for thresh'
#     r_tact.thresh_detect( 5000 )
