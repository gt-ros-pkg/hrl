#!/usr/bin/python
import roslib
roslib.load_manifest('force_torque')
import rospy
from geometry_msgs.msg import Vector3Stamped


from threading import RLock



## 1D kalman filter update.
def kalman_update(xhat, P, Q, R, z):
    #time update
    xhatminus = xhat
    Pminus = P + Q
    #measurement update
    K = Pminus / (Pminus + R)
    xhat = xhatminus + K * (z-xhatminus)
    P = (1-K) * Pminus
    return xhat, P



class FTRelay:
    def __init__(self):
        self.lock = RLock()
        self.fresh = False

    def set_ft(self, value, time_acquired):
        self.lock.acquire()
        self.data = value, time_acquired
        self.fresh = True
        self.lock.release()
        #print 'got', value, time_acquired

    def get_msg(self):
        r = None
        self.lock.acquire()
        if self.fresh:
            self.fresh = False
            r = self.data
        self.lock.release()
        return r

def FTread_to_Force( ftval, frame_id ):
    retval = Vector3Stamped()
    retval.header.stamp = rospy.rostime.get_rostime()
    retval.header.frame_id = frame_id
    retval.vector.x = ftval[0]
    retval.vector.y = ftval[1]
    retval.vector.z = ftval[2]

    return retval

if __name__ == '__main__':
    import roslib; roslib.load_manifest('force_torque')
    import rospy
    from force_torque.srv import *
    from hrl_msgs.msg import FloatArray as FloatArray
    import hrl_lib.rutils as ru
    import time
    import force_torque.FTSensor as ftc
    import numpy as np

    import optparse
    p = optparse.OptionParser()
    p.add_option('--name', action='store', default='ft1', type='string', 
                 dest='name', help='name given to FTSensor')
    opt, args = p.parse_args()

    node_name       = 'FTRelay_'     + opt.name
    ft_channel_name = 'force_torque_' + opt.name
    service_name    = node_name + '_set_ft'
    print node_name + ': serving service', service_name
    ftserver = FTRelay()

    rospy.init_node(node_name)
    rospy.Service(service_name, StringService, 
            ru.wrap(ftserver.set_ft, ['value', 'time'], 
                    response=StringServiceResponse))

    channel = rospy.Publisher(ft_channel_name, FloatArray, tcp_nodelay=True)
    channel2 = rospy.Publisher(ft_channel_name + '_raw', FloatArray, tcp_nodelay=True)
    chan_vec3 = rospy.Publisher(ft_channel_name + '_Vec3', Vector3Stamped, tcp_nodelay=True)    
    print node_name + ': publishing on channel', ft_channel_name
    P_force = [1., 1., 1.]
    xhat_force = [0., 0., 0., 0., 0., 0.]
    while not rospy.is_shutdown():
        msg = ftserver.get_msg()
        if msg is not None:
            data, tme = msg
            ftvalue = ftc.binary_to_ft(data)
            ftvalue = np.array(ftvalue)
            for i in range(3):
                xhat, p = kalman_update(xhat_force[i], P_force[i],
                        1e-3, 0.04, ftvalue[i])
                P_force[i] = p
                xhat_force[i] = xhat
                #ftvalue[i] = xhat
            xhat_force[3] = ftvalue[3]
            xhat_force[4] = ftvalue[4]
            xhat_force[5] = ftvalue[5]
            ftvalue = ftvalue.tolist()

            channel.publish(FloatArray(rospy.Header(stamp=rospy.Time.from_seconds(tme)),
                xhat_force))
            channel2.publish(FloatArray(rospy.Header(stamp=rospy.Time.from_seconds(tme)), ftvalue))
            chan_vec3.publish( FTread_to_Force( ftvalue, opt.name ))
            #times.append(time.time())
        #else:
        #    time.sleep(1/5000.0)
        time.sleep(1/5000.0)

    #import pylab as pl
    #import numpy as np
    #a = np.array(times)
    #pl.plot(a[1:] - a[:-1], '.')
    #pl.show()







