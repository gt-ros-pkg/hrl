#!/usr/bin/python
from threading import RLock

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


if __name__ == '__main__':
    import roslib; roslib.load_manifest('force_torque')
    import rospy
    from force_torque.srv import *
    from hrl_lib.msg import FloatArray as FloatArray
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
    print node_name + ': publishing on channel', ft_channel_name
    #times = []
    biasft = None
    while not rospy.is_shutdown():
    #while len(times) < 201:
        msg = ftserver.get_msg()
        if msg is not None:
            data, tme = msg
            ftvalue = ftc.binary_to_ft(data)
            if biasft == None:
                biasft = np.array(ftvalue)
                print biasft
            #print biasft.__class__, biasft
            ftvalue = (np.array(ftvalue) - biasft).tolist()
            channel.publish(FloatArray(rospy.Header(stamp=rospy.Time.from_seconds(tme)), ftvalue))
            #times.append(time.time())
        #else:
        #    time.sleep(1/5000.0)
        time.sleep(1/5000.0)

    #import pylab as pl
    #import numpy as np
    #a = np.array(times)
    #pl.plot(a[1:] - a[:-1], '.')
    #pl.show()







