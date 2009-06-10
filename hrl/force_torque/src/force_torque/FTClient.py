import roslib; roslib.update_path('force_torque')
import rospy
import hrl_lib.rutils as ru
import hrl_lib.util as ut
import numpy as np

##
# Corresponding client class
class FTClient(ru.FloatArrayListener):
    def __init__(self, topic_name):
        ru.FloatArrayListener.__init__(self, 'FTClient', topic_name, 100.0)
        self.bias_val = np.matrix([0,0,0, 0,0,0.0]).T

    def read(self, avg=1, without_bias=False, fresh=False):
        assert(avg > 0)
        rs = []
        for i in range(avg):
            r = ru.FloatArrayListener.read(self, fresh) 
            if r == None:
                return None
            rs.append(r)
        readings = ut.list_mat_to_mat(rs, axis=1)
        if not without_bias:
            return readings.mean(1) - self.bias_val
        else:
            return readings.mean(1)

    def bias(self, fresh=True):
        r = ru.FloatArrayListener.read(self, fresh)
        if r != None:
            self.bias_val = r

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('-t', action='store', default='force_torque_ft1', type='string', 
                 dest='topic', help='which topic to listen to')
    opt, args = p.parse_args()

    client = FTClient(opt.topic)
    while not rospy.is_shutdown():
        el = client.read()
        if el != None:
            print el.T












