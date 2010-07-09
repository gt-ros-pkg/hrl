#!/usr/bin/python

import roslib; roslib.load_manifest('modeling_forces')
import rospy
import force_torque.FTClient as ftc
import hrl_lib.util as ut

from std_msgs.msg import Empty

import time
import numpy as np
import glob


##
# make an appropriate plot.
# @param d - dictionary saved in the pkl
def plot_ft(d):
    ft_list = d['ft_list']
    time_list = d['time_list']
    ft_mat = np.matrix(ft_list).T # 6xN np matrix
    force_mat = ft_mat[0:3, :]

    tarray = np.array(time_list)
    print 'average rate', 1. / np.mean(tarray[1:] - tarray[:-1])
    time_list = (tarray - tarray[0]).tolist()
    print len(time_list)
    
    force_mag_l = ut.norm(force_mat).A1.tolist()
    #for i,f in enumerate(force_mag_l):
    #    if f > 15:
    #        break
    #force_mag_l = force_mag_l[i:]
    #time_list = time_list[i:]

    mpu.plot_yx(force_mag_l, time_list, axis=None, label='Force Magnitude',
                xlabel='Time(seconds)', ylabel='Force Magnitude (N)',
                color = mpu.random_color())

def got_trigger_cb(data, d):
    d['flag'] = True


if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p.add_option('-l', '--log', action='store_true', dest='log', help='log FT data')
    p.add_option('-p', '--plot', action='store_true', dest='plot', help='plot FT data')
    p.add_option('-r', '--ros', action='store_true', dest='ros',
                 help='start and stop logging messages over ROS.')
    p.add_option('-a', '--all', action='store_true', dest='all',
                 help='plot all the pkls in a single plot')
    p.add_option('-f', '--fname', action='store', default='',
                 type='string', dest='fname', help='pkl with logged data')
    
    opt, args = p.parse_args()

    if opt.log:
        client = ftc.FTClient('/force_torque_ft2')
        l = []
        time_list = []

        if opt.ros:
            topic_name_cb = '/ftlogger/trigger'
            got_trigger_dict = {'flag': False}
            rospy.Subscriber(topic_name_cb, Empty, got_trigger_cb,
                             got_trigger_dict)

            while got_trigger_dict['flag'] == False:
                time.sleep(0.05)

            got_trigger_dict['flag'] = False
            while not rospy.is_shutdown():
                ft, t_msg = client.read(fresh=True, with_time_stamp=True)
                if ft != None:
                    l.append(ft.A1.tolist())
                    time_list.append(t_msg)
                if got_trigger_dict['flag'] == True:
                    break
                time.sleep(1/1000.0)
        else:
            print 'Sleeping for 5 seconds.'
            time.sleep(5.)
            print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
            print 'BEGIN'
            print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
            client.bias()

            t_start = time.time()
            t_now = time.time()
            logging_time = 5.

            while (t_now - t_start) < logging_time:
                ft, t_msg = client.read(fresh=True, with_time_stamp=True)
                t_now = time.time()
                if ft != None:
                    l.append(ft.A1.tolist())
                    time_list.append(t_msg)
                time.sleep(1/1000.0)

        print 'saving the pickle'
        d = {}
        d['ft_list'] = l
        d['time_list'] = time_list

        fname = 'ft_log_'+ut.formatted_time()+'.pkl'
        ut.save_pickle(d, fname)


    if opt.plot:
        import matplotlib_util.util as mpu

        if opt.fname == '' and (not opt.all):
            raise RuntimeError('need a pkl name (-f or --fname) or --all')

        if opt.all:
            fname_list = glob.glob('ft_log*.pkl')
        else:
            fname_list = [opt.fname]

        for fname in fname_list:
            d = ut.load_pickle(fname)
            plot_ft(d)

        mpu.legend()
        mpu.show()


