#! /usr/bin/python

import sys
import numpy as np
import scipy.io
import scipy.stats
import matplotlib.pyplot as plt

import roslib
roslib.load_manifest('hrl_phri_2011')
import rospy

import rosbag

def main():
    bag = rosbag.Bag(sys.argv[1], 'r')
    x_forces, cur_x_forces = [], []
    for topic, msg, t in bag.read_messages(topics=["/l_cart/sensor_ft"]):
        f = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        f_mag = np.linalg.norm(f)
        if f_mag > 0.5:
            cur_x_forces.append(msg.wrench.force.x)
        else:
            if len(cur_x_forces) >= 20:
                x_forces.extend(cur_x_forces)
            cur_x_forces = []
    if len(cur_x_forces) >= 20:
        x_forces.extend(cur_x_forces)
    bag.close()
    ptile_inds = [25, 50, 75, 95]
    ptiles = {}
    for ptile in ptile_inds:
        ptiles[ptile] = scipy.stats.scoreatpercentile(x_forces, ptile)
        print "Percentile %d: %f" % (ptile, ptiles[ptile])
#plt.plot(x_forces)
#plt.show()

if __name__ == "__main__":
    main()
