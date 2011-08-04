#! /usr/bin/python

import numpy as np, math
import sys
import pygame as pg

import roslib
roslib.load_manifest('equilibrium_point_control')
roslib.load_manifest('hrl_lib')

import rospy

from equilibrium_point_control.ep_control import EPGenerator, EPC, EPStopConditions
from hrl_lib.timer import Timer

class TestEPGenerator(EPGenerator):
    def __init__(self, num_steps=1000):
        self.ind = 0
        self.pts = np.dstack([np.linspace(0, 0.5, num_steps),
                              np.linspace(-1, 2, num_steps),
                              np.linspace(1, -5, num_steps)])[0]

    def generate_ep(self):
        ep = self.pts[self.ind]
        stop = EPStopConditions.CONTINUE
        self.ind += 1
        return stop, ep

    def control_ep(self, ep):
        print "control_ep:", ep

    def clamp_ep(self, ep):
        return np.clip(ep, 0, 1)

    def terminate_check(self):
        if self.ind == len(self.pts):
            return EPStopConditions.SUCCESSFUL
        else:
            return EPStopConditions.CONTINUE

def main():
    rospy.init_node('test_epc')
    pg.init()
    screen = pg.display.set_mode((300,300))

    test_gen = TestEPGenerator(100)
    test_epc = EPC('test_epc')

    # wait for keys p for pause and s for stop
    test_epc.last_time = rospy.get_time()
    def keyboard_cb(timer_event):
        pg.event.get()
        keys = pg.key.get_pressed()
        if keys[pg.K_s] or keys[pg.K_q]:
            test_epc.stop_epc = True
        if keys[pg.K_p] and rospy.get_time() - test_epc.last_time > 0.3:
            test_epc.pause_epc = not test_epc.pause_epc
            test_epc.last_time = rospy.get_time()
    Timer(rospy.Duration(0.1), keyboard_cb)

    test_epc.epc_motion(test_gen, 0.1, 10)

if __name__ == '__main__':
    main()
