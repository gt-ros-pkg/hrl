
#
# Display ratio between ROS time and system-time.
# using curses to avoid scrolling but still keep things terminal
# based.
#
## author: Advait Jain


import sys, time
import curses
import numpy as np, math
import threading

import roslib; roslib.load_manifest('hrl_lib')
import rospy

class ROSTimeRate(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rt = -1
        self.start()

    def run(self):
        prev_sim_time = rospy.get_time()
        prev_sys_time = time.time()
        while not rospy.is_shutdown():
            time.sleep(0.1)
            sim_time = rospy.get_time()
            sys_time = time.time()
            self.rt = (sim_time-prev_sim_time) / (sys_time-prev_sys_time)
            prev_sim_time = sim_time
            prev_sys_time = sys_time

    def get_rate(self):
        return self.rt


if __name__ == '__main__':
    rospy.init_node('ros_time_rate_printer')
    
    rtr = ROSTimeRate()

    stdscr = curses.initscr()
    curses.start_color()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(1)

    row, col = 1, 2

    while not rospy.is_shutdown():
        rt = rtr.get_rate()
        stdscr.clear()
        msg = 'ROS time is %.2f x real-time'%rt
        stdscr.addstr(row, col, msg)
        stdscr.refresh()
        time.sleep(0.1)

    #------ gracefully exit curses? --------
    curses.nocbreak();
    stdscr.keypad(0);
    curses.echo()
    curses.endwin()





