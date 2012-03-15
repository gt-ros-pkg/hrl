#! /usr/bin/python
## {{{ http://code.activestate.com/recipes/572182/ (r2)
import sys, termios, atexit
from select import select
import roslib; roslib.load_manifest('hrl_lib')
import rospy

class KeyboardInput():
    def __init__(self):

        # save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)

        # new terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        atexit.register(self.set_normal_term)
        self.set_curses_term()

    # switch to normal terminal
    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    # switch to unbuffered terminal
    def set_curses_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

    def putch(self, ch):
        sys.stdout.write(ch)

    def getch(self):
        return sys.stdin.read(1)

    def getche(self):
        ch = self.getch()
        self.putch(ch)
        return ch

    def kbhit(self):
        dr,dw,de = select([sys.stdin], [], [], 0)
        return dr <> []

    def pauser(self):
        if self.kbhit():
            ch = self.getch()
            if ch == 'p':
                rospy.loginfo("PAUSED")
                while not rospy.is_shutdown() and ch != 'c':
                    ch = self.getch()
                    rospy.sleep(0.01)
                rospy.loginfo("CONTINUING")

    def pause(self):
        rospy.loginfo("PAUSED")
        ch = 'd'
        while not rospy.is_shutdown() and ch != 'c':
            ch = self.getch()
            rospy.sleep(0.01)
        rospy.loginfo("CONTINUING")

if __name__ == '__main__':
    ki = KeyboardInput()

    while True:
        if ki.kbhit():
            ch = ki.getch()
            if ch == 'p':
                while ch != 'c':
                    ch = ki.getch()
            if ch == 'q':
                break
        sys.stdout.write('.')

    print 'done'
## end of http://code.activestate.com/recipes/572182/ }}}

