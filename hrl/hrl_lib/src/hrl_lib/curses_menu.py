
# Simple menu using curses. Inherit this class and add your own
# functionality.
## author: Advait Jain

import sys
import curses

class CursesMenu():
    def __init__(self):
        self.stdscr = curses.initscr()
        curses.start_color()
        curses.noecho()
        curses.cbreak()
        self.stdscr.keypad(1)
        self.menu_dict = {}
        self.key_list = [] # to esnure that menu is displayed in the same order as it is created.

    def begin_menu(self, title):
        self.menu_title = title

    def add_item(self, key, text, function=None):
        self.menu_dict[key] = (text, function)
        self.key_list.append(key)

    def finish_menu(self, text):
        self.menu_finish = text

    def display_menu(self):
        self.row, self.col = 0, 0
        self.stdscr.addstr(self.row, self.col, self.menu_title)

        for k in self.key_list:
            self.row += 1
            self.col = 2
            self.stdscr.addstr(self.row, self.col, str(k)+': '+self.menu_dict[k][0])

        self.row += 2
        self.col = 0
        self.stdscr.addstr(self.row, self.col, self.menu_finish)
        self.col = len(self.menu_finish) + 2
        self.stdscr.move(self.row, self.col)

    def run(self):
        self.display_menu()
        while 1:
            c = chr(self.stdscr.getch())
            self.stdscr.clear()
            self.display_menu()
            self.stdscr.addstr(self.row, self.col, c)

            notification_row = self.row+1
            notification_col = 5
            self.stdscr.move(notification_row, notification_col)
            if c not in self.menu_dict:
                self.stdscr.addstr('Incorrect option')
            else:
                t,f = self.menu_dict[c]
                if f == None:
                    self.stdscr.addstr('Unimplemented option')
                else:
                    # call the function.
                    self.stdscr.addstr('Going to '+ t)
                    f(c)
            self.stdscr.move(self.row, self.col)

    def exit(self, key=''):
        curses.nocbreak();
        self.stdscr.keypad(0);
        curses.echo()
        curses.endwin()
        sys.exit()

    def ros_service_call(self, option):
        # make a ros service call to perform appropriate function.
        pass


if __name__ == '__main__':

    cm = CursesMenu()
    cm.begin_menu('Here is what I can do for you today:')
    cm.add_item('f', 'Fly')
    cm.add_item('g', 'Make a ROS service call', cm.ros_service_call)
    cm.add_item('q', 'Quit', cm.exit)
    cm.finish_menu('Select you option')
    cm.run()







