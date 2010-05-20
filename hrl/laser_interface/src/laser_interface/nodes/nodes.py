import time
import threading
import numpy
import sys
import math
import fun
from transforms2d import *

#import util as ut
from functools import partial
from StringIO import StringIO

import opencv.cv as cv
import opencv.highgui as hg
import connected_comp as cc

##
## Nodes to support navigation, convention is that nodes return vectors
## of type numpy.array.  Code is in the style of object oriented functional
## programming.
##

def str_of_tuple(a):
    p = StringIO()
    print >>p, "(", a[0], ",",
    for t in a[1:-1]:
        print >>p, t, ",", 
    print >>p, a[-1], ")",
    return p.getvalue()


class UnimplementedError(Exception):
    def __str__(self):
        return repr("UnimplementedError")

class Node(object):
    """
         Basic functional class.  Notice that
         because of __iter__, all nodes
         are iterator compatible.
    """
    last_time = -1
    last_val  = None
    verbosity = 0

    def val(self, t):
        if (t > self.last_time) or (t == -1):
           # Setting time before the call to calc prevents 
           # infinite loops in systems of nodes with circular
           # dependencies
            if (t > 0):
                self.last_time = t
            self.last_val = self.calc(t)
        return self.last_val

    def calc(self, t):
        """ Implement this in sub classes """
        raise UnimplementedError()        

    def __iter__(self):
        return as_iter(self)

    def is_done(self):
        if (self.verbosity > 0):
            print "Node: using default is_done() - running FOREVER"
        return ConstNode(False)

    def go(self, snoz, stop_on = None, threads = None, frequency = 80, run_motor = True):
        if stop_on == None:
            stop_on = self.is_done()
        loop  = CMDLoop(self, stop_on, snoz, frequency_run=frequency, run_motor = run_motor)
        if threads != None:
            for t in threads:
                loop.add_thread(t)
        loop.go()
        loop.stop()


class as_iter(object):
    def __init__(self, node):
        self.time = 0
        self.node = node

    def next(self):
        self.time += 1
        return self.node.val(self.time)


class as_node(Node):
    def __init__(self, iter):
        self.iter = iter

    def calc(self, t):
        return iter.next()


class Select(Node):
    def __init__(self, selector, selection):
        self.selector = selector
        self.embedded = selection
    
    def calc(self, t):
        selection = self.selector.val(t)
        return self.embedded[selection].val(t)
    
    
class FSA(Node):    
    """ 
        Finite State Machine: user has to specify triggers and end states 
    """
    def __init__(self, triggers = None):
        """
            trigger[current_state] - all triggers for the current state
        """
        if (triggers != None):
            self.triggers = triggers
        else:
            self.triggers = dict()
        self.state          = None
        self.on_change_func = None
        self.absorbing      = set()
        self.first = True

    def set_on_change(self, func):
        self.on_change_func = func

    def add_absorbing(self, state):
        self.absorbing.add(state)
            
    def add_state(self, name, triggers):
        """ 
            Add a state to this state machine
            Triggers is a list of (BooleanNode, state name) pairs.
            At each time step the triggers are evaluated in order
            for the current state, the first true trigger causes
            a transition.
        """
        self.triggers[name] = triggers
        if (self.state == None):
            self.state = name

    def calc(self, t):
        if self.first == True:
           print "FSA: Starting in state ", self.state
           self.first = False
        for (trigger, follow_state) in self.triggers[self.state]:
            if (trigger.val(t) == True):
                if (self.on_change_func != None):
                   self.on_change_func(self.state, follow_state)
                if self.verbosity > 0:
                    print "FSA: transition from '", self.state, 
                    print "' to: '", follow_state, "'"
                self.state = follow_state
                break
        return self.state

    def is_absorbing_B(self):
        def absorbing(t):
            return self.state in self.absorbing
        return FuncNode(absorbing, ())


def SequencialFSA(steps):
    """
        Construct FSA that execute sequentially takes in states of 
        form name, action, checks.
    """
    transitions = {}
    actions     = {}
    for i, n in enumerate(steps):
        name, action, checks  = n
        actions[name]        = action
        if (i+1) != len(steps):
            next_name, _         = steps[i+1]
            transitions[name] = [(checks, next_name)]
        else:
            transitions[name] = [(checks, name)]

    absorbing, c = steps[-1]
    start, _ = steps[0]

    fsa = FSA(transitions, absorbing)
    fsa.state = start
    fsa.add_absorbing(absorbing)

    return Select(fsa, actions)


def ExecFSA(start_state, states, absorbing = ""):
    """
        Simple fsa takes in states of form name, action, checks.
        Combines selector and fsa in one.
    """
    transitions = {}
    actions = {}
    for (name, action, checks) in states:
        transitions[name] = checks
        actions[name]     = action
    
    fsa = FSA(transitions)
    fsa.add_absorbing(absorbing)
    fsa.state = start_state
    return Select(fsa, actions)

            
class ConstNode(Node):
    """ Gives a boolean """
    def __init__(self, const):
        self.const = const        
    
    def calc(self,t):
        return self.const
 

class FuncNode(Node):
   """ 
      Executes a given function.  Useful for 
      executing functions at a later time.

      fun: float -> args -> some_value
   """
   def __init__(self, fun, args=()):
      self.fun = fun
      self.args = args

   def calc(self, t):
      timed = partial(self.fun, t)
      a = self.args
      return timed(*a)


class Filter(Node):
    """
       Used to execute some function on the result of preceeding node.
       A node similar to FuncNode but it also provides the value of
       a an embedded node.
    """
    def __init__(self, node, f):
        self.node = node
        self.f = f

    def calc(self, t):
        return self.f(self.node.val(t))


class FreezeVal(Node):
    """
        Freeze the value of a child node so that
        this node will always return the same value
        as the child node when the child node is 
        first executed.  Freeze as late as possible
        in computation chain to save resources.
    """
    def __init__(self, node, is_valid=None):
        self.node = node
        self.frozen_val = None
        
        if is_valid == None:
            def not_none(v):
                return v != None
            self.is_valid = not_none
        else:
            self.is_valid = is_valid

    def calc(self,t):
        if (self.frozen_val == None):
            v = self.node.val(t)
            if self.is_valid(v):
                self.frozen_val = v
        return self.frozen_val


class WhileExec(Node):
   """ Hackish operator to perform some computation while executing an action"""
   def __init__(self, action, compute):
      self.action = action
      self.compute = compute

   def calc(self,t):
      self.compute.val(t)
      return self.action.val(t)


class B_Any_BA(Node):
	def __init__(self, nodes, verbosity=0):
		self.nodes = nodes
		self.verbosity = verbosity

	def calc(self, t):
		def call_val(n):
			v = n.val(t)
			if v and self.verbosity > 0:
				print "B_Any_BA:", n.__class__, "was true"
			return v
		b = map(call_val, self.nodes)
		return any(b)


def Not(node_b):
    def b_not_b(v):
        return not v
    return Filter(node_b, b_not_b)


class Timer(Node):
    """
        Timer node, returns False until time is up.  Resets
        itself after it returns true.
    """
    def __init__(self, secs):
        self.start = None
        self.secs = secs
    
    def calc(self,t):
        if (self.start == None):
            self.start = time.time()
        if (time.time() - self.start) > self.secs:
            self.start = None
            return True
        else:
            return False
      

class TupleCastNode(Node):
    def __init__(self, type, node):
        self.type = type
        self.node = node
    
    def calc(self, t):
        return (type, node.val(t))

class P2d_Player(Node):    
    """
        Gives pose of robot according to odometry. User must call 
        player.read before querying for new values. 
    """
    def __init__(self, snoz):
        self.snoz = snoz
        player_client   = snoz.player_client

        # Fix for poses not updating initially
        for i in xrange(33):
           time.sleep(1.0/66.0)
           player_client.read()

    def set_pose(self, p2d):
        self.snoz.erratic.erratic.set_cmd_pose(p2d.pos[0,0], p2d.pos[1,0], p2d.angle, 0)
        
    def calc(self, t):
        p2d = self.snoz.erratic.erratic
        cur_p2d = Pose2D(p2d.px, p2d.py, p2d.pa)

        if (self.verbosity > 0):
            print "P2d_Player: ", cur_p2d
        return cur_p2d

class V_EuclidOfLaser_V(Node):
    def __init__(self, laser, urg_number=0):
        """ Gets laser points in base's reference frame """
        #self.transform = cc.euclid_of_laser()
        self.laser = laser

    def calc(self,t):
		r                = self.laser.val(t)
		euclidified      = cc.euclid_of_laser2(r)
		urg0_natural     = cc.urg0Tnatural(euclidified)
		urg0_global      = tr.globalTurg0(urg0_natural)
		return urg0_global[:2, :]



class LACommand(object):
   """
      A linear actuator command
   """
   #POSITION  = "position"
   TORQUE    = "torque"
   STOP      = "stop"
   UP        = "up"
   DOWN      = "down"

   def __init__(self, cmd_type, arg=None):
      self.cmd_type = cmd_type
      self.arg      = arg

   def __eq__(self, other):
      if (other == None):
	return False
      return (self.cmd_type == other.cmd_type) and (self.arg == other.arg)

   def do(self, zenither):
      if (self.cmd_type == LACommand.STOP):
         zenither.estop()
      if (self.cmd_type == LACommand.TORQUE):
         if (self.arg.__class__ != "string".__class__):
            zenither.zenith(torque=self.arg)
         elif (self.arg == LACommand.UP):
            zenither.zenith()
         else:
            zenither.nadir()


class BaseCommand(object):
   """
      A base command
   """

   def __init__(self, forward_vel, rot_vel):
      self.forward_vel = forward_vel
      self.rot_vel     = rot_vel

   def do(self, p2d):
      p2d.set_cmd_vel(self.forward_vel, 0.0, self.rot_vel, 1)


class CMDLoop(threading.Thread):
    """
         Used as main loop for network of nodes.  Takes as input a node to run whose
         commands will be sent to the robot actuators and another node for stopping condition.  

         As this class will serve as the 'main loop' it also include convenience facilities 
         for starting/stopping children threads automatically when nodes are added in add_thread.

         If frequency == None then run at max rate
    """
    def __init__(self, run_node, finished_node, snozz, frequency_run = 80, 
                 frequency_player=100, run_motor=True):
        threading.Thread.__init__(self)
        self.run_node  = run_node
        self.finished  = finished_node

        self.frequency_player = frequency_player
        self.frequency_run = frequency_run
        self.snozz     = snozz

        self.verbosity              = 0
        self.managed_threads        = []
        self.should_run             = True
        self.run_motor              = run_motor
        self.last_player_read       = time.time()
        self.last_player_pose       = (0.0,0.0,0.0)
        self.start()
        #Allow time for thread to start
        #time.sleep(.2)


    def add_thread(self, thread):
        self.managed_threads.append(thread)
        #print "starting thread"
        thread.start()


    def __del__(self):
        self.stop()


    def __restart(self):
        self.should_run = True
        self.start()
        for t in self.managed_threads:
            t.start()


    def run(self):
       """
          NOTE: Don't call this directly, this is only supposed to
          be called by constructor through thread.start().
       """
       if self.frequency_player == None:
           stime = 0
       else:
           stime = 1.0 / self.frequency_player

       try:
           while self.should_run:
               self.snozz.read_queue(run_motor=True)
               #Record time when we got the last batch of good player data
               self.last_player_read = time.time()
               #print "Read!"
               #time.sleep(stime)
       except:
           self.stop(should_join = False)
           raise


    def stop(self, should_join = True):
        """
            Call this to clean up threads started for player and
            other processes
        """
        self.should_run = False
        if should_join:
            self.join(3)
            if (self.isAlive()):
                raise RuntimeError("CMDLoop: unable to stop command thread.")
        for t in self.managed_threads:
            t.stop()
            if should_join:
                t.join(3)
                if (t.isAlive()):
                    p = StringIO()
                    print >>p, "CMDLoop: unable to stop thread: ", t.__class__
                    raise RuntimeError(p.getvalue())


    def go(self, max_stale = 20.0):
        """ Start this loop

            max_stale - maximum cycles to tolerate stale player data, if this
                        is exceeded, the loop will stop to wait for new player
                        data.
        """
        if (self.should_run == False):
            self.__restart()
        try:
            t = time.time()
            if self.run_motor:
                self.snozz.erratic.erratic.enable(1)
            else:
                self.snozz.erratic.erratic.enable(0)

            if self.frequency_run == None:
                stime = 0
            else:
                stime = 1.0 / self.frequency_run

            if (self.verbosity > 0):
                print "CMDLoop: Executing at ", self.frequency_run, " hz"
                print "         until ", self.finished.__class__, " says to stop"
                print "        ", len(self.managed_threads)+1, " child threads"

            while (not self.finished.val(t)):
                if (time.time() - self.last_player_read) < (stime * max_stale):
                    cmd = self.run_node.val(t)
                    if cmd.__class__ == LACommand:
                        cmd.do(self.snozz.zenither)
                    elif cmd.__class__ == BaseCommand:
                        #if self.snozz.had_to_reset:
                        #    print "had to reset, cmd: ", cmd.forward_vel, cmd.rot_vel
                        #else:
                        #print "cmd: ", cmd.forward_vel, cmd.rot_vel
                        cmd.do(self.snozz.erratic.erratic)
                    time.sleep(stime)
                    t = time.time()
                else:
                    if self.verbosity > -1:
                        print "CMDLoop: paused (no data)"
                    while (time.time() - self.last_player_read) > (stime * max_stale):
                        time.sleep(.2)
                    if self.verbosity > -1:
                        print "CMDLoop: resuming..."

            #if self.run_motor:
            #    self.snozz.erratic.erratic.enable(0)
            if (self.verbosity > 0):
                print "CMDLoop: finished."
        except:
            if (self.verbosity > 0):
                print "CMDLoop: exception stopped."
            self.stop()
            raise


def as_int(n):
    return (int) (round(n))

class RobotDisp(object):
    def __init__(self, name, size=2, draw_center=True, draw_grid=True, meters_radius=4.0):
		"""
			 name = name of window
			 meter_radus = 4.0
			 size = multiple of 400x200 to use for screen
			 meter_radius = how many per metrer 
		"""
		self.draw_center = draw_center
		self.draw_grid   = draw_grid
		self.w = (int) (round(size * 400.0))
		self.h = (int) (round(size * 200.0))


		self.meters_disp = 4.0  #Range in meters of area around robot to display
		self.laser_win = name
		self.buffer = cv.cvCreateImage(cv.cvSize(self.w, 2*self.h), cv.IPL_DEPTH_8U, 3)
		#print "RobotDisp: window width", self.buffer.width
		#print "RobotDisp: window height", self.buffer.height
		self.pixels_per_meter = self.h / self.meters_disp
		hg.cvNamedWindow(name, hg.CV_WINDOW_AUTOSIZE)
		hg.cvMoveWindow(name, 0, 50)

		self.font = cv.cvInitFont(cv.CV_FONT_HERSHEY_PLAIN, 
				as_int(1), as_int(1), 0, 1, cv.CV_AA)
		#as_int(1), as_int(1), 0, 1)


    def to_screen(self, vc):
        v        = vc.copy()
        v_cv     = screenTglobal(self.w, self.h*2, self.pixels_per_meter, v)
        return v_cv

    def clear(self):
        cv.cvRectangle(self.buffer, 
                cv.cvPoint(0,0), 
                cv.cvPoint(self.buffer.width, self.buffer.height), 
                cv.cvScalar(255,255,255),
                cv.CV_FILLED)

        if self.draw_grid:
            line_color = 230
            lc = cv.cvScalar(line_color,line_color,line_color)
            for i in xrange(1, as_int(self.meters_disp)+3):
                cv.cvCircle(self.buffer, cv.cvPoint(self.w/2,self.h), as_int(self.pixels_per_meter * (i-.5)), 
                        #lc, 1)
                        lc, 1, cv.CV_AA)
                cv.cvCircle(self.buffer, cv.cvPoint(self.w/2,self.h), as_int(self.pixels_per_meter * i), 
                        #lc, 1)
                        lc, 1, cv.CV_AA)

            for i in xrange(360/30):
                x = (self.w/2) + math.cos(math.radians(i*30)) * self.pixels_per_meter * (self.meters_disp+2)
                y = self.h     + math.sin(math.radians(i*30)) * self.pixels_per_meter * (self.meters_disp+2)
                cv.cvLine(self.buffer, cv.cvPoint(self.w/2,self.h), cv.cvPoint(as_int(x),as_int(y)), lc, 1, cv.CV_AA)

        if self.draw_center:
            cv.cvCircle(self.buffer, cv.cvPoint(self.w/2,self.h), 3, 
                    cv.cvScalar(0,0,200), cv.CV_FILLED, cv.CV_AA)
                    #cv.cvScalar(0,0,200), cv.CV_FILLED)

    def point(self, pts, size=1, color=cv.cvScalar(100,100,100)):
		def circ(n):
			pt  = cv.cvPoint(int(round(n[0,0])),int(round(n[1,0])))
			cv.cvCircle(self.buffer, pt, size,
					color, cv.CV_FILLED, cv.CV_AA)
		map(circ, fun.points_of_mat(pts))

    def draw(self, wait=10):
        hg.cvShowImage(self.laser_win, self.buffer)
        hg.cvWaitKey(wait)


class RobotDisp_VA(Node):
	def __init__(self, nodes, name, size=2):
		self.disp = RobotDisp(name, size)
		self.nodes = nodes
        
	def calc(self,t):
		def process_node(tup): 
			node, color, size, text = tup 
			color_scalar = cv.cvScalar(color[0], color[1], color[2]) 
			node_val     = node.val(t) 

			if node_val.__class__ != tuple: 
				if node_val != None: 
					v_cv = self.disp.to_screen(node.val(t)) 

				def circ(n): 
					pt  = cv.cvPoint(int(round(n[0,0])),int(round(n[1,0]))) 
					cv.cvCircle(self.disp.buffer, pt, size, 
							color_scalar, cv.CV_FILLED, cv.CV_AA) 
					pt2 = cv.cvPoint(pt.x + 2, pt.y + 2) 
					cv.cvPutText(self.disp.buffer, text, pt,  self.disp.font, cv.cvScalar(255,255,255)) 
					cv.cvPutText(self.disp.buffer, text, pt2, self.disp.font, cv.cvScalar(50,50,50)) 
				map(circ, fun.points_of_mat(v_cv))
			else: 
				start_pts, end_pts = node_val 
				for idx in range(start_pts.shape[1]): 
					start_pt = cvpoint_of_pt(self.disp.to_screen(start_pts[:,idx]))[0] 
					end_pt   = cvpoint_of_pt(self.disp.to_screen(  end_pts[:,idx]))[0] 
					cv.cvLine(self.disp.buffer, start_pt, end_pt, 
							color_scalar, size, cv.CV_AA)
         
		self.disp.clear()
		map(process_node, self.nodes)
		self.disp.draw()







