import nodes as n
import numpy as np
import math
import time
import util as ut
import itertools as it
import logging as lg
from StringIO import StringIO
import connected_comp as cc

def v_repulse(euclid_pts, goal=None, zone_of_influence=.6, zone_of_safety=.2):
	"""
		Calculates a repulsive vector
	"""
	polar_pts = ut.pol_of_cart(euclid_pts)
	#print "v_repulse: In polar", polar_pts.T

	if goal != None:
		polar_goal = ut.pol_of_cart(goal)

	def determine_mag(pt):
		if pt[0,0] < zone_of_safety:
			#print "DANGEr", pt[0,0]
			return 1.0
		elif pt[0,0] < zone_of_influence:
			return (zone_of_influence - pt[0,0]) / (zone_of_influence - zone_of_safety)
		else:
			return 0.0

	total = 0
	repulsive_vec = np.matrix([0.0, 0.0]).T
	for i in range(polar_pts.shape[1]):
		pt = polar_pts[:,i]
		repulse = pt.copy()
		if goal != None:
			turn_dir = ut.best_turn_dir(polar_goal[1,0], pt[1,0])
			#If it's in front
			if np.abs(turn_dir) < np.pi:
				#if it's to the right
				if turn_dir > 0:
					#print "RIGHT"
					repulse[1,0] = pt[1,0] + (np.pi/2.0)
				else:
					#if it's to the left
					#print "LEFT"
					repulse[1,0] = pt[1,0] - (np.pi/2.0)
		repulse[0,0]  = determine_mag(pt)
		repulsive_vec = repulsive_vec + ut.cart_of_pol(repulse)
		total = 1 + total

	if total == 0:
		return np.matrix([0.0, 0.0]).T
	else:
		return repulsive_vec / float(total)


def v_anti_obstacle(laser_scan, goal=None, zone_of_influence=.6, ignore_goal=True, 
        zone_of_goal = .4, zone_of_safety=.3, close_mean_alpha=.75):
    """
        Avoid obstacles in 2D
        goal              - euclidean point in ego centric frame
        ignore_goal       - ignore connected components in the vicinity of the goal
        close_mean_alpha  - blending between repulsion of obstacle's centroid 
                             and obstacle's closest point, higher value favors
                             the closest point
        zone_of_influence - points that are not within this distance from robot
                             are also not considered
        zone_of_goal      - points within this distance from the goal are not 
                             considered (should be same as zone of influcence if don't
                             want goal laser points to exert repulsion)
        TODO: add transforming laser points into base frame so 
              that the 'center' is perceived correctly
    """
    components = list(cc.connected_comps(laser_scan)) 
    influence = []
    #print "################ Components", len(components)

    def within_range(comp):
        ranges, indices = comp
        return np.min(ranges) < zone_of_influence

    def to_euclid(comp):
        ranges, indices = comp
        return (cc.euclid_of_laser(scans=laser_scan, indices=np.matrix(indices)), indices)

    def mean(comp):
        euclid_pt, indices = comp
        return np.sum(euclid_pt, axis=1) / float(euclid_pt.shape[1])

    def closest_pt(comp):
        ranges, indices = comp
        closest_idx     = indices[np.argmin(ranges)]
        return  cc.euclid_of_laser(scans=laser_scan, indices=np.matrix([closest_idx])) 

    def repulse(pt):
        return v_repulse(pt, goal=goal, zone_of_influence=zone_of_influence, zone_of_safety=zone_of_safety)

    def alpha_combine(pair):
        closest_r, mean_r = pair
        return (close_mean_alpha * closest_r) + ((1.0- close_mean_alpha) * mean_r)

    def add_vec(a,b):
        return a+b

    def not_in_goal_range(pt):
        if goal != None:
            return np.linalg.norm(goal - pt) > zone_of_goal
        else:
            return True

    in_range         = filter(within_range, components)
    euclid_pts       = map(to_euclid, in_range)
    mean_pts         = map(mean, euclid_pts)
                                    #Only calculate repulsion for points
                                    #not in vincinity of goal
    mean_repulses    = map(repulse, filter(not_in_goal_range, mean_pts))

    closest_pts      = map(closest_pt, in_range)
    closest_repulses = map(repulse, filter(not_in_goal_range, closest_pts))

    influence        = map(alpha_combine, it.izip(closest_repulses, mean_repulses))

    #print "# obstacles", len(influence)
    if len(influence) > 0:
        ret = reduce(add_vec, influence) / float(len(influence))
        return ret
    else:
        return np.matrix([0.0, 0.0]).T


class V_Obstacle_Avoid(n.Node):
    def __init__(self, laser, goal=None, zone_of_goal = .4, 
            zone_of_influence=.6, zone_of_safety=.2):
        self.laser        = laser
        self.goal         = goal
        self.zone_of_goal = zone_of_goal
        self.zone_of_safety=zone_of_safety
        self.zone_of_influence=zone_of_influence

    def calc(self, t):
        if self.goal != None:
            return v_anti_obstacle(
                    self.laser.val(t), self.goal.val(t), 
                    zone_of_goal = self.zone_of_goal,
                    zone_of_influence=self.zone_of_influence,
                    zone_of_safety=self.zone_of_safety)
        else:
            return v_anti_obstacle(
                    self.laser.val(t), zone_of_goal = self.zone_of_goal,
                    zone_of_safety=self.zone_of_safety,
                    zone_of_influence=self.zone_of_influence)

class V_UpdatedLocal_V(n.Node):
	"""
		Keeps whenever time_valid is True, takes the value of 
		updates and keeps it in the local reference frame.
		Uses:  Viewing the latest laser pointer estimate in the local frame
			   Viewing the latest face detection in the local frame
	"""
	def __init__(self, robot_pose, node, updates, time_valid):
		self.robot_pose = robot_pose
		self.node       = node
		self.updates    = updates
		self.time_valid = time_valid
		self.localized  = V_KeepLocal_P2d_V(robot_pose, n.ConstNode(n.Pose2D(0,0,0).pos))
		self.reseted    = True

	def calc(self, t):
		#This line is important, don't delete it! -Hai
		self.node.val(t)
		if self.time_valid.val(t) or self.reseted:
			self.reseted = False
			new_pos = self.updates.val(t)
			self.localized.remember(n.ConstNode(new_pos[0:2,:]))
			print "V_UpdatedLocal_V: Updated estimate to", new_pos.T
		ret = self.localized.val(t)
		print "V_UpdatedLocal_V: point is at ", ret.T
		return ret

	def reset(self):
		print "V_UpdatedLocal_V: RESETTED TO 0, 0"
		self.localized = V_KeepLocal_P2d_V(self.robot_pose, n.ConstNode(n.Pose2D(0,0,0).pos))
		self.reseted   = True


class B_TooClose_V(n.Node):
	"""
	   Indicates whether points given in a set of points is too close.
	"""
	def __init__(s, dists_node, safe_distance, origin=np.matrix([0,0]).T):
		s.dists_node = dists_node
		s.safe_distance = safe_distance
		s.origin = origin
		s.log = ut.log("TooClose")


	def calc(s,t):
		points = s.dists_node.val(t)
		diffs = (points.T - s.origin.A[:,0]).T
		distances = np.power(np.sum(np.power(diffs, 2), 0),0.5)

		closest_idx  = distances.argmin()
		s.closest_pt = points[:, closest_idx]
		s.log.log(lg.INFO, "closest distance ", distances[0,closest_idx])
		ret = (distances < s.safe_distance).any()
		if ret:
			s.log.log(lg.DEBUG, "too close (safe distance ", s.safe_distance, "m)")
		return ret

	def close_pt(s):
		def close_pt_(t):
			s.val(t)
			return s.closest_pt
		return n.FuncNode(close_pt_)


class V_LocalToGlobal_P2d_V(n.Node):
	"""
		Converts from the local frame to the global frame
	"""
	def __init__(self, global_pos_node, local_node):
		self.global_pos = global_pos_node
		self.local_pose = local_node

	def calc(self, t):
		g = self.global_pos.val(t)
		l = self.local_pose.val(t)
		g_T_l = np.linalg.inv(n.transform2D(g.pos, g.angle))
		return ut.homo_to_point(g_T_l * ut.point_to_homo(l))


class P_WeightedAdd_PA(n.Node):
	"""
		Calculates the weighted addition of a list of nodes.
		Renormalize weights to all sum up to 1.
	"""
	def __init__(self, nodes = None):
		if (None == nodes):
			self.nodes = []
		else:
			self.nodes = nodes

	def add(self, node, weight):
		self.nodes.append((node,weight))

	def calc(self,t):
		mag  = 0.0
		ang  = 0.0

		total_weights = 0.0
		for n, w in self.nodes:
			polar = n.val(t)

			#Don't count this node as outputing anything if it does not output a command
			if not (ut.approx_equal(polar[0,0], 0.0) and ut.approx_equal(polar[1,0], 0.0)): 
				mag   += w * polar[0,0]
				ang   += w * polar[1,0]
				total_weights += w
		#print "total_weights", total_weights
		return np.matrix([mag, ang]).T / total_weights


class B_Close_VV(n.Node):
	"""
		Tests to see if two points are close to each other
	"""
	def __init__(self, close_dist, node_a, node_b):
		self.node_a = node_a
		self.node_b = node_b
		self.close_dist = close_dist

	def calc(self, t):
		dist = np.linalg.norm(self.node_a.val(t) - self.node_b.val(t))
		return dist < self.close_dist


class V_LinearAttract_V(n.Node):

	def __init__(self, target, dead_zone=0.05, far_dist=1.0, min_attr = 0.1):
		""" 
			If target is within dead_zone then stop, or if the target's
			distance is greater than far_dist then go at normal speed,
			otherwise speed is a linear function of distance

			min_attr mininum attractive force
		"""
		self.dead_zone = dead_zone
		self.far_dist = far_dist
		self.target = target
		self.last_dist = far_dist
		self.min_attr = min_attr
		self.verbosity = 0

	def calc(self, t):
		goal = self.target.val(t)                        
		dist = np.linalg.norm(goal)
		self.last_dist = dist
		mag = 0.0        

		processed = None
		out = None
		norm_dist = dist - self.dead_zone
		if (dist >= self.dead_zone):
			mag = ((1 - self.min_attr) / (self.far_dist - self.dead_zone))*norm_dist + self.min_attr
			mag = min(max(mag, self.min_attr), 1.0)
			out = mag * (goal / dist)
		else:
			out = np.matrix([0., 0.]).T      

		if (self.verbosity > 0):
			print "V_LinearAttract_V using: ", out.T, " to go to: ", goal.T, " dist: ", dist
		return out

	def is_done(self):
		def f(t):
			#print "V_LinearAttract_V", self.target.val(t)
			dist = np.linalg.norm(self.target.val(t))
			if (self.verbosity > 1):
				print "V_LinearAttract_V: dist", dist, "dead_zone", self.dead_zone, 
				print "done? ", dist < self.dead_zone

			if dist < self.dead_zone:
				if self.verbosity > -1:
					print "V_LinearAttract_V: arrived! dist:", dist, " dead_zone:", self.dead_zone

			return dist < self.dead_zone
		return n.FuncNode(f,())


class P_FlipReverse_P(n.Node):
	def __init__(self, polar_node):
		"""
			 If the polar coordinates representation points behind the robot
			 then flip and reverse sign so that R_ToBaseCommand_P can drive
			 backwards.
		"""
		self.polar_node = polar_node

	def calc(self, t):
		dir = self.polar_node.val(t)

		speed = dir[0,0]
		turn  = dir[1,0]

		#If it's behind us, err flip it and reverse it
		if (turn < (-np.pi/2.0)) or (turn > (np.pi/2.0)):
			dir[0,0] = speed * -1
			dir[1,0] = ut.standard_rad(turn + np.pi)

		return dir


class P_TurnTo_V(n.Node):
    """
        Turns to the direction given by vector (does not keep track of given vector
                in local pose, so must use a quantity that is being updated - this
                is since the same node can be used to turn to face a continually
                updated stimuli).
        """
    def __init__(self, direction, time_tol = .1, tolerance=math.radians(5), 
                 initial_angle = math.radians(5), max_vel=math.radians(45), verbosity = 0):
        self.verbosity = verbosity
        self.direction = direction
        self.tolerance = tolerance
        self.time_tol = time_tol
        #self.at_loc = (False, time.time())
        self.reset()
        self.set_slope(initial_angle, max_vel)

    def set_slope(self, angle, max_vel):
        self.slope = angle / max_vel

    def calc(self,t):
        dir_raw = ut.pol_of_cart(self.direction.val(t))
        dir = dir_raw * self.slope
        if self.verbosity > 0:
            print "P_TurnTo_V: at", math.degrees(dir_raw[1,0]), " deg"

        if self.tolerance > abs(dir_raw[1,0]):
            if not self.at_loc[0]:
                self.at_loc = (True, time.time())
        else:
            self.at_loc = (False, time.time())

        if abs(dir[1,0]) < math.radians(3):
            if dir[1,0] < 0:
                dir[1,0] = math.radians(-3)
            else:
                dir[1,0] = math.radians(3)
        ret = np.matrix([0, dir[1,0]]).T
        if self.verbosity > 0:
            print "P_TurnTo_V: returned", ret.T
        return ret

    def reset(self):
        self.at_loc = (False, time.time())

    def is_done(self):
        def turn_to_b(t):
            if self.at_loc[0]:
                diff = time.time() - self.at_loc[1]
                if diff > self.time_tol:
                    if self.verbosity > 0:
                        print "P_TurnTo_V: within threshold for ", diff, "s"
                    self.reset()
                return diff > self.time_tol
            else:
                return False

        return n.FuncNode(turn_to_b, ())

class V_KeepLocal_P2d_V(n.Node):
	"""
		Given a point or group of points in the local frame,
		keeps track of it in the local frame as the robot pose
		is updated by odometry.
	"""
	robot_pose_node = None
	o_T_a = None  # Transforms from initial pose to global (frame o)
	_point = None # aG

	def __init__(self, robot_pose_node_p2d, point, immediate = None, verbosity=0, name=""):
		""" 
			point egocentric column vector(s)

			immediate: whether to record the robot pose immediately
		"""
		self.robot_pose_node = robot_pose_node_p2d
		self.local_point     = point
		self.global_point    = None
		self.verbosity       = verbosity
		self.name = name

		if immediate != None:
			print "V_KeepLocal_P2d_V: point", name, " immediate flag deprecated!!!!"
			#self.remember(self.local_point)


	def remember(self, local_point):
		"""
			Set egocentric points to keep track of
		"""
		self.local_point  = local_point
		point             = local_point.val(-1)
		start_pos         = self.robot_pose_node.val(-1)
		a_T_o             = n.transform2D(start_pos.pos, start_pos.angle)
		self.global_point = np.linalg.inv(a_T_o) * ut.point_to_homo(point)

		if (self.verbosity > 0):
		   print "V_KeepLocal_P2d_V: (", self.name, ") point ", point.T
		   print "V_KeepLocal_P2d_V: (", self.name, ") start_pos ", start_pos
		   print "V_KeepLocal_P2d_V: (", self.name, ") global_point ", self.global_point.T


	def reset(self):
		self.global_point = None

	def force_update(self):
		self.val(-1)

	def calc(self, t):
		""" 
			Return point(s) in the robot's current coordinate frame (frame b)
			i.e. bG

			Given:
				aG
				a_T_o
				b_T_o

			Calc: bG            
			With: bG (b_T_o) (o_T_a) aG
		"""
		if (self.global_point == None):
			self.remember(self.local_point)
		current_pose = self.robot_pose_node.val(t)
		#print 'V_KeepLocal_P2d_V', current_pose
		b_T_o        = n.transform2D(current_pose.pos, current_pose.angle)   
		results = ut.homo_to_point(b_T_o * self.global_point)     
		#print 'V_KeepLocal_P2d_V', results.T
		return results

class R_SelectBaseCommand_(n.Node):
	"""
		Takes as input a node that sometime return base commands
		return only base commands and stops when there aren't any
		base commands.
	"""
	def __init__(self, commands):
		self.commands = commands

	def calc(self, t):
		r = self.commands.val(t)
		if (r.__class__ == n.BaseCommand):
			return r
		else:
			return n.BaseCommand(0,0)

def R_Conv_V(node, allow_backwards_driving=True):
	return R_ToBaseCommand_P(P_Conv_V(node), allow_backwards_driving=allow_backwards_driving)

class R_ToBaseCommand_P(n.Node): 
    """ 
        Scale numbers to real world units, takes in polar coordinates
        return robot commands also in polar form (as it isn't possible to
    express pure turning with Cartesian vectors).

        reverse_lr - reverse left and right
    """
    def __init__(self, desired_direction, max_turn_speed=np.pi/8, max_vel=.10, reverse_lr = False, allow_backwards_driving=True, verbosity = 0):
        """ 
            Convert a unit vector to a command to the robot base in rotational and
            forward translational velocity.  Assume that "desired_direction" gives
            vectors in egocentric coordinates.
        """            
        if self.verbosity > 0:
            print "R_ToBaseCommand_P: max vel", max_vel, " max turn ", max_turn_speed
        if allow_backwards_driving:
            self.desired_direction = P_FlipReverse_P(desired_direction)
        else:
            self.desired_direction = desired_direction
        self.verbosity = verbosity

        if self.verbosity > 0:
            print "R_ToBaseCommand_P: desired dir", desired_direction.__class__

        if (max_vel < 0):
            if self.verbosity > -1:
                print "R_ToBaseCommand_P: WARNING max velocity less than 0"
        if (max_turn_speed < 0):
            if self.verbosity > -1:
                print "R_ToBaseCommand_P: WARNING max turn speed less than 0"

        self.max_turn_speed = abs(max_turn_speed)
        self.max_vel        = abs(max_vel)
        self.reverse_lr     = reverse_lr


    def calc(self, t):        
        pol_desired      = self.desired_direction.val(t)
        if self.verbosity > 0:
            print "R_ToBaseCommand_P: desired", pol_desired.T

        turn_amount      = pol_desired[1,0]
        dspeed           = pol_desired[0,0]

        if (abs(turn_amount) > self.max_turn_speed):
            #Don't go forward if robot is trying its best to turn
            if (self.verbosity > 0):
                print "R_ToBaseCommand_P: Turning only"
            dspeed = 0
            if (turn_amount < 0):
                turn_amount = -self.max_turn_speed
            else:
                turn_amount = self.max_turn_speed

        if (dspeed > 1):            
            dspeed = 1
        vel =  dspeed * self.max_vel
        if self.reverse_lr:
            turn_amount = turn_amount * -1.0

        if self.verbosity > 0:
            print "R_ToBaseCommand_P: vel", vel, "turn", turn_amount
        return n.BaseCommand(vel, turn_amount)

class P_Conv_V(n.Node):
	"""
		Converts from Cartesian to polar coordinates
	"""
	def __init__(self, enode):
		self.enode = enode

	def calc(self, t):
		vec = self.enode.val(t)
		return np.matrix([np.linalg.norm(vec), math.atan2(vec[1,0], vec[0,0])]).T

class V_Conv_P2d(n.Node):    
	"""
		Converts from a Pose2D to a vector with only position in tact
	"""
	def __init__(self, p2d):
		self.p2d = p2d

	def calc(self,t):
		return self.p2d.val(t).pos

