#!/usr/bin/env python

## @package hrl_haptic_mpc
# 
# @author Trironk Kiatkungwanglai 
# @version 0.1
# @copyright Simplified BSD Licence

import sys, os
import numpy, math
import copy, threading
import collections

import roslib; roslib.load_manifest('hrl_tactile_controller')
import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import trajectory_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs

import haptic_mpc_util

# Robot kinematic classes and skin clients. These are specific to each robot
# and used to perform forward kinematics.
import pr2_arm_kinematics_darpa_m3 as pr2_arm
import pr2_skin_client as pr2_sc

import gen_sim_arms as sim_robot
import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
import sim_skin_client as sim_sc

from sets import Set
from collections import deque
from Queue import PriorityQueue

## @class Bug3D node that consumes a model from Haptic Map Node and a goal from
#  the TeleOp Node and produces waypoints that explore the environment to
#  achieve a goal configuration.
class IseD():
  node_name = None # ROS node name
  opt = None  # Options - should be specified by the optparser when run from the console.
  rate = 25.0 # Publish rate. By default, 25Hz
  msg_seq = 0 # Sequence counter
  
  # ROS Parameter paths - robot dependent.
  param_path = ""
  base_path = "/haptic_mpc"
  cody_param_path = "/cody"
  pr2_param_path = "/pr2"
  sim_param_path = "/sim3"
  simcody_param_path = "/simcody"
  
  # ROS Topics. TODO: Move these to param server to allow easy changes across all modules.
  current_pose_topic = "/haptic_mpc/robot_state"
  goal_pose_topic = "/haptic_mpc/goal_pose"
  goal_pose_array_topic = "/haptic_mpc/goal_pose_array"
  #waypoint_pose_topic = "/haptic_mpc/traj_pose"
  waypoint_pose_topic = "/haptic_mpc/exploration_pose"
  collision_map_topic = "/haptic_mpc/collision_map"
  
  traj_lock = threading.RLock()
  goal_lock = threading.RLock()
  state_lock = threading.RLock()
  map_lock = threading.RLock()
  
  
  ## Constructor. Calls functions to initialise robot specific parameters, then initialises all publishers/subscribers.  
  def __init__(self, node_name, opt):
    rospy.loginfo("Initialising trajectory generator for Haptic MPC")
    self.opt = opt
    self.node_name = node_name
    rospy.init_node(node_name)

    self.gripper_pose = None
    self.goal_pose = None    
    self.collision_map = None

    # Set up the relevant robot parameters
    self.initPR2()
    
    # Set up the publishers/subscribers and their callbacks.
    self.initComms()

    return

  ## Initialise PR2 kinematics. NB: Only used for joint limits, will eventually be removed once these are passed with the robot state.
  def initPR2(self):
    rospy.loginfo("Trajectory generator for: PR2")
    if not self.opt.arm:
      rospy.logerr('Arm not specified for PR2')
      sys.exit()
    self.robot_kinematics = pr2_arm.PR2ArmKinematics(self.opt.arm)
    self.tf_listener = tf.TransformListener()

    if self.opt.arm == None:
        rospy.logerr('Need to specify --arm_to_use.\nExiting...')
        sys.exit()

   ## Initialise all publishers/subscribers used by the waypoint generator.
  def initComms(self):        
    # Publish to a waypoint pose topic
    self.pose_waypoint_pub = rospy.Publisher(self.waypoint_pose_topic, geometry_msgs.msg.PoseStamped) # Pose waypoint publishing      
    
    # Subscribe to the a goal pose topic. 
    rospy.Subscriber(self.goal_pose_topic, geometry_msgs.msg.PoseStamped, self.goalPoseCallback)
    # Subscribe to the current robot state 
    rospy.Subscriber(self.current_pose_topic, haptic_msgs.RobotHapticState, self.robotStateCallback)
    # Subscribe to the a collision map topic. 
    rospy.Subscriber(self.collision_map_topic, sensor_msgs.msg.PointCloud, self.collisionMapCallback)
 
 
  ## Update goal pose.
  # @param msg A geometry_msgs.msg.PoseStamped object.
  def goalPoseCallback(self, msg):
    rospy.loginfo("Got new goal pose")
    if not 'torso_lift_link' in msg.header.frame_id:
      try:
        self.tf_listener.waitForTransform(msg.header.frame_id, '/torso_lift_link', msg.header.stamp, rospy.Duration(5.0))
      except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
        rospy.logerr('[arm_trajectory_generator]: TF Exception: %s' %e)
      ps = geometry_msgs.msg.PoseStamped(msg.header, msg.pose)
      ps.header.stamp = rospy.Time(0)
      new_pose_stamped = self.tf_listener.transformPose('/torso_lift_link', ps)
      msg = new_pose_stamped

    with self.goal_lock:
      self.goal_pose = msg.pose
  
  def collisionMapCallback(self, msg):
    with self.map_lock:
      self.collision_map = msg

  ## Store the current pose from the haptic state publisher
  # @param msg RobotHapticState messge object
  def robotStateCallback(self, msg):
    with self.state_lock:
      self.gripper_pose = msg.hand_pose
      self.joint_angles = msg.joint_angles
       
  ## Returns a message header (std_msgs object), populated with the time, frame_id and sequence number
  def getMessageHeader(self):
    header = std_msgs.msg.Header()
    header.seq = self.msg_seq
    self.msg_seq += 1
    header.stamp = rospy.get_rostime()
    header.frame_id = "/torso_lift_link"
    
    return header
    
  ## Euclidian distance between two poses. Ignores differences in orientation.
  # @param poseA geometry_msgs.msg.Pose
  # @param poseB geometry_msgs.msg.Pose
  # @return The euclidian distance between the two pose positions.
  def distanceBetweenPoses(self, poseA, poseB):
    xdiff = poseA.position.x - poseB.position.x
    ydiff = poseA.position.y - poseB.position.y
    zdiff = poseA.position.z - poseB.position.z
    return numpy.sqrt(xdiff**2 + ydiff **2 + zdiff**2)      

  ## Return the angle between two quaternions (axis-angle representation)
  # @param poseA geometry_msgs.msg.Pose
  # @param poseB geometry_msgs.msg.Pose
  # @return The angular distance between the two pose orientations.
  def angularDistanceBetweenPoses(self, poseA, poseB):
    quatA = [poseA.orientation.x, poseA.orientation.y, poseA.orientation.z, poseA.orientation.w]
    quatB = [poseB.orientation.x, poseB.orientation.y, poseB.orientation.z, poseB.orientation.w]
    ang_diff = ut.quat_angle(quatA, quatB)
    return ang_diff
  
  ## Start the waypoint generator publishing waypoints.  
  def start(self):
    rate = rospy.Rate(self.rate) # 25Hz, nominally

    self.mode = self.SEARCH
    self.minima = {}
    self.path = deque()
    self.branches = []
    self.memory = []

    current_pos = self.getCurrentPos()
    current_goal = self.getGoalPos()
    self.branches.append((current_pos, []))

    while not rospy.is_shutdown():
      print rospy.Time()

      if current_pos == None:
        rospy.loginfo("Cannot detect a current position")
        rate.sleep()
        continue

      if current_goal == None:
        rospy.loginfo("Cannot detect a goal position")
        rate.sleep()
        continue
     
      # Reset if given a new goal
      if current_goal != self.getGoalPos():
        self.mode = self.SEARCH
        self.minima.clear()
        self.path.clear()
        self.branches.clear()
        self.memory.clear()
        current_goal = self.getGoalPos()
        self.branches.append(current_pos, [])

      self.runIse()
      rate.sleep()

  ## Mode Constants
  BACK_TRACK = "BACK_TRACK"
  FOLLOW = "FOLLOW"
  DONE = "DONE"
  
  ## Publishes a given pose
  def publishWaypoint(self, desired_pos):
    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header = self.getMessageHeader()
    waypoint_msg.pose.position = desired_pos
    waypoint_msg.pose.orientation.w = 1.0

    self.pose_waypoint_pub.publish(waypoint_msg)

  ## Get the current position
  def getGoalPos(self):
    with self.goal_lock:
      return copy.copy(self.goal_pose.pose.position)

  ## Get the current position
  def getCurrentPos(self):
    with self.state_lock:
      return copy.copy(self.gripper_pose.pose.position)

  ## Get an updated copy of the collision map
  def getCollisionMap(self):
    message = sensor_msgs.msg.PointCloud()
    return Set(message.points)

  ## Returns the colliding point, or None of there is no colliding point
  def checkCollision(self, collision_map=None, curr_pos=None):
    # Poll variables
    if collision_map == None:
      collision_map = self.getCollisionMap()
    if curr_pos == None:
      curr_pos = self.getCurrentPos()
   
    # Get point of the grid that are adjacent to the curr_pos 
    adjacents = getAdjacentPoints( self.discretizePoint(curr_pos) )

    # Check of any of those points is in the collision_map
    for point in adjacents:
      if point in collision_map:
        return point

    return None

  ## Returns all adjacent points to a given point
  def getAdjacentPoints(self, point):
    result = Set()
    for dx in [-self.grid_size, 0, self.grid_size]:
      for dy in [ -self.grid_size, 0, self.grid_size]: 
        for dz in [ -self.grid_size, 0, self.grid_size]:
          result.add((result.x + dx, result.y + dy, result.z + dz))
    result.remove(point)
    return result

  self.grid_size = .01
  
  ## Discretize a position to the nearest grid point
  def discretizePoint(self, pos):

    # Get point of the grid that are adjacent to the curr_pos 
    result = geometry_msgs.msg.Point()
    result.x = discretize(pos.x)
    result.y = discretize(pos.y)
    result.z = discretize(pos.z)
     
    return result 

  ## Discretize a value
  def discretize(self, val):
    return haptic_mpc_util.divRound(val, self.grid_size)

  ## Computes the distance between two points
  def computeDistance(self, pos1, pos2):
    vec = [ pos1.x - pos2.x, pos1.y - pos2.y, pos1.z - pos2.z ]
    return numpy.sqrt(numpy.sum(numpy.square(vec))

  ## Returns the first collision point between two given points, or None if there are none
  def checkLinearPath(self, point1, point2):
    distance = computeDistance(point1, point2)
    
    # Get the intermediate waypoints along a straight line
    x_path_continuous = numpy.interp(point1.x, point2.x, distance)
    y_path_continuous = numpy.interp(point1.y, point2.y, distance)
    z_path_continuous = numpy.interp(point1.z, point2.z, distance)
    
    # Discretize the intermediate waypoints
    x_path = []
    for x in x_path_continuous:
      x_path.append( self.discretize(x) )
    y_path = []
    for y in y_path_continuous:
      y_path.append( self.discretize(y) )
    x_path = []
    for z in z_path_continuous:
      z_path.append( self.discretize(z) )
    
    # Construct a set of waypoints to check
    path = []
    for i in range(distance): 
      path.append( geometry_msgs.msg.Point() )
      path[-1].x = x_path[i]
      path[-1].y = y_path[i]
      path[-1].z = z_path[i]

    # Check for collisions
    collision_map = self.getCollisionMap()
    for point in path:
      if point in collision_map:
        return point

    return None

  ## Check to see if we have made progress recently
  def distanceRecentlyTraveled(self, steps=10):

    distance_traveled = 0
    for i in range(2, steps):
      distance_traveled += self.computeDistance(self.memory[-i], self.memory[-i+1])

    return distance_traveled

  ## Runs a 3D bug algorithm to explore an environment
  def runIse(self):
    rate = rospy.Rate(self.rate)
   
    rospy.loginfo("Current mode: " + self.mode)
 
    current_goal = self.getGoalPos()
    current_pos = self.getCurrentPos()
    current_pos = discretizePoint(current_pos)
    self.memory.append(current_pos)

    # Check to see if we have a new closest point
    distance_to_goal = self.computeDistance(current_pos, current_goal)
    if min(self.minima.keys()) < distance_to_goal:
      rospy.loginfo("Found a new closest point at " + str(current_pos))
      self.minima[distance_to_goal] = current_pos

    # If we made it to a point in our path, remove it
    if self.path[0] == current_pos:
      path.popleft()

    # If there is a next point on our path, publish it
    self.publishWaypoint(path[0])

    # Handle following
    if self.mode == self.FOLLOW:

      ###########################
      # CHECK FOR NEW COLLISION #
      ###########################

      # Check if it's time to back track
      if self.distanceRecentlyTraveled(10):

        # Clear the current path
        path.clear()

        # Get the target to backtrack to
        target, self.full_backtrack_memory = self.branch[-1]

        # Set the path as a trimmed subset of the self.full_backtrack_memory
        path.append( self.full_backtrack_memory[0] )
        prev_point = path[0]
        for point in self.full_backtrack_memory:
          
          # Prevent redundant points inside of our path
          if point == path[-1]:
            continue

          # If there is a collision to this point, put the previous point on the path
          if self.checkLinearPath( path[-1], point != None:
            path.append( prev_point )
          
          # Track the previous point
          prev_point = point

        self.mode = BACK_TRACK

    # Handle backtracking
    elif self.mode == self.BACK_TRACK:

      # Check if we've arrived at the destination
      if len(self.path) == 0:

        # Remove all points from our backtrack memory
        self.full_backtrack_memory.clear()

        # Run an AStar search to the goal
        path = self.AStarSearch(current_pos, current_goal)

      # Check if it's time to replan
      if self.checkCollision() != None:
        pass
        ####################
        # Run AStar stuff. #
        ####################
      

  ## Run an AStar search from point1 to point2 and returns a path, or None if there isn't a path
  def AStarSearch(self, point1, point2):
    
    ## Initialize variables
    collision_map = getCollisionMap()
    priorityQueue = PriorityQueue()
    priorityQueue.put( (self.computeDistance(point1, point2), [ point1 ]) )

    # Continue while the priority queue is not empty
    while priorityQueue.empty() == False:
      
      # Get the next item
      distance, path = priorityQueue.get()

      # Check terminating condition
      if self.computeDistance(path[-1], point2) == 0:
        return path

      # Get the next possible points
      adjacents = self.getAdjacents( path[-1] )
      possibleAdjacents = Set()
      for point in adjacents:
        if point in collision_map:
          continue
        possibleAdjacents.put(point)

      # Push each of the new paths onto the priority queue
      for point in possibleAdjacents:

        # Copy the old path
        newPath = path[:]

        # Append the adjacent point
        newPath.append(point)

        # Put a new item into the priority queue
        priorityQueue.put( (self.computeDistance(point, point2) + len(new_path), path) )

    # If it gets here, it's impossible to reach point2
    return None

if __name__ == '__main__':
    # Set up input arg parser
    import optparse
    p = optparse.OptionParser()

    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    # Create and start the trajectory manager module. 
    traj_mgr = Bug3D('mpc_traj_gen', opt) # loads all parameter sets on init
    traj_mgr.start()
    




