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


## @class Bug3D node that consumes a model from Haptic Map Node and a goal from
#  the TeleOp Node and produces waypoints that explore the environment to
#  achieve a goal configuration.
class Bug3D():
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

    self.mode = self.NAIVE
    current_pos = self.getCurrentPos()
    current_goal = self.getGoalPos()

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
        self.mode = self.NAIVE
        current_goal = self.getGoalPos()

      self.run3DBug()
      rate.sleep()

  ## Mode Constants
  NAIVE = "NAIVE"
  BUG = "BUG"
  
  ## Publishes a given pose
  ## Publishes a given pose
  def publishWaypoint(self, desired_pos):
    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header = self.getMessageHeader()
    waypoint_msg.pose.position = desired_pos
    waypoint_msg.pose.orientation.w = 1.0

    self.pose_waypoint_pub.publish(waypoint_msg)  ## Get the current position


  def getGoalPos(self):
    with self.goal_lock:
      return copy.copy(self.goal_pose)

  ## Get the current position
  def getCurrentPos(self):
    with self.state_lock:
      return copy.copy(self.gripper_pose)

  ## Get an updated copy of the collision map
  def getCollisionMap(self):
    message = sensor_msgs.msg.PointCloud()
    return Set(message.points)

  ## Returns the colliding point, or None of there is no colliding point
  def checkCollision(self):
    # Poll variables
    collision_map = self.getCollisionMap()
    curr_pos = self.getCurrentPos()
   
    # Get point of the grid that are adjacent to the curr_pos 
    grid_size = .01
    x = haptic_mpc_util.divRound(curr_pos.x, grid_size)
    y = haptic_mpc_util.divRound(curr_pos.y, grid_size)
    z = haptic_mpc_util.divRound(curr_pos.z, grid_size)
    adjacents = Set()
    for dx in [-1, 0, 1]:
      for dy in [-1, 0, 1]:
        for dz in [-1, 0, 1]:
          adjacents.add((x + dx, y + dy, z + dz))

    # Check of any of those points is in the collision_map
    for point in adjacents:
      if point in collision_map:
        return point

    return None


  ## Runs a 3D bug algorithm to explore an environment
  def run3DBug(self):
    rate = rospy.Rate(self.rate)
   
    rospy.loginfo("Current mode: " + self.mode)
 
    current_goal = self.getGoalPos()
    
    # Handling naive pursuit
    if self.mode == self.NAIVE:
      # Publish a naive goal
      self.publishWaypoint(current_goal)
      
      coll_pos = self.checkCollision()

      if coll_pos != None:
        rospy.loginfo("Detected a collision at" + str(coll_pos))

        # Extract variable information
        coll_x = coll_pos.x
        coll_y = coll_pos.y
        coll_z = coll_pos.z
   
        goal_x = current_goal.x
        goal_y = current_goal.y
        goal_z = current_goal.z

        # Compute vector from collision point to goal
        self.initial_vec = [goal_x - coll_x, goal_y - coll_y, goal_z - coll_z]
        self.initial_vec = initial_vec/(numpy.sqrt(numpy.sum(numpy.square(self.initial_vec))))

        # Compute orthogonal plane from collision point to goal
        vectors = []
        vectors.append( numpy.cross(initial_vec,[0,0,1]) )
        vectors.append( numpy.cross(initial_vec,vectors[0]) )
        vectors[0] = vector[0]/(numpy.sqrt(numpy.sum(numpy.square(vector[0]))))
        vectors[1] = vector[1]/(numpy.sqrt(numpy.sum(numpy.square(vector[1]))))
        vectors.append( -vector[0] )
        vectors.append( -vector[1] )
        
        # Initialize BUG variable
        self.vectors_index = 0
        self.segment = 1
        self.segment_remaining = 1

        # Update the mode and return
        self.mode = self.BUG
        return

    # Handling Bug pursuit
    if self.mode == self.BUG:
 
      # Turn if necessary
      spiral_size = .04
      if self.segment_remaining == 0:
        self.vectors_index = (self.vectors_index + 1) % 4
        if self.vectors_index % 2 == 0:
          self.segment += 1
        self.segment_remaining = self.segment

      # Compute and publish the next waypoint
      waypoint = curr_pos + self.vectors[self.vectors_index] * spiral_size + self.initial_vec * spiral_size
      self.segment_remaining -= 1
      self.publishWaypoint(waypoint)

      # Determine if we are no longer colliding with anything
      coll_pos = self.checkCollision()
      if coll_pos == None:
        self.mode = self.NAIVE

      return

    while True:
      rospy.loginfo("ERROR: UNRECOGNIZED MODE.")
if __name__ == '__main__':
    # Set up input arg parser
    import optparse
    p = optparse.OptionParser()

    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    # Create and start the trajectory manager module. 
    traj_mgr = Bug3D('mpc_traj_gen', opt) # loads all parameter sets on init
    traj_mgr.start()
    




