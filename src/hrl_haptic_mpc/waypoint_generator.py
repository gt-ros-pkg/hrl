#!/usr/bin/env python

## @package hrl_haptic_mpc
#
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import sys, os
import numpy, math
import copy, threading
import collections

import roslib; roslib.load_manifest('hrl_haptic_mpc')
import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import trajectory_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs

import haptic_mpc_util

## @class WaypointGenerator Node which takes either a goal pose or a trajectory and passes
# local goals to the MPC controller to try to make the controller follow it.
class WaypointGenerator():
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
  waypoint_pose_topic = "/haptic_mpc/traj_pose"

  # Member variables
  robot = None # Robot object
  robot_name = None # Robot name
  sensor = None # Sensor object
  eq_gen_type = None
  default_eq_gen_type = 'mpc_qs_1'

  skin_topic_list = None
  scl = None # skin client
  epcon = None

  # Trajectory pose parameters.
  max_pos_step = 0.05 # 50mm steps at largest
  #max_pos_step = 0.0 # changed
  max_ang_step = 0.02 # 0.02 rad. Slightly over 1 degree
  at_waypoint_threshold = 0.02 # tolerance for being at the goal and moving to the next waypoint.
  at_waypoint_ang_threshold = numpy.radians(5.0)

  current_trajectory_deque = collections.deque()
  gripper_pose = None
  goal_pose = None
  current_gripper_waypoint = None

  traj_lock = threading.RLock()
  goal_lock = threading.RLock()
  state_lock = threading.RLock()


  ## Constructor. Calls functions to initialise robot specific parameters, then initialises all publishers/subscribers.
  def __init__(self, node_name, opt):
    rospy.loginfo("Initialising trajectory generator for Haptic MPC")
    self.opt = opt
    self.node_name = node_name
    rospy.init_node(node_name)

    # Set up the relevant robot parameters
    if (self.opt.robot == "cody"):
      self.initCody()
    elif (self.opt.robot == "pr2"):
      self.initPR2()
    elif(self.opt.robot == "sim3"):
      self.initSim3()
    elif(self.opt.robot == "simcody"):
      self.initSimCody()
    else:
      rospy.logerr("Invalid Robot type: %s" % robot)

    # Set up the publishers/subscribers and their callbacks.
    self.initComms()

    return

  ## Initialise 3DOF Sim kinematics. NB: This doesn't actually do anything and is added mostly for structural consistency.
  def initCody(self):
    rospy.loginfo("Trajectory generator for: Cody")
    #TODO:
    sys.exit()

  ## Initialise PR2 kinematics. NB: Only used for joint limits, will eventually be removed once these are passed with the robot state.
  def initPR2(self):
    # new kinematics
    from pykdl_utils.kdl_kinematics import create_kdl_kin
    # old kinematicsi
    import pr2_arm_kinematics_darpa_m3_deprecated as pr2_arm
    

    rospy.loginfo("Trajectory generator for: PR2")
    if not self.opt.arm:
      rospy.logerr('Arm not specified for PR2')
      sys.exit()
    # new kinematics
    #self.robot_kinematics = create_kdl_kin('torso_lift_link', self.opt.arm+'_gripper_tool_frame')
    # old kinematics - DEPRECATED.
    self.robot_kinematics = pr2_arm.PR2ArmKinematics(self.opt.arm)
    self.tf_listener = tf.TransformListener()

    if self.opt.arm == None:
        rospy.logerr('Need to specify --arm_to_use.\nExiting...')
        sys.exit()

  ## Initialise 3DOF Sim kinematics. NB: This doesn't actually do anything and is added mostly for structural consistency.
  def initSim3(self):
    rospy.loginfo("Trajectory generator for: Simulation 3DOF")
    # Nothing to initialise for this.
    
  ## Initialise all publishers/subscribers used by the waypoint generator.
  def initComms(self):        
    # Publish to a waypoint pose topic
    self.pose_waypoint_pub = rospy.Publisher(self.waypoint_pose_topic, geometry_msgs.msg.PoseStamped) # Pose waypoint publishing      
    
    # Subscribe to the a goal pose topic. 
    rospy.Subscriber(self.goal_pose_topic, geometry_msgs.msg.PoseStamped, self.goalPoseCallback)
    # Subscribe to the current robot state 
    rospy.Subscriber(self.current_pose_topic, haptic_msgs.RobotHapticState, self.robotStateCallback)
    # OpenRave planner for the PR2
    rospy.Subscriber('/haptic_mpc/openrave/joint_trajectory_plan', trajectory_msgs.msg.JointTrajectory, self.jointTrajectoryCallback)
    # Subscribe to the goal pose array topic.
    rospy.Subscriber(self.goal_pose_array_topic, geometry_msgs.msg.PoseArray, self.poseTrajectoryCallback)
    self.pose_traj_viz_pub = rospy.Publisher('/haptic_mpc/current_pose_traj', geometry_msgs.msg.PoseArray, latch=True) 

 
  ## Update goal pose.
  # @param msg A geometry_msgs.msg.PoseStamped object.
  def goalPoseCallback(self, msg):
    rospy.loginfo("Got new goal pose")
    if not 'torso_lift_link' in msg.header.frame_id:
      print "msg.header.frame_id"
      print msg.header.frame_id
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
    with self.traj_lock:
      self.current_trajectory_deque.clear()
  
  ## Store the current pose from the haptic state publisher
  # @param msg RobotHapticState messge object
  def robotStateCallback(self, msg):
    with self.state_lock:
      self.gripper_pose = msg.hand_pose
      self.joint_angles = msg.joint_angles
     
  ## Store a trajectory of poses in the deque. Converts it to the 'torso_frame' if required.
  # @param msg A geometry_msgs.msg.PoseArray object
  def poseTrajectoryCallback(self, msg):
    rospy.loginfo("Got new pose trajectory")
    self.goal_pose = None
    with self.traj_lock:
      self.current_trajectory_msg = msg
      self.current_trajectory_deque.clear()
      # if we have an empty array, clear the deque and do nothing else.
      if len(msg.poses) == 0:
        rospy.logwarn("Received empty pose array. Clearing trajectory buffer")
        return        

      #Check if pose array is in torso_lift_link.  If not, transform.
      if not 'torso_lift_link' in msg.header.frame_id:
        print "msg.header.frame_id"
        print msg.header.frame_id
        try:
          self.tf_listener.waitForTransform(msg.header.frame_id, '/torso_lift_link',
                                            msg.header.stamp, rospy.Duration(5.0))
        except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
          rospy.logerr('[arm_trajectory_generator]: TF Exception: %s' %e)
        
      pose_array = []
      for pose in msg.poses:
        ps = geometry_msgs.msg.PoseStamped(msg.header, pose)
        ps.header.stamp = rospy.Time(0)
        new_pose_stamped = self.tf_listener.transformPose('/torso_lift_link', ps)
        pose_array.append(new_pose_stamped.pose)

      # visualisation message
      pose_array_msg = geometry_msgs.msg.PoseArray()
      pose_array_msg.header.frame_id = "/torso_lift_link"
      pose_array_msg.poses = pose_array
      self.pose_traj_viz_pub.publish(pose_array_msg)
      #Append the list of poses to the recently cleared deque
      self.current_trajectory_deque.extend(pose_array)
      

  ## Store a joint angle trajectory in the deque. Performs forward kinematics to convert it to end effector poses in the torso frame.
  # @param msg A trajectory_msgs.msg.JointTrajectory object.
  def jointTrajectoryCallback(self, msg):    
    rospy.loginfo("Got new joint trajectory")
    with self.traj_lock:
      self.current_trajectory_msg = msg
      self.current_trajectory_deque.clear()
      
      for point in msg.points:
        # Calculate pose for this point using FK
        # Append pose to deque
        joint_angles = point.positions
        end_effector_position, end_effector_orient_cart = self.robot_kinematics.FK(joint_angles, len(joint_angles))
        end_effector_orient_quat = tr.matrix_to_quaternion(end_effector_orient_cart)
        
        pose = geometry_msgs.msg.Pose()
        ee_pos = end_effector_position.A1
        pose.position.x = ee_pos[0]
        pose.position.y = ee_pos[1]
        pose.position.z = ee_pos[2]
        pose.orientation.x = end_effector_orient_quat[0]
        pose.orientation.y = end_effector_orient_quat[1]
        pose.orientation.z = end_effector_orient_quat[2]
        pose.orientation.w = end_effector_orient_quat[3]
        
        self.current_trajectory_deque.append(pose)
        
   
  ## Returns the next waypoint along a straight line trajectory from the current gripper pose to the goal pose.
  # The step size towards the goal is configurable through the parameters passed in.
  # @param current_pose geometry_msgs.msg.Pose
  # @param goal_pose geometry_msgs.msg.Pose
  # @param max_pos_step = scalar float for position step size to take (metres)
  # @param max_ang_step = scalar float for orientation step size to take (radians)
  # @return A geometry_msgs.msg.Pose to send to the MPC 
  def straightLineTrajectory(self, current_pose, goal_pose, max_pos_step, max_ang_step):
    desired_pose = geometry_msgs.msg.Pose() # Create the Pose for the desired waypoint
    
    current_pos_vector = numpy.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    goal_pos_vector = numpy.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
    
    position_waypoint = self.getPositionStep(current_pos_vector, goal_pos_vector, max_pos_step)
  
    desired_pose.position.x = position_waypoint[0]
    desired_pose.position.y = position_waypoint[1]
    desired_pose.position.z = position_waypoint[2]
    
    # Calculate the new orientation. Use slerp - spherical interpolation on quaternions
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    goal_orientation = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]
    
    orientation_waypoint = goal_orientation#self.getOrientationStep(current_orientation, goal_orientation, max_ang_step)

    desired_pose.orientation.x = orientation_waypoint[0]
    desired_pose.orientation.y = orientation_waypoint[1]
    desired_pose.orientation.z = orientation_waypoint[2]
    desired_pose.orientation.w = orientation_waypoint[3]
    
    # Return completed pose data structure
    return desired_pose
  
  ## Returns a linearly interpolated step towards the goal pos from the current pos
  # @param current_pos Current position as a numpy array (assumed to be [x,y,z])
  # @param goal_pos Goal position as a numpy array (assumed to be [x,y,z])
  # @param max_pos_step A scalar max step size for position (in metres).
  # @return An interpolated step in position towards the goal.
  def getPositionStep(self, current_pos, goal_pos, max_pos_step):
    difference_to_goal = goal_pos - current_pos
    dist_to_goal = numpy.sqrt(numpy.vdot(difference_to_goal, difference_to_goal))
    
    if dist_to_goal > max_pos_step:
      step_vector = difference_to_goal / dist_to_goal * max_pos_step # Generate a linear step towards the goal position.
    else:
      step_vector = difference_to_goal # The distance remaining to the goal is less than the step size used.
    desired_position = current_pos + step_vector
    
    return desired_position
  
  ## Returns a linearly interpolated step towards the goal orientation from the current orientation (using SLERP)
  # @param q_h_orient Current hand orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
  # @param q_g_orient Current goal orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
  # @param max_ang_step A scalar max step size for orientation (in radians).  
  # @return An interpolated step in orientation towards the goal.
  def getOrientationStep(self, q_h_orient, q_g_orient, max_ang_step):
    ang = ut.quat_angle(q_h_orient, q_g_orient)

    ang_mag = abs(ang)
    step_fraction = 0.001
    if step_fraction * ang_mag > max_ang_step:
      # this is pretty much always true, can clean up the code.
      step_fraction = max_ang_step / ang_mag

    interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
    return interp_q_goal
        
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
  
  ## Try to get a waypoint pose from the trajectory deque. 
  # 
  # Also trims the trajectory to reduce the number of waypoint. 
  # Scans through the trajectory to find the next pose that is at least max_pos_step away (or the last pose in the deque).
  # @return A Pose object if there is a currently stored trajectory, otherwise None.
  def getWaypointFromTrajectory(self):      
    with self.state_lock:
      curr_gripper_pose = copy.copy(self.gripper_pose)
    with self.traj_lock:
      # If we have a current trajectory, represented as a sequence of Pose objects in a deque  
      if len(self.current_trajectory_deque) > 0:
        # Check if we need to trim the list
        if len(self.current_trajectory_deque) > 1:
          # Return the next point closest along the trajectory if we're close enough to it (eg within 5mm of it)
          if self.distanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.at_waypoint_threshold:# \
#            and self.angularDistanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.at_waypoint_ang_threshold:      
            # Trim the trajectory so that the current waypoint is a reasonable distance away - too fine steps make the controller unhappy.
            # Discard trajectory points that are within the min distance unless this the last point in the trajectory.
            # Adjust this by increasing or decreasing max_pos_step
            while self.distanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.max_pos_step and len(self.current_trajectory_deque) > 1:
              print "Trimming trajectory - dist: %s, len(deque): %s" %  (self.distanceBetweenPoses(self.gripper_pose, self.current_trajectory_deque[0]), len(self.current_trajectory_deque))
              self.current_trajectory_deque.popleft()
        
        desired_pose = self.current_trajectory_deque[0]#self.straightLineTrajectory(curr_gripper_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
        return desired_pose
     
      else: # We haven't got a valid trajectory. Return None. 
        return None  
  
  ## Publishes the next waypoint for the control to try to achieve.
  #
  # If we have a valid trajectory, get the next waypoint from that. 
  # If not and we have a valid end goal pose instead, move in a straight line towards it.
  # If we have neither, don't publish anything.   
  def generateWaypoint(self):   
    # TODO: Work out which way to generate the next waypoint. Currently always do a straight line 
    # update waypoint if we've achieved this waypoint, else keep current waypoint.
    with self.state_lock:
      tmp_curr_pose = copy.copy(self.gripper_pose)
    with self.goal_lock:
      tmp_goal_pose = copy.copy(self.goal_pose)  
   
    if tmp_curr_pose == None: # If we haven't heard robot state yet, don't act.
      return
 
    desired_pose = self.getWaypointFromTrajectory() # If we have a trajectory, return a valid pose (else None)
    if desired_pose == None and tmp_goal_pose != None: # If we have a teleop goal but don't have a trajectory, go in a straight line to the goal
      desired_pose = self.straightLineTrajectory(tmp_curr_pose, tmp_goal_pose, self.max_pos_step, self.max_ang_step)
    
    if desired_pose == None:
      return # Don't publish invalid waypoints.
      desired_pose = tmp_curr_pose 
 
    # Publish a waypoint every cycle.
    waypoint_msg = geometry_msgs.msg.PoseStamped()
    waypoint_msg.header = self.getMessageHeader()
    waypoint_msg.pose = desired_pose

    self.pose_waypoint_pub.publish(waypoint_msg)   

  ## Start the waypoint generator publishing waypoints.  
  def start(self):
    rate = rospy.Rate(self.rate) # 25Hz, nominally.
    rospy.loginfo("Beginning publishing waypoints")
    while not rospy.is_shutdown():
      self.generateWaypoint()
      #print rospy.Time()
      rate.sleep()

if __name__ == '__main__':
    # Set up input arg parser
    import optparse
    p = optparse.OptionParser()

    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    # Create and start the trajectory manager module. 
    traj_mgr = WaypointGenerator('mpc_traj_gen', opt) # loads all parameter sets on init
    traj_mgr.start()
    




