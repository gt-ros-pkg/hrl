#!/usr/bin/env python

## @package hrl_haptic_mpc
# 
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import roslib
roslib.load_manifest("hrl_tactile_controller")
import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import hrl_haptic_controllers_darpa_m3.epc_skin_math as esm # Core maths functions used by the MPC

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import hrl_haptic_manipulation_in_clutter_srvs.srv as haptic_srvs
import geometry_msgs.msg
import std_msgs.msg
import hrl_msgs.msg

# TODO - move to hrl_haptic_mpc package ie not just in src/
import multiarray_to_matrix # Helper class with functions to convert a Float64MultiArray to a list of numpy matrices
import haptic_mpc_util # Utility script with helper functions used across all haptic mpc nodes.
 
import numpy as np
import threading
import copy
import itertools as it
import sys
import math

## Container class for the controller parameters. 
#
# Some of these are parameters rather than control data, but they're small cf. the data which changes at each timestep.
class MPCData():
  def __init__(self, q, x_h, x_g, dist_g, 
           q_h_orient, q_g_orient, 
           position_weight, orient_weight, 
           control_point_joint_num, 
           Kc_l, Rc_l, Jc_l, Je, 
           delta_f_min, delta_f_max, 
           phi_curr, K_j, 
           loc_l, n_l, f_l, f_n,
           jerk_opt_weight, max_force_mag,
           jep, time_step, stop):
  
    self.q = q            # Joint angles
    self.x_h = x_h        # end effector position
    self.x_g = x_g        # end effector goal
    self.dist_g = dist_g  # dist to goal
    self.q_h_orient = q_h_orient # end effector orientation
    self.q_g_orient = q_g_orient
    self.position_weight = position_weight
    self.orient_weight = orient_weight
    self.control_point_joint_num = control_point_joint_num
    self.Kc_l = Kc_l
    self.Rc_l = Rc_l
    self.Jc_l = Jc_l
    self.Je = Je
    self.delta_f_min = delta_f_min
    self.delta_f_max = delta_f_max
    self.phi_curr = phi_curr
    self.K_j = K_j
    self.loc_l = loc_l
    self.n_l = n_l 
    self.f_l = f_l
    self.f_n = f_n
    self.jerk_opt_weight = jerk_opt_weight
    self.max_force_mag = max_force_mag
    self.jep = jep
    self.time_step = time_step
    self.stop = stop
  
  ## String representation of the data structure. Useful for debugging.
  def __str__(self):
    string = ""
    string += "MPC Data Structure:"
    string += "\nq: \t\t%s" % str(self.q)
    string += "\nx_h: \t\t%s" % str(self.x_h)
    string += "\nx_g: \t\t%s" % str(self.x_g)
    string += "\ndist_g: \t\t%s" % str(self.dist_g)  # dist to goal
    string += "\nq_h_orient: \t\t%s" % str(self.q_h_orient) # end effector orientation
    string += "\nq_g_orient: \t\t%s" % str(self.q_g_orient)
    string += "\nposition_weight: \t\t%s" % str(self.position_weight)
    string += "\norient_weight: \t\t%s" % str(self.orient_weight)
    string += "\ncontrol_point_joint_num: \t\t%s" % str(self.control_point_joint_num)
    string += "\nKc_l: \t\t%s" % str(self.Kc_l)
    string += "\nRc_l: \t\t%s" % str(self.Rc_l)
    string += "\nJc_l: \t\t%s" % str(self.Jc_l)
    string += "\nJe: \t\t%s" % str(self.Je)
    string += "\ndelta_f_min: \t\t%s" % str(self.delta_f_min)
    string += "\ndelta_f_max: \t\t%s" % str(self.delta_f_max)
    string += "\nphi_curr: \t\t%s" % str(self.phi_curr)
    string += "\nK_j: \t\t%s" % str(self.K_j)
    string += "\nloc_l: \t\t%s" % str(self.loc_l)
    string += "\nn_l: \t\t%s" % str(self.n_l)
    string += "\nf_l: \t\t%s" % str(self.f_l)
    string += "\nf_n: \t\t%s" % str(self.f_n)
    string += "\njerk_opt_weight: \t\t%s" % str(self.jerk_opt_weight)
    string += "\nmax_force_mag: \t\t%s" % str(self.max_force_mag)
    string += "\njep: \t\t%s" % str(self.jep)
    string += "\ntime_step: \t\t%s" % str(self.time_step)
    string += "\nstop: \t\t%s" % str(self.stop)
    return string
    
## Haptic Model Predictive Controller class.
class HapticMPC():
  ## Constructor
  # @param opt optparse options. Should be created using the helper utility script for reliability.
  # @param node_name Name used for the ROS node.
  def __init__(self, opt, node_name="haptic_mpc"):
    rospy.loginfo("Initialising Haptic MPC")
    self.opt = opt

    self.state_lock = threading.RLock() ## Haptic state lock
    self.goal_lock = threading.RLock() ## Goal state lock
    self.monitor_lock = threading.RLock() ## Monitor state lock
    self.gain_lock = threading.RLock() ## Controller gain state lock
    self.mpc_data = None ## MPCData data structure. 
    self.msg = None ## Haptic state message

    # Haptic State parameters - from a RobotHapticState listener
    self.last_msg_time = None
    self.timeout = 0.50 # If the last time the MPC heard a state message was >50ms ago -> Stop!
    self.waiting_to_resume = False
    self.waiting_for_no_errors = False 
    self.mpc_state = None ## Current MPC state. Stored as a list of strings
    self.mpc_error = None ## Current MPC errors. Stored as a list of strings
    
    self.joint_names = []
    self.joint_angles = []
    self.desired_joint_angles = []
    self.joint_velocities = []
    self.joint_stiffness = []
    self.joint_damping = []
    
    self.end_effector_pos = None
    self.end_effector_orient_quat = None
    
    self.skin_data = []
    self.Jc = []
    self.Je = []
    
    self.time_step = 0.01 # seconds. NB: This is a default which is set by the "start" function.
    
    # Trajectory goal position - from a PoseStamped listener
    self.goal_pos = None
    self.goal_orient_quat = None
    
    # Control parameters - read from parameter server
    self.orient_weight = 1.0
    self.pos_weight = 1.0
    self.deadzone_distance = 0.005 # 5mm 
    self.deadzone_angle = 10.0 # 10 degrees
    self.currently_in_deadzone = False

    self.mpc_enabled = True
    
    # Jacobian MultiArray to Matrix converter
    self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()
    
  ## Read parameters from the ROS parameter server and store them.
  def initControlParametersFromServer(self):
    base_path = '/haptic_mpc'
    control_path = '/control_params'
    
    rospy.loginfo("HapticMPC: Initialising controller parameters from server. Path: %s" % (base_path+control_path))
    # controller parameters
    # Force limits for the controller.
    self.allowable_contact_force = rospy.get_param(base_path + control_path + '/allowable_contact_force') # Max force allowed by the controller
    self.max_delta_force_mag = rospy.get_param(base_path + control_path + '/max_delta_force_mag') # Max change in force allowed.
    self.stopping_force = rospy.get_param(base_path + control_path + '/stopping_force') # Completely shut down if this exceeded
    
    self.goal_velocity_for_hand = rospy.get_param(base_path + control_path + '/goal_velocity_for_hand')
    self.deadzone_distance = rospy.get_param(base_path + control_path + '/deadzone_distance')
    self.deadzone_angle = rospy.get_param(base_path + control_path + '/deadzone_angle')
    
    # stiffness parameters
    self.static_contact_stiffness_estimate = rospy.get_param(base_path + control_path + '/static_contact_stiffness_estimate')
    self.estimate_contact_stiffness = rospy.get_param(base_path + control_path + '/estimate_contact_stiffness')
    
    self.orient_weight = rospy.get_param(base_path + control_path + '/orientation_weight')
    self.pos_weight = rospy.get_param(base_path + control_path + '/position_weight')
    self.jerk_opt_weight = rospy.get_param(base_path + control_path + '/jerk_opt_weight')
    self.mpc_weights_pub.publish(std_msgs.msg.Header(), self.pos_weight, self.orient_weight)

    self.frequency = rospy.get_param(base_path + control_path + '/frequency')
  
  ## Initialise the robot specific kinematic client.
  # This will eventually be moved to the robot haptic state message as the kinematic clients are only used for joint limits.
  # @param robot The robot type, specified as a string: eg 'pr2', 'cody', 'sim3', 'simcody'
  def initRobot(self, robot):
    if robot == "pr2":
      import pr2_arm_kinematics_darpa_m3
      if self.opt.arm == 'r' or self.opt.arm == 'l':
        self.robot_kinematics = pr2_arm_kinematics_darpa_m3.PR2ArmKinematics(self.opt.arm)
      else:
        rospy.logerr('Arm not specified for PR2 kinematics')
        sys.exit()
    elif robot == "sim3":
      import gen_sim_arms as sim_robot
      import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
      self.robot_kinematics = sim_robot.RobotSimulatorKDL(sim_robot_config) # KDL chain.
    elif robot == "cody" or robot == "simcody":
      import hrl_cody_arms.cody_arm_kinematics as cody_robot
      if self.opt.arm == 'r' or self.opt.arm == 'l':
        self.robot_kinematics = cody_robot.CodyArmKinematics(self.opt.arm)
      else:
        rospy.logerr('Arm not specified for Cody kinematics')
        sys.exit()
    else: # Unknown robot type
      rospy.logerr('Unknown robot type given. Valid options are: [\'pr2\', \'cody\', \'sim3\', \'simcody\']')
      sys.exit()
  
  ## getSkinData accessor function.
  # @return A copy of the skin_data dictionary, containing the latest taxel data. 
  def getSkinData(self):
    with self.state_lock:
      skin_data = copy.copy(self.skin_data)
    return skin_data
  
  ## getJointAngles accessor function.
  # @return A copy of the buffered joint angles list.
  def getJointAngles(self):
    with self.state_lock:
      joint_angles = copy.copy(self.joint_angles)
    return joint_angles
  
  ## getDesiredJointAngles accessor function.
  # @return A copy of the desired joint angles list, ie, the current arm controller setpoint.
  def getDesiredJointAngles(self):
    with self.state_lock:
      desired_joint_angles = copy.copy(self.desired_joint_angles)
    return desired_joint_angles
  
  ## getJointStiffness accessor function.
  # @return A copy of the joint stiffness parameters used by the controller.
  def getJointStiffness(self):
    with self.state_lock:
      joint_stiffness = copy.copy(self.joint_stiffness)
    return joint_stiffness
  
  ## getMPCData accessor function.
  # Returns a copy of the control data structure.
  def getMPCData(self):
    with lock:
      return copy.copy(self.mpc_data) 

  ## Builds the MPC data structure from the individual parameters
  # This is just a convenience data structure to amalgamate the data.
  def initMPCData(self):
    # Copy the latest data received from the robot haptic state.
    # To ensure the data is synchronised, all necessary data is copied with the lock rather than using the accessor functions.
    with self.state_lock:
      q = copy.copy(self.joint_angles)
      q_des = copy.copy(self.desired_joint_angles)
      Jc = copy.copy(self.Jc)  
      skin_data = copy.copy(self.skin_data)
      ee_pos = copy.copy(self.end_effector_pos)
      ee_orient_quat = copy.copy(self.end_effector_orient_quat)
      joint_stiffness = copy.copy(self.joint_stiffness)
      Je = copy.copy(self.Je)

    # Copy the goal parameters.
    with self.goal_lock:
      goal_pos = copy.copy(self.goal_pos)
      goal_orient_quat = copy.copy(self.goal_orient_quat)

    n_l = haptic_mpc_util.getNormals(skin_data)
    f_l = haptic_mpc_util.getValues(skin_data)
    loc_l = haptic_mpc_util.getLocations(skin_data)
    
    # Control params
    k_default = self.static_contact_stiffness_estimate
    dist_g = self.time_step * self.goal_velocity_for_hand 
    
    # Compute various matrices.
    Kc_l = self.contactStiffnessMatrix(n_l, k_default) # Using default k_est_min and k_est_max
    Rc_l = self.contactForceTransformationMatrix(n_l)
    
    # calculate the normal components of the current contact
    # forces
    tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
    f_n = np.matrix([tmp_i[0,0] for tmp_i in tmp_l]).T
    
    delta_f_min, delta_f_max = self.deltaForceBounds(f_n, 
                                                       max_pushing_force = self.allowable_contact_force,
                                                       max_pulling_force = self.allowable_contact_force,
                                                       max_pushing_force_increase = self.max_delta_force_mag, 
                                                       max_pushing_force_decrease = self.max_delta_force_mag, 
                                                       min_decrease_when_over_max_force = 0.,
                                                       max_decrease_when_over_max_force = 10.0)
      
    q_des_matrix = (np.matrix(q_des)[0:len(q_des)]).T
      
    K_j = np.diag(joint_stiffness)
   
    if len(Je) >= 1: # Je could potentially be a list of end effector jacobians. We only care about the first one in this implementation.
      Je = Je[0]
    
    # Calculate MPC orientation/position weight. This is a bad hack to essentially damp the response as it approaches the goal.
    with self.gain_lock:
        orient_weight = self.orient_weight
        position_weight = self.pos_weight
    planar = False   
 
    x_h = ee_pos # numpy array
    x_g = goal_pos
    if x_g == None:
      x_g = x_h
    q_h_orient = ee_orient_quat
    q_g_orient = goal_orient_quat
    if q_g_orient == None:
      q_g_orient = q_h_orient    
    
    dist_goal_2D = np.linalg.norm((x_g - x_h)[0:2])
    dist_goal_3D = np.linalg.norm(x_g - x_h)
    #if planar:
    #  dist_goal = dist_goal_2D
    #else:
    dist_goal = dist_goal_3D

    angle_error = ut.quat_angle(q_h_orient, q_g_orient)
    
    jerk_opt_weight = self.jerk_opt_weight   
 
    if orient_weight != 0:
      proportional_ball_radius = 0.1
      proportional_ball_dist_slope = 1.0
      if dist_goal < proportional_ball_radius:
        position_weight = position_weight * dist_goal/ (proportional_ball_radius * proportional_ball_dist_slope)

      jerk_opt_weight = max(jerk_opt_weight * position_weight * 2, jerk_opt_weight)

      proportional_ball_angle = math.radians(30)
      proportional_ball_angle_slope = 1.
      if angle_error < proportional_ball_angle:
        orient_weight = orient_weight * angle_error/ (proportional_ball_angle * proportional_ball_angle_slope)

    # Initialise and return the mpc data structure.
    mpc_data = MPCData( q=q, 
                        x_h = x_h, # numpy array
                        x_g = x_g, # numpy array
                        dist_g = dist_g, 
                        q_h_orient = q_h_orient, 
                        q_g_orient = q_g_orient, 
                        position_weight = position_weight, 
                        orient_weight = orient_weight, 
                        control_point_joint_num = len(q), # number of joints 
                        Kc_l = Kc_l, 
                        Rc_l = Rc_l, 
                        Jc_l = Jc, 
                        Je = Je, # NB: Je is a list of end effector jacobians (to use the same utils functions as Jc) 
                        delta_f_min = delta_f_min, 
                        delta_f_max = delta_f_max, 
                        phi_curr = q_des_matrix, 
                        K_j = K_j, 
                        loc_l = loc_l, # Contact locations from taxels. From skin client. 
                        n_l = n_l, # Normals for force locations
                        f_l = f_l, # Cartesian force components for contacts
                        f_n = f_n, # Normal force for contacts
                        jerk_opt_weight = jerk_opt_weight, # From control params
                        max_force_mag = self.allowable_contact_force, # From control params
                        jep = q_des, 
                        time_step = self.time_step, 
                        stop = False)
    return mpc_data

  ## Update the position/orientation weights used by the controller.
  # @param msg HapticMpcWeights message object
  def updateWeightsCallback(self, msg):   
    with self.gain_lock:
      rospy.loginfo("Updating MPC weights. Pos: %s, Orient: %s" % (str(msg.pos_weight), str(msg.orient_weight)))
      self.pos_weight = msg.pos_weight
      self.orient_weight = msg.orient_weight
      self.mpc_weights_pub.publish(msg)

  ## Store the current trajectory goal. The controller will always attempt a linear path to this.
  # @param msg A PoseStamped message
  def goalCallback(self, msg):
    with self.goal_lock:
      self.goal_pos = np.matrix([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])
      self.goal_orient_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
  
  ## Store the state information from the monitor node. Allows the control to be somewhat stateful.
  # The state and error fields are lists of strings indicating some state.
  # @param msg HapticMpcState message object
  def mpcMonitorCallback(self, msg):
    with self.monitor_lock:
      self.mpc_state = msg.state
      self.mpc_error = msg.error
  
  ## Store the robot haptic state.
  # @param msg RobotHapticState message object
  def robotStateCallback(self, msg):
    with self.state_lock:
      self.last_msg_time = rospy.Time.now() # timeout for the controller
      
      self.msg = msg
      self.joint_names = msg.joint_names
      self.joint_angles = list(msg.joint_angles) 
      self.desired_joint_angles = list(msg.desired_joint_angles)
      self.joint_velocities= list(msg.joint_velocities)
      self.joint_stiffness = list(msg.joint_stiffness)
      self.joint_damping = list(msg.joint_damping)
      
      self.end_effector_pos = np.matrix([[msg.hand_pose.position.x], [msg.hand_pose.position.y], [msg.hand_pose.position.z]])
      self.end_effector_orient_quat = [msg.hand_pose.orientation.x, msg.hand_pose.orientation.y, msg.hand_pose.orientation.z, msg.hand_pose.orientation.w]
      
      self.skin_data = msg.skins
 
      self.Je = self.ma_to_m.multiArrayToMatrixList(msg.end_effector_jacobian)
      self.Jc = self.ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)

  ## Interpolate a step towards the given goal orientation.
  # @param q_h_orient The current hand orientation as a quaternion in list form: [x,y,z,w]
  # @param q_g_orient The current goal orientation as a quaternion in list form: [x,y,z,w]
  # @return A desired change in orientation as a delta: 
  # TODO: clean this up so it doesn't use the "step" multiplier
  def goalOrientationInQuat(self, q_h_orient, q_g_orient, max_ang_step):
    # If the goal position is invalid, return the current end effector position 
    if not q_g_orient:
      q_g_orient = q_h_orient
    
    ang = ut.quat_angle(q_h_orient, q_g_orient)
    ang_mag = abs(ang)
    step_fraction = 0.1
    if step_fraction * ang_mag > max_ang_step:
      # this is pretty much always true, can clean up the code.
      step_fraction = max_ang_step / ang_mag

    interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
    delta_q_des = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_h_orient))

    return np.matrix(delta_q_des[0:3]).T
  
  ## Interpolate a step towards the given goal position.
  # @return A goal delta in position as a numpy matrix.
  # @param x_h The current hand position as a numpy matrix: [x,y,z]
  # @param x_g The current goal position as a numpy matrix: [x,y,z]
  def goalMotionForHand(self, x_h, x_g, dist_g): 
    # Goal should be None (if no goal pose has been heard yet, or a numpy column vector.
    if x_g == None or x_g.size == 0:
      x_g = x_h

    err = x_g - x_h
    err_mag = np.linalg.norm(err)

    # Advait: reducing the step size as the arm reaches close to
    # the goal. scale is something that I picked empirically on
    # Cody (I printed out err_mag and dist_g)
    #scale = 4.
    scale = 15.0
    if err_mag > dist_g * scale:
      delta_x_g = dist_g * (err/err_mag)
    else:
      delta_x_g = err/scale

    return delta_x_g


  
  ## Main control calculation.
  # @param mpc_dat An MPCData class, populated with relevant control parameters and current data
  # @return A list of joint equilibrium point deltas, ie, a desired increment to the current joint position(s).
  def deltaQpJepGen(self, mpc_dat):  
    # If we have invalid control data, do nothing
    if mpc_dat == None:
      return None
   
    # If we're within the deadzone band, do nothing
    dist_to_goal = np.linalg.norm(mpc_dat.x_h - mpc_dat.x_g)
    ang_to_goal = np.degrees(ut.quat_angle(mpc_dat.q_h_orient, mpc_dat.q_g_orient))
    dist_g = mpc_dat.dist_g
    ang_dist_g = np.radians(0.05)

    if dist_to_goal < self.deadzone_distance and (ang_to_goal < self.deadzone_angle and mpc_dat.orient_weight > 0.0):
      dist_g = 0.0 
      ang_dist_g = 0.0   
      if self.currently_in_deadzone == False:
        rospy.loginfo("MPC is in deadzone: pos %s (%s); orient %s (%s)" % (str(dist_to_goal), str(self.deadzone_distance), str(ang_to_goal), str(self.deadzone_angle) ))   
      self.currently_in_deadzone = True
      
      # If we're in the deadzone and have no forces, return zeroes.
      if len(mpc_dat.loc_l) == 0:
        return [0.0] * len(self.joint_angles)

    else:
      self.currently_in_deadzone = False
  
    # Execute control algorithm
    J_all = mpc_dat.Je
    delta_pos_g  = self.goalMotionForHand(mpc_dat.x_h, 
                                                    mpc_dat.x_g, 
                                                    dist_g)
    

    delta_orient_g = self.goalOrientationInQuat(mpc_dat.q_h_orient, 
                                                   mpc_dat.q_g_orient, 
                                                   ang_dist_g)

    if mpc_dat.position_weight == 0.:
      delta_x_g = delta_orient_g
      J_h = J_all[3:]
      T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                      * np.matrix(np.eye(3)) 
                      - haptic_mpc_util.getSkewMatrix(mpc_dat.q_h_orient[0:3]))
      J_h = T_quat*J_h

    elif mpc_dat.orient_weight == 0.:
      delta_x_g = delta_pos_g
      J_h = J_all[0:3]
    else:
      
      delta_x_g = np.vstack((delta_pos_g*np.sqrt(mpc_dat.position_weight), 
                             delta_orient_g*np.sqrt(mpc_dat.orient_weight)))
      
      J_h = J_all
      T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                      * np.matrix(np.eye(3)) 
                      - haptic_mpc_util.getSkewMatrix(mpc_dat.q_h_orient[0:3]))
      
      J_h[3:] = T_quat*J_h[3:]
      J_h[0:3] = J_h[0:3] * np.sqrt(mpc_dat.position_weight)
      J_h[3:] = J_h[3:] * np.sqrt(mpc_dat.orient_weight)


    J_h[:, mpc_dat.control_point_joint_num:] = 0.
    J_h = J_h[:,0:mpc_dat.K_j.shape[0]] # comes into play with Cody and the wrist cover


    # If torque constraints are violated - ie Q_des is far from Q - then reset Q_des to Q
    q_diff = np.array(mpc_dat.jep) - np.array(mpc_dat.q)
    max_q_diff = np.max(np.abs(q_diff))
    if max_q_diff > 15.0: # TODO: Hardcoded parameter used in the control function - move this elsewhere!
      # Reset JEP to Q.
      rospy.loginfo("JEPs too far from current position - resetting them to current position")
      self.publishDesiredJointAngles(self.getJointAngles())    

    cost_quadratic_matrices, cost_linear_matrices, \
    constraint_matrices, \
    constraint_vectors, lb, ub = esm.convert_to_qp(J_h, # 6xDOF
                                                   mpc_dat.Jc_l, # list of contacts
                                                   mpc_dat.K_j, # joint_stiffnesses (DOFxDOF)
                                                   mpc_dat.Kc_l, # list of 3x3s
                                                   mpc_dat.Rc_l, # list of 3x3s
                                                   mpc_dat.delta_f_min, # num contacts * 1
                                                   mpc_dat.delta_f_max,
                                                   mpc_dat.phi_curr, # DOF
                                                   delta_x_g, mpc_dat.f_n, mpc_dat.q,
                                                   self.robot_kinematics,
                                                   mpc_dat.jerk_opt_weight,     #between 0.000000001 and .0000000001, cool things happened
                                                   mpc_dat.max_force_mag)

    delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
                                                      cost_linear_matrices, 
                                                      constraint_matrices, 
                                                      constraint_vectors, 
                                                      lb, ub, 
                                                      debug_qp=False)

    # Updated joint positions.
    mpc_dat.jep[0:len(self.joint_names)] = (mpc_dat.phi_curr + delta_phi_opt).A1

    # warn if JEP goes beyond joint limits
    #if self.robot_kinematics.within_joint_limits(mpc_dat.jep) == False:
    #  rospy.logwarn('Outside joint limits. They will be clamped later...')
    #  rospy.logwarn("Limits: %s" % str(self.robot_kinematics.joint_lim_dict))
    #  rospy.logwarn("Current JEP: %s" % str(mpc_dat.jep))

    return delta_phi_opt # Return a joint position delta
    #return mpc_dat.jep # Return the an absolute joint position - old implementation

  
  ## Computes the contact stiffness matrix, K_ci.
  # @param n_l List of normal forces
  # #param k_default Default stiffness, N/m. Default is typically 200.0-1000.0
  def contactStiffnessMatrix(self, n_l, k_default, k_est_min=100.0, k_est_max=100000.0):
    # This computes the contact stiffness matrices, K_ci
    #
    # K_ci are size 3 x 3
    # K_c is size 3n x 3n
    Kc_l = []
    for n_ci in it.izip(n_l):
      # n_ci is a unit vector located at the point of contact,
      # or at the center of the taxel, that is normal to the
      # surface of the robot and points away from the surface of
      # the robot.
      #
      # This should result in a stiffness matrix that has a
      # stiffness of k in the direction of the normal and a
      # stiffness of 0 in directions orthogonal to the normal.
      #
      # If the contact location moves away from the current
      # contact location along the arm's surface normal, then
      # the magnitude of the force applied by the robot to the
      # environment should increase proportionally to the
      # distance the contact location has moved along the
      # surface normal. Given our conventions, this motion
      # projected onto the surface normal, n_ci, would result in
      # a positive value. Moreover, since we represent forces as
      # the force applied by the robot to the environment, the
      # resulting force vector should point in the same
      # direction.
      #
      n_ci = np.nan_to_num(n_ci)
      K_ci = k_default * np.outer(n_ci, n_ci)

      Kc_l.append(np.matrix(K_ci))
    return Kc_l

  ## Calculates the force transformation matrix from a list of force normals
  # @param n_l List of force normals
  # @return List of transformation matrices (rotations)
  def contactForceTransformationMatrix(self, n_l):        
    # Compute R_c, which as originally conceived, would consist of
    # rotation matrices that transform contact forces to the local
    # contact frame R_c f_c_global = f_c_local
    #
    # For now, R_c instead ignores all forces other than the force
    # normal to the surface of the robot's arm at the point of
    # contact. R_c is only used for the constraint matrices. So,
    # R_ci recovers the component of the contact force, f_ci, that
    # is normal to the surface. If f_ci points toward the arm,
    # then it represents the robot pulling on the environment. If
    # f_ci points away from the arm, then it represents the robot
    # pushing on the environment.
    #
    # R_ci f_ci = f_norm_scalar
    # R_ci is size 1 x 3
    # R_c is size n x 3n
    # 
    Rc_l = []
    for n_ci in n_l:
      # n_ci is a unit vector that is normal to the surface of
      # the robot and points away from the surface of the robot.
      R_ci = np.matrix(np.zeros((1,3)))
      R_ci[:] = n_ci[:].T
      Rc_l.append(R_ci)
    return Rc_l

  ## Compute bounds for delta_f
  # @retval delta_f_min Minimum bound
  # @retval delta_f_max Maximum bound
  def deltaForceBounds(self, f_n, 
                     max_pushing_force, max_pulling_force,
                     max_pushing_force_increase, max_pushing_force_decrease, 
                     min_decrease_when_over_max_force, 
                     max_decrease_when_over_max_force):
    # Compute bounds for delta_f:  delta_f_max and delta_f_min
    #
    # all inputs should be positive
    assert (max_pushing_force >= 0.0), "delta_f_bounds: max_pushing_force = %f < 0.0" % max_pushing_force
    assert (max_pushing_force_increase >= 0.0), "delta_f_bounds: max_pushing_force_increase = %f < 0.0" % max_pushing_force_increase
    assert (max_pushing_force_decrease >= 0.0), "delta_f_bounds: max_pushing_force_decrease = %f < 0.0" % max_pushing_force_decrease
    assert (max_pulling_force >= 0.0), "delta_f_bounds: max_pulling_force = %f < 0.0" % max_pulling_force
    assert (min_decrease_when_over_max_force >= 0.0), "delta_f_bounds: min_decrease_when_over_max_force = %f < 0.0" % min_decrease_when_over_max_force
    assert (max_decrease_when_over_max_force >= 0.0), "delta_f_bounds: max_decrease_when_over_max_force = %f < 0.0" % max_decrease_when_over_max_force
    
    # Set delta_f_max. The change to the normal components of
    # the contact forces must be less than these
    # values. delta_f_max limits the magnitude of the force
    # with which the robot can push on the environment at each
    # of its contact locations.
    #
    # f_max is size n x 1
    #
    # Compute how much the contact force can change before it hits
    # the maximum, and limit the expected increase in force by
    # this quantity.
    n = f_n.shape[0]
    f_max = max_pushing_force * np.matrix(np.ones((n,1)))
    delta_f_max = f_max - f_n
    # Also incorporate constraint on the expected increase in the
    # contact force for each contact. This limits the rate of
    # increase in pushing force.
    delta_f_max = np.minimum(delta_f_max, 
                             max_pushing_force_increase * np.matrix(np.ones((n,1))))

    # Set delta_f_min. The change to the normal components of
    # the contact forces must be greater than these
    # values. delta_f_min limits the magnitude of the force
    # with which the robot can pull on the environment at each
    # of its contact locations.
    #
    # f_min is size n x 1
    #        
    f_min = -max_pulling_force * np.matrix(np.ones((n,1)))
    delta_f_min = f_min - f_n
    # Also incorporate constraint on the expected change of
    # the contact force for each contact
    delta_f_min = np.maximum(delta_f_min, 
                             -max_pushing_force_decrease * np.matrix(np.ones((n,1))))

    # # Setting negative values of delta_f_min to large negative
    # # numbers so that adhesive forces are not a binding constraint
    delta_f_min[np.where(delta_f_min<=0)]=-10000

    # If a force has exceeded the maximum use special constraints.
    over_max = f_n > max_pushing_force
    if over_max.any():
      # at least one of the contact forces is over the maximum allowed
      delta_f_max[over_max] = -min_decrease_when_over_max_force
      delta_f_min[over_max] = -max_decrease_when_over_max_force

    return delta_f_min, delta_f_max

  ## Publishes a list of Deltas for the desired joint positions.
  # @param ctrl_data List of desired joint position deltas to be published
  def publishDeltaControlValues(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.delta_q_des_pub.publish(msg)
  
  ## Publish a desired joint position list directly (rather than deltas)
  # @param ctrl_data List of desired joint positions to be published (not deltas).
  def publishDesiredJointAngles(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.q_des_pub.publish(msg)

  ## Main control function. Builds the control data structure and passes it to the control calculation routine
  def updateController(self):
    # Check for controller enabled
    if not self.mpc_enabled:
      return  
  
    # Check for haptic state msg timeout
    with self.state_lock:
      time_since_last_msg = rospy.Time.now() - self.last_msg_time # will always exist as we block on startup until a haptic state msg is heard.

    # Timeout based on time since the last heard message.
    # TODO: Replace this with a freshness parameter on the skin data. More flexible than a timeout.
#    if time_since_last_msg.to_sec() > self.timeout:
#      if self.waiting_to_resume == False:
#        rospy.logwarn("MPC hasn't heard a haptic state messages for %s s. Stopping control effort." % str(time_since_last_msg.to_sec()))
#        self.waiting_to_resume = True
#        #return from updateController without actually doing anything
#        return
#    else:
#      if self.waiting_to_resume == True:
#        self.waiting_to_resume = False
#        rospy.logwarn("MPC resumed control - haptic state messages received again.")
#    
    # Listen to a state topic from the monitor node and respond to errors.
    with self.monitor_lock:
      mpc_state = copy.copy(self.mpc_state)
      mpc_error = copy.copy(self.mpc_error)
    
    if mpc_error != None and len(mpc_error) > 0:
      if self.waiting_for_no_errors == False:
        rospy.logwarn("HapticMPC: MPC monitor detected error conditions. Stopping control effort.\nState:\n%s\nErrors:\n%s" % (str(mpc_state), str(mpc_error)))
        self.waiting_for_no_errors = True
      return
    else:
      if self.waiting_for_no_errors == True:
        self.waiting_for_no_errors = False
        rospy.logwarn("HapticMPC: MPC resumed control - errors cleared.")
    
    # If good, run controller.
    mpc_data = self.initMPCData()
    desired_joint_pos = self.deltaQpJepGen(mpc_data) # deltas
    if desired_joint_pos == None:
      desired_joint_pos = [0.0] * len(self.joint_angles)
    self.publishDeltaControlValues(desired_joint_pos)
    

  ## Initialise the ROS communications - init node, subscribe to the robot state and goal pos, publish JEPs
  def initComms(self, node_name):
    rospy.init_node(node_name)
    self.robot_state_sub = rospy.Subscriber("/haptic_mpc/robot_state", haptic_msgs.RobotHapticState, self.robotStateCallback)
    self.goal_sub = rospy.Subscriber("/haptic_mpc/traj_pose", geometry_msgs.msg.PoseStamped, self.goalCallback)
    self.delta_q_des_pub= rospy.Publisher("/haptic_mpc/delta_q_des", hrl_msgs.msg.FloatArrayBare)
    self.q_des_pub = rospy.Publisher("/haptic_mpc/q_des", hrl_msgs.msg.FloatArrayBare)
    self.mpc_monitor_sub = rospy.Subscriber("/haptic_mpc/mpc_state", haptic_msgs.HapticMpcState, self.mpcMonitorCallback)
    self.mpc_weights_sub = rospy.Subscriber("haptic_mpc/weights", haptic_msgs.HapticMpcWeights, self.updateWeightsCallback)
    self.mpc_weights_pub = rospy.Publisher("haptic_mpc/current_weights", haptic_msgs.HapticMpcWeights, latch=True)
  
    self.enable_mpc_srv = rospy.Service("/haptic_mpc/enable_mpc", haptic_srvs.EnableHapticMPC, self.enableHapticMPC)
  
  ## Enable Haptic MPC service handler (default is enabled).
  def enableHapticMPC(req):
    if req.new_state == "enabled":
      self.mpc_enabled = True
    else:
      self.mpc_enabled = False

    return haptic_srvs.EnableHapticMPCResponse(self.mpc_state)  


  ## Start the control loop once the controller is initialised.
  def start(self):
    self.initRobot(self.opt.robot)
    self.initComms("haptic_mpc")
    rospy.sleep(1.0) # Delay to allow the ROS node to initialise properly.
    self.initControlParametersFromServer()
    
    r = rospy.Rate(self.frequency)
    self.time_step = 1.0/self.frequency # Default: 0.02s at 50 Hz

    rospy.loginfo("HapticMPC: Waiting for Robot Haptic State message")
    while not self.getJointAngles():
      r.sleep()
    rospy.loginfo("HapticMPC: Got Robot Haptic State message")

    rospy.loginfo("HapticMPC: Resetting desired joint angles to current position")
    self.publishDesiredJointAngles(self.getJointAngles())    

    # Main control loop
    rospy.loginfo("HapticMPC: Starting MPC")
    while not rospy.is_shutdown():
      self.updateController()
      #rospy.spin() # For debug - run once
      r.sleep()

if __name__== "__main__":
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)
  
  mpc_controller = HapticMPC(opt, "haptic_mpc")
  mpc_controller.start()

