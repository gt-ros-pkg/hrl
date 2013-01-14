#!/usr/bin/env python

import math
import numpy as np
import threading
import copy
import sys, os
import datetime

import roslib
roslib.load_manifest("hrl_tactile_controller")
import rospy

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import hrl_haptic_manipulation_in_clutter_srvs.srv as haptic_srvs
import geometry_msgs.msg
import std_msgs.msg

import hrl_lib.util as ut
import hrl_lib.circular_buffer as cb
import hrl_lib.transforms as tr

import haptic_mpc_util
import multiarray_to_matrix

class HapticMPCLogger():
  def __init__(self):
    self.rate = 100.0 # Hz
    
    self.logger_lock = threading.RLock()
    self.currently_logging = False
    
    self.state_lock = threading.RLock()
    self.traj_lock = threading.RLock()
    self.goal_lock = threading.RLock()
    
    self.traj_pos = None
    self.traj_orient_quat = None
    
    # Jacobian MultiArray to Matrix converter
    self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()
    
    self.initComms()
    
    # Buffers for logging
    # buffers that would be common to all the controllers. Each of
    # these buffers will be logged into a pkl and cleared after
    # every run of one of the controllers.

    self.hist_size = 20000

    self.ee_pos_buf = cb.CircularBuffer(self.hist_size, (3,))
    self.max_force_buf = cb.CircularBuffer(self.hist_size, ())

    # magnitude of all contact forces.
    self.all_forces_buf = cb.CircularBuffer(self.hist_size, ())

    # locations of the contact forces
    self.all_forces_locs_buf = cb.CircularBuffer(self.hist_size, (3,))

    # normals of the contact forces
    self.all_forces_nrmls_buf = cb.CircularBuffer(self.hist_size, (3,))

    # jt num (as returned by the skin client).
    self.all_forces_jts_buf = cb.CircularBuffer(self.hist_size, ())

    # use this buffer to figure out mapping between joint angles
    # and all the contact forces corresponding to that arm
    # configuration
    self.num_contacts_at_time_instant_buf = cb.CircularBuffer(self.hist_size, ())

    n_jts = 7 # TODO: Only 3 for sim. Need 7 for PR2.
    self.jep_buf = cb.CircularBuffer(self.hist_size, (n_jts,))
    self.q_buf = cb.CircularBuffer(self.hist_size, (n_jts,))
    self.qdot_buf = cb.CircularBuffer(self.hist_size, (n_jts,))

    self.time_stamp_buf = cb.CircularBuffer(self.hist_size, ())

    self.ee_gaussian_mn_buf_50 = cb.CircularBuffer(self.hist_size, (3,))
    self.ee_gaussian_cov_buf_50 = cb.CircularBuffer(self.hist_size, (3,3))
    self.mean_motion_buf = cb.CircularBuffer(self.hist_size, ())
    

  def initComms(self, node_name="haptic_mpc_logger"):
    rospy.init_node(node_name)
    
    self.mpc_state_pub = rospy.Publisher("/haptic_mpc/mpc_state", haptic_msgs.HapticMpcState)
    
    self.robot_state_sub = rospy.Subscriber("/haptic_mpc/robot_state", haptic_msgs.RobotHapticState, self.robotStateCallback)
    self.traj_sub = rospy.Subscriber("/haptic_mpc/traj_pose", geometry_msgs.msg.PoseStamped, self.trajPoseCallback)
    self.goal_sub = rospy.Subscriber("/haptic_mpc/goal_pose", geometry_msgs.msg.PoseStamped, self.goalPoseCallback)
    
    self.logger_state_srv = rospy.Service('haptic_mpc_logger_state', haptic_srvs.HapticMPCLogging, self.changeLoggerStateSrv)
  
   
  # Service provided to change logger state
  # If we're currently logging and are asked to start logging to a new file - save to disk and start logging new data to the new file
  # If asked to stop, save to disk.
  def changeLoggerStateSrv(self, req):    
    with self.logger_lock:
      if req.log_state == "start": # Request to start logging
        rospy.logwarn("MPC Logger: Requested to start logging to: %s/%s" % (os.path.abspath(req.log_dir), req.log_file))
        if self.currently_logging: 
          rospy.logwarn("MPC Logger: Currently logging, saving existing log to disk first: %s/%s" % (os.path.abspath(self.log_dir), self.log_file))
          self.saveDataToDisk()
          self.clearLog()
        
        self.log_file = req.log_file
        self.log_dir = req.log_dir
        self.currently_logging = True # Start logging data to the buffers (if we aren't already)
      
      if req.log_state == "stop":
        if self.currently_logging == True:
          rospy.logwarn("MPC Logger: Stopping logging. Writing data to disk: %s/%s" % (os.path.abspath(self.log_dir), self.log_file))
          self.currently_logging = False # Stop logging data to the buffers
          self.saveDataToDisk()
          self.clearLog()
          
          self.log_dir = None
          self.log_file = None      
      
      return haptic_srvs.HapticMPCLoggingResponse()
      
  # Store the current trajectory goal sent to the trajectory generator.
  # Expects a PoseStamped message
  def goalPoseCallback(self, msg):
    with self.goal_lock:
      self.goal_pos = np.matrix([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])
      self.goal_orient_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]    
  
  # Store the current trajectory waypoint, generated by the trajectory generator.
  # Expects a PoseStamped message
  def trajPoseCallback(self, msg):
    with self.traj_lock:
      self.traj_pos = np.matrix([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])
      self.traj_orient_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]  
  
  # Store the robot state  
  def robotStateCallback(self, msg):
    with self.state_lock:
      self.msg = msg
      self.joint_names = msg.joint_names
      self.joint_angles = list(msg.joint_angles) 
      self.desired_joint_angles = list(msg.desired_joint_angles)
      self.joint_velocities= list(msg.joint_velocities)
      self.joint_stiffness = list(msg.joint_stiffness)
      self.joint_damping = list(msg.joint_damping)
      
      self.torso_position = np.matrix([[msg.torso_pose.position.x], [msg.torso_pose.position.y], [msg.torso_pose.position.z]])
      self.torso_orient_quat = [msg.torso_pose.orientation.x, msg.torso_pose.orientation.y, msg.torso_pose.orientation.z, msg.torso_pose.orientation.w]
      self.torso_rotation = tr.quaternion_to_matrix(self.torso_orient_quat)
      
      self.end_effector_pos = np.matrix([[msg.hand_pose.position.x], [msg.hand_pose.position.y], [msg.hand_pose.position.z]])
      self.end_effector_orient_quat = [msg.hand_pose.orientation.x, msg.hand_pose.orientation.y, msg.hand_pose.orientation.z, msg.hand_pose.orientation.w]
      
      self.skin_data = msg.skins
 
      self.Je = self.ma_to_m.multiArrayToMatrixList(msg.end_effector_jacobian)
      self.Jc = self.ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)

  # TODO: This may need to be spawned in a separate thread if files are large - will cause whatever calls it to block.
  def saveDataToDisk(self):
    dir_name = os.path.abspath(self.log_dir)
    try:
      os.makedirs(dir_name)
    except OSError:
      if os.path.exists(dir_name):
        pass # Already exists
      else:
        raise # There was an error on creation, so make sure we know about it
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filepath = dir_name + "/" + self.log_file + "_" + timestamp + ".pkl"
    #filepath = dir_name + "/" + self.log_file +".pkl"
    #filepath can either be a relative or absolute path
    filepath = os.path.abspath(filepath) # Just checking!
    dict = self.createLogDict("haptic_mpc")
    ut.save_pickle(dict, filepath)

  # Pass in a list of SIM link names - this returns a corresponding list of joint indices
  # NB: SIM ONLY. PR2/Cody will differ
  def convertLinkNamesToJointNums(self, links_list):
    joints_list = []
    for link in links_list:
      if link == "link3":
        joints_list.append(2)
      elif link=="link2":
        joints_list.append(1)
      elif link=="link1":
        joints_list.append(0)
    return joints_list

  # Called by the main logger loop to push current data to the circular buffers.
  def logDataToBuffers(self):
    # Copy current state while locked.
    with self.state_lock:
      skin_data = copy.copy(self.skin_data)
      jep = copy.copy(self.desired_joint_angles)
      q = copy.copy(self.joint_angles)
      q_dot = copy.copy(self.joint_velocities)
      ee = copy.copy(self.end_effector_pos)
      
    # Process data
    locations_list, normals_list, values_list, links_list = haptic_mpc_util.getAllTaxelArrayDataAsLists(skin_data)
    joints_list = self.convertLinkNamesToJointNums(links_list)
    # Log force info
    max_force = 0.0 
    if values_list:
      mat = np.column_stack(values_list)
      f_mag_l = ut.norm(mat).A1.tolist()
      
      for f_mag, nrml, loc, jt in zip(f_mag_l, normals_list, locations_list, joints_list):
        self.all_forces_buf.append(f_mag)
        self.all_forces_locs_buf.append(loc.A1)
        self.all_forces_nrmls_buf.append(nrml.A1)
        self.all_forces_jts_buf.append(jt)
      
      max_force = np.max(f_mag_l)
      
    self.max_force_buf.append(max_force)
    self.num_contacts_at_time_instant_buf.append(len(values_list))

    # Log robot state
    self.jep_buf.append(jep)
    self.q_buf.append(q)
    self.qdot_buf.append(q_dot)
    self.ee_pos_buf.append(ee.A1) # end effector position - not orientation

    # fit Gaussian to last 50 locations of the end effector.
    self.updateEeGaussianBuf(self.ee_gaussian_mn_buf_50,
                                self.ee_gaussian_cov_buf_50, 50)
    # find distance (in 2D) between the means of the Gaussians
    # that are 200 samples apart.
    self.updateMeanMotion(self.ee_gaussian_mn_buf_50, step=200)
      
    self.time_stamp_buf.append(rospy.get_time())

  def updateEeGaussianBuf(self, mn_buf, cov_buf, hist_size):
    if len(self.ee_pos_buf) < hist_size:
        return
    ee_hist = self.ee_pos_buf.get_last(hist_size)
    ee_hist = np.matrix(ee_hist).T
    mn_buf.append(np.mean(ee_hist, 1).A1)
    cov_buf.append(np.cov(ee_hist))

  def updateMeanMotion(self, mn_buf, step):
    if len(mn_buf) < step:
        return
    d = np.linalg.norm((mn_buf[-1] - mn_buf[-step])[0:2])
    self.mean_motion_buf.append(d)

  def clearLog(self):
    # common to all controllers.
    self.max_force_buf.clear()
    self.all_forces_buf.clear()
    self.all_forces_locs_buf.clear()
    self.all_forces_nrmls_buf.clear()
    self.all_forces_jts_buf.clear()
    self.num_contacts_at_time_instant_buf.clear()
    self.ee_pos_buf.clear()
    self.jep_buf.clear()
    self.q_buf.clear()
    self.qdot_buf.clear()
    self.ee_gaussian_mn_buf_50.clear()
    self.ee_gaussian_cov_buf_50.clear()
    self.mean_motion_buf.clear()
    self.time_stamp_buf.clear()  
      
  # Called when data should be dumped to disk (infrequent)
  def createLogDict(self, controller):
    d = {}
    with self.state_lock:
      d['max_force_list'] = self.max_force_buf.to_list()
      d['all_forces_list'] = self.all_forces_buf.to_list()
      d['all_forces_locs_list'] = self.all_forces_locs_buf.to_list()
      d['all_forces_nrmls_list'] = self.all_forces_nrmls_buf.to_list()
      d['all_forces_jts_list'] = self.all_forces_jts_buf.to_list()
      d['num_contacts_at_time_instant_list'] = self.num_contacts_at_time_instant_buf.to_list()
      d['ee_pos_list'] = self.ee_pos_buf.to_list()
      d['jep_list'] = self.jep_buf.to_list()
      d['q_list'] = self.q_buf.to_list()
      d['qdot_list'] = self.qdot_buf.to_list()
      d['ee_gaussian_50_mn_list'] = self.ee_gaussian_mn_buf_50.to_list()
      d['ee_gaussian_50_cov_list'] = self.ee_gaussian_cov_buf_50.to_list()
      d['mean_motion_list'] = self.mean_motion_buf.to_list()
      d['time_stamp_list'] = self.time_stamp_buf.to_list() 
      d['torso_position'] = self.torso_position # Instantaneous
      d['torso_rotation'] = self.torso_rotation
    
    with self.traj_lock:
      d['local_goal'] = self.traj_pos
      
    d['controller'] = controller
    
    return d
  
  def start(self):
    rospy.loginfo("HapticMPCLogger: Starting Haptic MPC Logger")
    rate = rospy.Rate(self.rate) # 100Hz, nominally.
    
    rospy.loginfo("HapticMPCLogger: Waiting for Robot Haptic State message")
    rospy.wait_for_message("/haptic_mpc/robot_state", haptic_msgs.RobotHapticState)
    rospy.loginfo("HapticMPCLogger: Got Robot Haptic State message")
    
    rospy.loginfo("HapticMPCLogger: Ready to start logging. Waiting for service call.")
    while not rospy.is_shutdown():
      with self.logger_lock:
        #if self.currently_logging:
        self.logDataToBuffers()
      rate.sleep()

if __name__=="__main__":
  # Parse an options list specifying robot type
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)
  
  
  mpc_logger = HapticMPCLogger()
  mpc_logger.start()
