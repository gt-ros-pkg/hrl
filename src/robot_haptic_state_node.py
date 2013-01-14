#!/usr/bin/env python

## @package hrl_haptic_mpc
#
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import math
import numpy as np
import threading
import itertools as it

import roslib
roslib.load_manifest("hrl_tactile_controller")
import rospy
import geometry_msgs.msg as geom_msgs
from tf import TransformListener
from std_msgs.msg import Header

from hrl_lib import transforms as tr
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import multiarray_to_matrix
import haptic_mpc_util

## @class RobotHapticStateServer Haptic state publisher: publishes all relevant haptic state information on a common interface independent of robot type.
class RobotHapticStateServer():
  ## Constructor for robot haptic state server
  def __init__(self, opt, node_name="robot_haptic_state"):
    self.opt = opt
    # Set up all ros comms to start with
    self.node_name = node_name
    rospy.init_node(self.node_name)
    self.tf_listener = None
    self.state_pub = None
    self.rate = 100.0 # 100 Hz.
    self.msg_seq = 0 # Sequence counter

    # ROS Param server paths.
    self.base_path = "/haptic_mpc"

    # Skin data
    self.skin_topic_list = [] # List of topic names
    self.skin_client = None

    # Robot object. Contains all the subscribers and robot specific kinematics, etc
    self.robot = None

    # Joint data
    self.joint_names = []
    self.joint_angles = []
    self.desired_joint_angles = []
    self.joint_velocities = []
    self.joint_stiffness = []
    self.joint_damping = []
    self.joint_data_lock = threading.RLock()
    self.joint_names = []

    # End effector pose
    self.end_effector_position = None
    self.end_effector_orient_cart = None
    self.end_effector_orient_quat = None

    # Jacobian storage
    self.Jc = None # Contact jacobians
    self.Je = None # End effector jacobian
    self.trim_threshold = 1.0 #this is 1.0 for forces

    # Jacobian MultiArray to Matrix converter
    self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()

    # Initialise various parameters.
    self.initComms()
    self.initRobot(self.opt.robot)

  ## Initialise all robot specific parameters (skin topics, robot interfaces, etc). Calls the appropriate init function.
  def initRobot(self, robot_type="pr2"):
    if robot_type == "pr2":
      self.initPR2()
    elif robot_type == "cody":
      self.initCody()
    elif robot_type == "sim3":
      self.initSim()
    elif robot_type == "simcody":
      self.initSimCody()
    else:
      rospy.logerr("RobotHapticState: Invalid robot type specified")
      sys.exit()

  ## Initialise parameters for the state publisher when used on the PR2.
  def initPR2(self):
    # Robot kinematic classes and skin clients. These are specific to each robot
    import urdf_arm_darpa_m3 as urdf_arm
    import pr2_skin_client as pr2_sc

    # Load parameters from ROS Param server
    self.robot_path = "/pr2"
    self.skin_topic_list = rospy.get_param(self.base_path +
                                           self.robot_path +
                                           '/skin_list/fabric')
    self.torso_frame = rospy.get_param(self.base_path +
                                       self.robot_path +
                                       '/torso_frame' )
    self.inertial_frame = rospy.get_param(self.base_path +
                                          self.robot_path +
                                          '/inertial_frame')
    rospy.loginfo("RobotHapticState: Initialising PR2 haptic state publisher" +
                  "with the following skin topics: \n%s"
                  %str(self.skin_topic_list))
    self.skin_client = pr2_sc.PR2_SkinClient(self.skin_topic_list,
                                             self.torso_frame,
                                             self.tf_listener)
    self.skin_client.setTrimThreshold(self.trim_threshold)
    rospy.loginfo("RobotHapticState: Initialising robot interface")
    if not self.opt.arm:
      rospy.logerr("RobotHapticState: No arm specified for PR2")
      sys.exit()
    self.robot = urdf_arm.URDFArm(self.opt.arm, self.tf_listener)

  ## Initialise parameters for the state publisher when used on Cody.
  def initCody(self):
    import hrl_cody_arms.cody_arm_client as cac
    import cody_guarded_move as cgm

    # Load the skin list from the param server
    self.robot_path = '/cody'
    self.skin_topic_list = rospy.get_param(self.base_path +
                                           self.robot_path +
                                           '/skin_list')
    self.torso_frame = rospy.get_param(self.base_path +
                                       self.robot_path +
                                       '/torso_frame' )
    self.inertial_frame = rospy.get_param(self.base_path +
                                          self.robot_path +
                                          '/inertial_frame')
    rospy.loginfo("RobotHapticState: Initialising Cody haptic state publisher" +
                  "with the following skin topics: \n%s"
                  %str(self.skin_topic_list))
    self.skin_client = cody_skin_client.CodySkinClient(self.skin_topic_list,
                                                       self.torso_frame,
                                                       self.tf_listener)
    self.skin_client.setTrimThreshold(self.trim_threshold)
    rospy.loginfo("RobotHapticState: Initialising robot interface")
    if not self.opt.arm:
      rospy.logerr("RobotHapticState: No arm specified for Cody")
      sys.exit()
    self.robot = sim_robot.ODESimArm(sim_config)

  ## Initialise parameters for the state publisher when used in simulation
  #with the 3DOF arm.
  def initSim(self):
    import gen_sim_arms as sim_robot
    import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
    import sim_skin_client as sim_sc
    # Load the skin list from the param server
    self.robot_path = '/sim3'
    self.skin_topic_list = rospy.get_param(self.base_path +
                                           self.robot_path +
                                           '/skin_list')
    self.torso_frame = rospy.get_param(self.base_path +
                                       self.robot_path +
                                       '/torso_frame')
    self.inertial_frame = rospy.get_param(self.base_path +
                                          self.robot_path +
                                          '/inertial_frame')
    rospy.loginfo("RobotHapticState: Initialising Sim haptic state publisher" +
                  "with the following skin topics: \n%s"
                  %str(self.skin_topic_list))
    self.skin_client = sim_sc.SimSkinClient(self.skin_topic_list,
                                            self.torso_frame,
                                            self.tf_listener)
    self.skin_client.setTrimThreshold(self.trim_threshold)

    # TODO: Add config switching here.
    rospy.loginfo("RobotHapticState: Initialising Sim robot interface")
    sim_config = sim_robot_config
    self.robot = sim_robot.ODESimArm(sim_config)
    #jep_start = np.radians([-100.0, 110, 110])
    #self.robot.set_ep(jep_start)

  #Initialise parameters for the state publisher when used in simulation
  #with the 7DOF cody arm.
  def initSimCody(self):
    import cody_arm_darpa_m3 as cody_arm
    import sim_skin_client as sim_sc

    # Load the skin list from the param server
    self.robot_path = '/simcody'
    self.skin_topic_list = rospy.get_param(self.base_path +
                                           self.robot_path +
                                           '/skin_list')
    self.torso_frame = rospy.get_param(self.base_path +
                                       self.robot_path +
                                       '/torso_frame' )
    self.inertial_frame = rospy.get_param(self.base_path +
                                          self.robot_path +
                                          '/inertial_frame')
    rospy.loginfo("RobotHapticState: Initialising Sim haptic state publisher" +
                  "with the following skin topics: \n%s"
                  %str(self.skin_topic_list))
    self.skin_client = sim_sc.SimSkinClient(self.skin_topic_list,
                                            self.torso_frame,
                                            self.tf_listener)
    self.skin_client.setTrimThreshold(self.trim_threshold)

    # TODO: Add config switching here.
    rospy.loginfo("RobotHapticState: Initialising robot interface")
    if not self.opt.arm:
      rospy.logerr("RobotHapticState: No arm specified for Sim Cody")
      sys.exit()

    self.robot = cody_arm.CodyArmClient(self.opt.arm)


  # Initialise publishers for the robot haptic state,
  # the current gripper pose, and a TF listener.
  # NB: The skin client and robot clients will have their own
  # publishers/subscribers specific to them.
  def initComms(self):
    self.tf_listener = TransformListener()
    self.state_pub = rospy.Publisher('/haptic_mpc/robot_state',
                                     haptic_msgs.RobotHapticState)
    self.gripper_pose_pub = rospy.Publisher('/haptic_mpc/gripper_pose',
                                            geom_msgs.PoseStamped)

  # Returns a header type with the current timestamp.
  # Does not set the frame_id
  def getMessageHeader(self):
    header = Header()
    header.stamp = rospy.get_rostime()
    return header

  # Updates the stored end effector Jacobian from the current joint angles
  # and end effector position
  def updateEndEffectorJacobian(self):
    self.Je = [self.robot.kinematics.jacobian(self.joint_angles, self.end_effector_position)]

  ## Compute contact Jacobians based on the provided taxel array dictionary
  # @param skin_data Dictionary containing taxel array messages indexed by topic name
  def updateContactJacobians(self, skin_data):
    # loc_l = list of taxel locations relative the "torso_lift_link" frame.
    # jt_l = list of joints beyond which the jacobian columns are zero.
    # loc_l. jt_l from skin client.
    Jc_l = []
    loc_l, jt_l = self.skin_client.getTaxelLocationAndJointList(skin_data)

    if len(loc_l) != len(jt_l):
      while True: #FIXME: This seems like a bad way to handle this...what am I missing? -Phil
          rospy.sleep(2.0)

    for jt_li, loc_li in it.izip(jt_l, loc_l):
      Jc = self.robot.kinematics.jacobian(self.joint_angles, loc_li)
      Jc[:, jt_li+1:] = 0.0
      Jc = Jc[0:3, 0:len(self.joint_stiffness)] # trim the jacobian to suit the number of DOFs.
      Jc_l.append(Jc)
    self.Jc = Jc_l

  ## Returns a Pose object for the torso pose in the stated inertial frame
  def updateTorsoPose(self):
    # Get the transformation from the desired frame to current frame
    self.tf_listener.waitForTransform(self.inertial_frame, self.torso_frame,
                                      rospy.Time(), rospy.Duration(10.0))
    t1, q1 = self.tf_listener.lookupTransform(self.inertial_frame,
                                              self.torso_frame,
                                              rospy.Time(0))
    torso_pose = geom_msgs.Pose()
    torso_pose.position = geom_msgs.Point(*t1)
    torso_pose.orientation = geom_msgs.Quaternion(*q1)
    return torso_pose

  def updateSkinState(self, skin_data):
    self.updateEndEffectorJacobian()
    self.updateContactJacobians(skin_data)

  ## Store latest joint states from the specified robot class
  # @var joint_names: Joint names
  # @var joint_angles: Joint angles
  # @var joint_velocities: Joint velocities
  # @var joint_stiffness: Joint stiffness
  # @var joint_damping: Joint damping
  # @var q_des: Desired joint angles
  def updateJointStates(self):
    self.joint_names = self.robot.get_joint_names()
    self.joint_angles = self.robot.get_joint_angles()
    self.joint_stiffness, self.joint_damping = self.robot.get_joint_impedance()
    self.joint_velocities = self.robot.get_joint_velocities()
    q_des = self.robot.get_ep()
    if q_des != None:
      self.desired_joint_angles = q_des

  # Compute and store the end effector position, orientation, and jacobian
  # from the current joint angles.
  def updateEndEffectorPose(self):
    pos, rot = self.robot.kinematics.FK(self.joint_angles)
    self.end_effector_position = pos
    self.end_effector_orient_cart = rot
    self.end_effector_orient_quat = tr.matrix_to_quaternion(rot)

  # Build and publish the haptic state message.
  def publishRobotState(self):
    self.updateEndEffectorJacobian()

  ## Modify taxel data for PR2 specific situations
  # TODO SURVY
  # @param skin_data Dictionary containing taxel array messages indexed by topic name
  def modifyPR2Taxels(self, skin_data):
    #print "modifyPR2Taxels"
    return skin_data

  ## Modifies data from the taxel array based on robot specific configurations.
  # An example of this is ignoring the PR2 wrist taxels when the wrist
  # is near its joint limit as the wrist itself will trigger the taxel.
  # @param skin_data Dict containing taxel array messages indexed by topic name
  # @return skin_data Modified dictionary containing taxel array messages
  def modifyRobotSpecificTaxels(self, skin_data):
    if self.opt.robot == 'pr2':
      return self.modifyPR2Taxels(skin_data)
    return skin_data # If this is running on a differen robot, don't modify the data.

  ## Build and publish the haptic state message.
  def publishRobotState(self):
    msg = haptic_msgs.RobotHapticState()

    msg.header = self.getMessageHeader()
    msg.header.frame_id = self.torso_frame

    # TODO LOCKING
    # Joint states
    self.updateJointStates()
    msg.joint_names = self.joint_names
    msg.joint_angles = self.joint_angles
    msg.desired_joint_angles = self.desired_joint_angles
    msg.joint_velocities = self.joint_velocities
    msg.joint_stiffness = self.joint_stiffness
    msg.joint_damping = self.joint_damping

    msg.torso_pose = self.updateTorsoPose()

    # End effector calculations
    self.updateEndEffectorPose()
    msg.hand_pose.position = geom_msgs.Point(*self.end_effector_position)
    msg.hand_pose.orientation = geom_msgs.Quaternion(*self.end_effector_orient_quat)

    skin_data = self.skin_client.getTrimmedSkinData()
    msg.skins = skin_data.values() # List of TaxelArray messages
    self.updateSkinState(skin_data)
    #if skin_data:
      #print "self.Jc:"
      #print self.Jc
      #print "skin_data"
      #print skin_data
      #print "len(Jc): %s, len(skin_data): %s" % (len(self.Jc), len(msg.skins[0].centers_x))
      #if len(self.Jc) != len(msg.skins[0].centers_x):
      #  rospy.spin()

    msg.end_effector_jacobian = self.ma_to_m.matrixListToMultiarray(self.Je)

    # Skin sensor calculations.
    # Get the latest skin data from the skin client
    skin_data = self.skin_client.getTrimmedSkinData()
    # Trim skin_data based on specific robot state (eg wrist configuration).
    skin_data = self.modifyRobotSpecificTaxels(skin_data)
    # Add the list of  TaxelArray messages to the message
    msg.skins = skin_data.values()
    self.updateContactJacobians(skin_data)
    msg.contact_jacobians = self.ma_to_m.matrixListToMultiarray(self.Jc)

    # Publish the newly formed state message
    self.state_pub.publish(msg)
    # Publish gripper pose for debug purposes
    ps_msg = geom_msgs.PoseStamped()
    ps_msg.header = self.getMessageHeader()
    ps_msg.header.frame_id = self.torso_frame

    ps_msg.pose.position = geom_msgs.Point(*self.end_effector_position)
    ps_msg.pose.orientation = geom_msgs.Quaternion(*self.end_effector_orient_quat)
    self.gripper_pose_pub.publish(ps_msg)

  ## Start the state publisher
  def start(self):
    rospy.loginfo("RobotHapticState: Starting Robot Haptic State publisher")
    rate = rospy.Rate(self.rate) # 100Hz, nominally.

    # Blocking sleep to prevent the node publishing until joint states
    # are read by the robot client.
    rospy.loginfo("RobotHapticState: Waiting for robot state")
    joint_stiffness, joint_damping = self.robot.get_joint_impedance()
    while (self.robot.get_joint_angles() == None or
           self.robot.get_joint_velocities() == None or
           joint_stiffness == None):
      joint_stiffness, joint_damping = self.robot.get_joint_impedance()
      rate.sleep()
    rospy.loginfo("RobotHapticState: Got robot state")

    if self.robot.get_ep() == None:
      rospy.loginfo("RobotHapticState: Setting desired joint angles to current joint_angles")
      self.robot.set_ep(self.robot.get_joint_angles())

    rospy.loginfo("RobotHapticState: Starting publishing")
    while not rospy.is_shutdown():
      self.publishRobotState()
#      rospy.spin() # Blocking spin for debug/dev purposes
      rate.sleep()


if __name__ == "__main__":
  # Parse an options list specifying robot type
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)

  if not opt.robot or not opt.sensor or not opt.arm:
    p.error("Robot haptic state publisher requires a specified robot, sensor, AND arm to use.")

  robot_state = RobotHapticStateServer(opt, "robot_haptic_state_server")
  robot_state.start()
