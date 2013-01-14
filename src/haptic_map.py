#!/usr/bin/env python

import roslib
roslib.load_manifest("hrl_tactile_controller")
import rospy

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

import haptic_mpc_util

import threading

class HapticMapBuilder:
  def __init__(self):
    print "init"
    self.frequency = 1.0 # Hz
    
    self.point_set = set() # list of Point32 objects
    self.grid_size = 0.01 
    
    self.point_lock = threading.RLock()

  ## Returns a header type with the current timestamp. Does not set the frame_id
  def getMessageHeader(self):
    header = std_msgs.msg.Header()
    header.stamp = rospy.get_rostime()
    header.frame_id = '/torso_lift_link'
    return header
  
  ## Store the robot haptic state.
  # @param msg RobotHapticState message object
  def robotStateCallback(self, msg):
    with self.point_lock:
      self.last_msg_time = rospy.Time.now() # timeout for the controller
      
      self.msg = msg
      self.skin_data = msg.skins

      for taxel_array in msg.skins: # Iterate over all taxelarrays
        for i in range(0, len(taxel_array.centers_x)): # And all values in each array
          point_tuple = (taxel_array.centers_x[i], taxel_array.centers_y[i], taxel_array.centers_z[i])
          self.addPointTuple(point_tuple) 
  
  ## Convert a geometry_msgs.msg.Point32 msg object to a tuple
  def point32ToPointTuple(self, point32):
    return (point32.x, point32.y, point32.z)
  
  ## Convert a tuple (x,y,z) to a geometry_msgs.msg.Point32
  def pointTupleToPoint32(self, point_tuple):
    point = geometry_msgs.msg.Point32()
    point.x = point_tuple[0]
    point.y = point_tuple[1]
    point.z = point_tuple[2]
    return point  

  ## Round a point tuple (x,y,z) to a specified grid size
  def roundPointTuple(self, point_tuple, grid_size):
    x = haptic_mpc_util.divRound(point_tuple[0], grid_size)
    y = haptic_mpc_util.divRound(point_tuple[1], grid_size)
    z = haptic_mpc_util.divRound(point_tuple[2], grid_size)
    return (x,y,z) 
  
  ## Round a geometry_msgs.msg.Point32 (or Point) to a specified grid size
  def roundPoint32(self, point, grid_size):
    point.x = haptic_mpc_util.divRound(point.x, grid_size)
    point.y = haptic_mpc_util.divRound(point.y, grid_size)
    point.z = haptic_mpc_util.divRound(point.z, grid_size)
    return point
  
  ## Add a point to the buffer. Needs to be a geometry_msgs.msg.Point32.
  # @param point geometry_msgs.msg.Point32 message object
  # @param value Value to be added to the reward channel for the pointcloud. Defaults to 1 as it's not used.
  def addPoint32(self, point32):
    point32 = self.roundPoint32(point32, self.grid_size)
    self.point_set.add(self.point32ToTuple(point32))  
  
  ## Add a point to the buffer. Needs to be a tuple of (x,y,z).
  # @param point_tuple Point specified as a tuple: (x,y,z) 
  # @param value Value to be added to the reward channel for the pointcloud. Defaults to 1 as it's not used.
  def addPointTuple(self, point_tuple):
    point = self.roundPointTuple(point_tuple, self.grid_size)
    self.point_set.add(point)  
  
  def publishMap(self):
    #rospy.loginfo("Publishing map")
    msg = sensor_msgs.msg.PointCloud()
    msg.header = self.getMessageHeader()
    
    print "&&&&&&&\nBUFFER!!:"
    print msg.header
    print self.point_set
    
    with self.point_lock:
      for point in self.point_set:
        msg.points.append(self.pointTupleToPoint32(point))
    
    print "**************\nMESSAGE:"
    print msg
    self.haptic_map_pub.publish(msg)
  
  def initComms(self, node_name="haptic_map_builder"):
    rospy.init_node(node_name)
  
    self.robot_state_sub = rospy.Subscriber("/haptic_mpc/robot_state", haptic_msgs.RobotHapticState, self.robotStateCallback)
    self.haptic_map_pub = rospy.Publisher("/haptic_mpc/contact_map", sensor_msgs.msg.PointCloud)
  
  def start(self):
    self.initComms()
    
    rospy.loginfo("HapticMapBuilder started.")
    rate = rospy.Rate(self.frequency)
    while not rospy.is_shutdown():
      self.publishMap()
      rate.sleep() 

if __name__=="__main__":
  haptic_map = HapticMapBuilder()
  haptic_map.start()


