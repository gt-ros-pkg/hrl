## @package hrl_haptic_mpc
# 
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('hrl_haptic_mpc')
import rospy

import skin_client as sc


## PR2 specific implementation of the TaxelArrayClient class
#
# Adds one function which needs to know about PR2 specific parameters.
# This structure isn't fantastic and will eventually be pulled into the
# robot haptic state node as it's only used to form the contact jacobians.
class PR2_SkinClient(sc.TaxelArrayClient):
  def __init__(self, skin_topic_list, torso_frame, tf_listener=None):
    sc.TaxelArrayClient.__init__(self, skin_topic_list, torso_frame, tf_listener)

  ## Returns a list of taxel locations and a list of joint numbers after which the
  # joint torque will have no effect on the contact force
  # @param skin_data Dictionary of TaxelArrays indexed by topic
  # @retval locations List of taxel locations where a force is present
  # @retval joint_nums List of joints after which the joint torque will have no effect on the contact force 
  # @return These arrays will both be the same length (as the joint number corresponds 
  def getTaxelLocationAndJointList(self, skin_data):
    locations = []
    joint_nums = []
    
    for ta_msg in skin_data.values():
      # Get points list
      ta_locs = self.getContactLocationsFromTaxelArray(ta_msg)
      # Create list of joints beyond which the joint torque has no effect on contact force
      ta_jts = []
      for contact_index in range(len(ta_msg.centers_x)):
        jt_num = 6
        
        if len(ta_msg.link_names) < len(ta_msg.centers_x):
          ta_jts.append(jt_num)
          continue
        
        link_name = ta_msg.link_names[contact_index] 

        if 'shoulder_pan' in link_name:
          jt_num = 0
        elif 'shoulder_lift' in link_name:
          jt_num = 1
        elif 'upper_arm' in link_name:
          jt_num = 2
        elif 'elbow' in link_name:
          jt_num = 3
        elif 'forearm' in link_name:
          jt_num = 4
        elif 'wrist_flex' in link_name:
          jt_num = 5    
        ta_jts.append(jt_num)
          
      # Attach these lists to the end of the global list (incorporating multiple taxel arrays)
      locations.extend(ta_locs)
      joint_nums.extend(ta_jts)
      
    return locations, joint_nums




