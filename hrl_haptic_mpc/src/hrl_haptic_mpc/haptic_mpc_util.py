#   Copyright 2013 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

## @package hrl_haptic_mpc
#
# @author Jeff Hawke
# @version 0.1
# @copyright Apache 2.0

# Useful utility functions used by the Haptic MPC controller and associated nodes.
# Functions generally are used to reformat data structures.

import numpy as np
import hrl_lib.transforms as tr
import hrl_lib.util as ut
import decimal as dec


## Return a taxel array message data as lists (formats it nicely for iteration).
# @param ta_msg TaxelArray message object
# @retval locations_list List of taxel locations, specified as numpy matrices: [x,y,z].T
# @retval normals_list List of normal vectors, specified as numpy matrices: [x,y,z].T
# @retval values_list List of force (or distance) magnitudes, specified as numpy matrices: [x,y,z].T
# @retval joints_list List of taxel locations, specified as numpy matrices: [x,y,z].T
def getTaxelArrayDataAsLists(ta_msg):
  normals_list = []
  locations_list = []
  values_list = []
  joints_list = []
  for i in range(0, len(ta_msg.normals_x)):
    normal_vector = np.matrix([ta_msg.normals_x[i], ta_msg.normals_y[i], ta_msg.normals_z[i]]).T
    normals_list.append(normal_vector)
    point_vector = np.matrix([ta_msg.centers_x[i], ta_msg.centers_y[i], ta_msg.centers_z[i]]).T
    locations_list.append(point_vector)
    values_vector = np.matrix([ta_msg.values_x[i], ta_msg.values_y[i], ta_msg.values_z[i]]).T
    values_list.append(values_vector)
    joints_list.append(ta_msg.link_names[i])
  return locations_list, normals_list, values_list, joints_list

## Return a taxel array normal vector data as a list (formats it nicely for iteration).
# @param ta_msg TaxelArray message object
# @retval normals_list List of normal vectors, specified as numpy matrices: [x,y,z].T
def getNormalsFromTaxelArray(ta_msg):
  normals_list = []
  for i in range(0, len(ta_msg.normals_x)):
    normal_vector = np.matrix([ta_msg.normals_x[i], ta_msg.normals_y[i], ta_msg.normals_z[i]]).T
    normals_list.append(normal_vector)  
  return normals_list

## Return a taxel array force location vector data as a list (formats it nicely for iteration).
# @param ta_msg TaxelArray message object
# @retval locations_list List of contact locations, specified as numpy matrices: [x,y,z].T
def getLocationsFromTaxelArray(ta_msg):
  points_list = []
  for i in range(0, len(ta_msg.centers_x)):
    point_vector = np.matrix([ta_msg.centers_x[i], ta_msg.centers_y[i], ta_msg.centers_z[i]]).T
    points_list.append(point_vector)
  return points_list

## Return a taxel array force magnitude vector data as a list (formats it nicely for iteration).
# @param ta_msg TaxelArray message object
# @retval values_list List of taxel values (eg, force or proximity), specified as numpy matrices: [x,y,z].T
def getValuesFromTaxelArray(ta_msg):
  values_list = []
  for i in range(0, len(ta_msg.values_x)):
    values_vector = np.matrix([ta_msg.values_x[i], ta_msg.values_y[i], ta_msg.values_z[i]]).T
    values_list.append(values_vector)
  return values_list

## Return a taxel array force magnitude vector data as a list (formats it nicely for iteration).
# @param ta_msg_list A list of TaxelArray message objects.
# @retval locations_list List of taxel locations, specified as numpy matrices: [x,y,z].T
# @retval normals_list List of normal vectors, specified as numpy matrices: [x,y,z].T
# @retval values_list List of force (or distance) magnitudes, specified as numpy matrices: [x,y,z].T
# @retval joints_list List of taxel locations, specified as numpy matrices: [x,y,z].T
def getAllTaxelArrayDataAsLists(ta_msg_list):
  locations_list = []
  normals_list = []
  values_list = []
  joints_list = []
  for taxel_array in ta_msg_list:
    ll, nl, vl, jl = getTaxelArrayDataAsLists(taxel_array)
    locations_list.extend(ll)
    normals_list.extend(nl)
    values_list.extend(vl)
    joints_list.extend(jl)
  return locations_list, normals_list, values_list, joints_list


## Return a taxel array force magnitude vector data as a list (formats it nicely for iteration).
# @param ta_msg_list A list of TaxelArray message objects.
# @retval locations_list List of taxel locations, specified as numpy matrices: [x,y,z].T
def getLocations(ta_msg_list):
  locations_list = []
  for taxel_array in ta_msg_list:
    ll = getLocationsFromTaxelArray(taxel_array)
    locations_list.extend(ll)
  return locations_list

## Return a taxel array force magnitude vector data as a list (formats it nicely for iteration).
# @param ta_msg_list A list of TaxelArray message objects.
# @retval values_list List of force (or distance) magnitudes, specified as numpy matrices: [x,y,z].T
def getValues(ta_msg_list, force = True, distance = False):
  values_list = []
  for taxel_array in ta_msg_list:
    vl = getValuesFromTaxelArray(taxel_array)
    values_list.extend(vl)
  return values_list

## Return a taxel array force magnitude vector data as a list (formats it nicely for iteration).
# @param ta_msg_list A list of TaxelArray message objects.
# @retval normals_list List of normal vectors, specified as numpy matrices: [x,y,z].T
def getNormals(ta_msg_list):
  normals_list = []
  for taxel_array in ta_msg_list:
    nl = getNormalsFromTaxelArray(taxel_array)
    normals_list.extend(nl)
  return normals_list

## Return a skew matrix as a numpy matrix given an vector (n=3) input.
# @param vec List of length 3
# @return 3x3 numpy skew matrix
def getSkewMatrix(vec):
    return np.matrix([[0, -vec[2], vec[1]],
                      [vec[2], 0, -vec[0]], 
                      [-vec[1], vec[0], 0]])



## Interpolate a step towards the given goal orientation.
# @param q_h_orient The current hand orientation as a quaternion in list form: [x,y,z,w]
# @param q_g_orient The current goal orientation as a quaternion in list form: [x,y,z,w]
# @return Desired change in orientation as a delta
# @todo clean this up so it doesn't use the "step" multiplier
def goalOrientationInQuat(q_h_orient, q_g_orient, max_ang_step):
  ang = ut.quat_angle(q_h_orient, q_g_orient)

  ang_mag = abs(ang)
  step_fraction = 0.001
  if step_fraction * ang_mag > max_ang_step:
    # this is pretty much always true, can clean up the code.
    step_fraction = max_ang_step / ang_mag

  interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
  delta_q_des = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_h_orient))

  return np.matrix(delta_q_des[0:3]).T

## Return a skew matrix as a numpy matrix given an vector (n=3) input.
# Identical to getSkewMatrix - needs to be cleaned up.
# @param vec List of length 3
# @return 3x3 numpy skew matrix
def get_skew_matrix(vec):
  return np.matrix([[0, -vec[2], vec[1]],
                    [vec[2], 0, -vec[0]], 
                    [-vec[1], vec[0], 0]])

## Sets up options for an optparse.OptionParser. These parameters are used across multiple classes - eg, robot haptic state publisher, arm trajectory generator, etc
# @param p optparse.OptionParser object
def initialiseOptParser(p):
  p.add_option('--ignore_skin', '--is', action='store_true', dest='ignore_skin',
               help='ignore feedback from tactile skin')
  p.add_option('--orientation', '-o', action='store_true', dest='orientation',
               help='try to go to commanded orientation in addition to position')
  p.add_option('--robot', '-r', action='store', dest='robot',
               help='robot name: cody, cody5dof, pr2, sim3, sim3_with_hand, simcody')
  p.add_option('--sensor', '-s', action='store', dest='sensor',
               help='sensor type: meka_sensor, fabric_sensor, hil, ignore')
  p.add_option('--use_wrist_joints', action='store_true', dest='use_wrist_joints',
               help='use wrist joints (Cody ONLY)')
  p.add_option('--arm_to_use', '-a', action='store', dest='arm', type='string',
               help='which arm to use (l, r)', default=None)
  p.add_option('--start_test', action='store_true', dest='start_test',
               help='makes cody move to specified jep if true')

## Takes an optparse.OptionParser as an input, and returns a valid opt structure (or flags an error).
# @param p optparse.OptionParser object
# @return Validated options from parsed input.
def getValidInput(p):
  opt, args = p.parse_args()
  # Validate input options
  valid_robots = ['cody', 'cody5dof', 'pr2', 'sim3', 'sim3_with_hand', 'simcody', 'crona']
  valid_sensors = ['meka', 'fabric', 'hil', 'none', 'pps', 'sim']

  if opt.robot and opt.robot not in valid_robots:
    p.error("Must specify a valid robot type: -r %s" % str(valid_robots))
  if opt.sensor and opt.sensor not in valid_sensors:
    p.error("Must specify a valid sensor type: -s %s" % str(valid_sensors))
  if opt.robot and opt.sensor and opt.robot == 'pr2' and (opt.sensor == 'meka' or opt.sensor == 'hil'):
    p.error("Invalid sensor for the PR2 [meka, hil]. Valid options: -s [fabric, pps, none]")  
  return opt

## Get number of decimal points in a float.
# @param float_value The number being evaluated for the number of decimal places
def getNumDecimalPlaces(float_value):
  return abs(dec.Decimal(str(float_value)).as_tuple().exponent)

## Rounds the value provided to the nearest 'unit'
# @param value The number being rounded
# @param unit The step size to round the value to.
def divRound(value, unit):
  return round(round(value/unit)*unit, getNumDecimalPlaces(value))
