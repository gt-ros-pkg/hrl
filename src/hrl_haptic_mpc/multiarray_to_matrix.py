#!/usr/bin/env python

## @package hrl_haptic_mpc
# 
# @author Jeff Hawke jhawke@gatech.edu
# @version 0.1
# @copyright Simplified BSD Licence

import roslib
roslib.load_manifest('std_msgs')
import rospy

import std_msgs.msg

import numpy

## Helper class to convert numpy matrix lists to and from a Float64MultiArray message object. 
class MultiArrayConverter():
   
  ## Take a Float64 MultiArray message, convert it into a list of numpy matrices
  def multiArrayToMatrixList(self, ma_msg):
    dim = len(ma_msg.layout.dim)
    offset = ma_msg.layout.data_offset
    
    if dim != 3:
      print "Error: Must be 3 dimensions"
    
    if (ma_msg.layout.dim[0].label != "matrix"):
      print "Error: dim[0] should be the matrices"
    num_matrices = ma_msg.layout.dim[0].size
    
    if (ma_msg.layout.dim[1].label != "row"):
      print "Error: dim[1] should be the rows"
    rows = ma_msg.layout.dim[1].size
    
    if (ma_msg.layout.dim[2].label != "column"):
      print "Error: dim[2] should be the columns"
    columns = ma_msg.layout.dim[2].size
    
    ## Initialise empty matrix based on number of row/columns.
    #
    # NB: THIS IS ASSUMED TO BE CONSTANT FOR A GIVEN MESSAGE (or the multiarray structure breaks)
    mat = numpy.matrix(numpy.empty([rows,columns]))
    mat.fill(numpy.nan)
    
    matrix_list = [mat]*num_matrices
    
    for i in range(0, num_matrices):
      for j in range(0, rows):
        for k in range(0, columns):
          matrix = matrix_list[i]
          data_index = ma_msg.layout.data_offset + (rows*columns) * i + (columns) * j + k
          matrix[j,k] = ma_msg.data[data_index] 
    
    return matrix_list
    
    
  ## Convert a list of 2D numpy matrices to a Float64MultiArray message
  #
  # Assumption: Each numpy matrix must be the same size (rows, columns)
  def matrixListToMultiarray(self, matrix_list):
    num_matrices = len(matrix_list)
    if num_matrices > 0:
      rows, columns = matrix_list[0].shape
    else:
      rows = 0
      columns = 0
    
    msg = std_msgs.msg.Float64MultiArray()
    
    # Set up layout
    msg.layout.data_offset = 0
    
    # Set up layout dimensions
    matrix_dim = std_msgs.msg.MultiArrayDimension()
    matrix_dim.label = "matrix"
    matrix_dim.size = num_matrices
    matrix_dim.stride = columns * rows * num_matrices
    
    row_dim = std_msgs.msg.MultiArrayDimension()
    row_dim.label = "row"
    row_dim.size = rows
    row_dim.stride = columns * rows
    
    col_dim = std_msgs.msg.MultiArrayDimension()
    col_dim.label = "column"
    col_dim.size = columns
    col_dim.stride = columns
    
    msg.layout.dim = [matrix_dim, row_dim, col_dim]
    
    # Copy data from matrices into the msg data block
    msg_data = [float('NaN')] * (msg.layout.data_offset + num_matrices * rows * columns)
    
    for i in range(0, num_matrices):
      for j in range(0, rows):
        for k in range(0, columns):
          #print "k: %s" % str(k)
          matrix = matrix_list[i]
          data_index = msg.layout.data_offset + (rows*columns) * i + (columns) * j + k
          msg_data[data_index] = matrix[j,k]
          
    msg.data = msg_data
    
    return msg
    
    
    