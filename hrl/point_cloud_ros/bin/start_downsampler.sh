#!/bin/bash

#rosrun point_cloud_mapping normal_estimation_node _downsample_leaf_width_x:=0.05 _downsample_leaf_width_y:=0.05 _downsample_leaf_width_z:=0.05
rosrun point_cloud_mapping cloud_downsampler_node _leaf_width_x:=0.01 _leaf_width_y:=0.01 _leaf_width_z:=0.01







