# To get stuff ready for RFID particle filter to run...

open rviz with robot_traj.vcg

show_robot.launch

# Build up tf from the actual home by playing back a bagfile in rfid_datacapture (eg. search_cap)
rosbag play --clock search_aware_home/woot_150_8.bag

# During the playback:
roslaunch rfid_pf pf_costmap.launch

# Bring up rosclock
roslaunch sim_search gazebo.launch

# Generate the costmap filter ( cset[:,2] = 1.0 ==> Good, 0.0 ==> Bad )
python gen_costmap_locations.py --recalc
