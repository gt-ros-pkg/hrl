

roslaunch rfid_demos hfa_study.launch


# Assign Ground Truth Locations for each pt.
[rfid_datacapture] ./cap_ground_truth.py


FOR i IN 0:tags
    # NOTE: Object #0 on point #i (others circular shift)

    # Search for tags -- woot_150_i.bag will contain the output bag file
    [rfid_datacapture] ./sm_explore_capture.py --fname woot_150_i --radius 1.5 

    # Optional (backup files in search_aware_home/)
    [rfid_datacapture/search_cap/search_aware_home] cp * /removable/tdeyle/swh_backup/
    
    For j in 0:objects
    	# Go capture up-close information.
	# Will need to manually position to exact start (then Ctrl+C due to latch)
	# Will also need to Ctrl+C at end when printing the tf transforms
    	./servo_cap_script.sh j i 


    

    

# Robot Trajectory Plot:
open rviz with robot_traj.vcg
show_robot.launch
rosbag play --clock search_aware_home/woot_150_8.bag
python publish_trajectory.py --trial 4
capture screenshot
python publish_readings.py --trial 6 --obj 0

python publish_readings.py --trial 9 --obj 1
CLICK RVIZ, WAIT FOR SCREENSHOTS.


For the "Robot View" (Kinect data)
open rviz with robot_view.vcg
python publish_robotpose.py --trial 0 --obj 0 --sc
python publish_robotpose.py --trial 9 --obj 0 --sc  (for all automated)


# For PARTICLE FILTER use:
python process_servo_reads.py


# Plots of final robot poses:
rosbag play --clock search_aware_home/woot_150_8.bag
python display_tags_and_poses.py
(rfid_pf) python gen_costmap_locations.py --fname pf_costmap.pkl
